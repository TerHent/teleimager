import os
import sys
import yaml
import socket
import threading
import queue
import time

# --- Setup Paths ---
this_dir = os.path.dirname(os.path.abspath(__file__))
# Add teleimager to sys.path
teleimager_src = os.path.normpath(os.path.join(this_dir, "src"))
if os.path.exists(teleimager_src) and teleimager_src not in sys.path:
    sys.path.insert(0, teleimager_src)

from teleimager.image_server import ImageServer

try:
    from dynamixel_sdk import PortHandler, PacketHandler
except ImportError:
    print("[ERROR] dynamixel_sdk is not installed. Run: pip install dynamixel-sdk")
    raise

# --- Servo Driver Code (from head_servo_control_server_calibrated.py) ---
class CalibratedServoDriver:
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    PROTOCOL_VERSION = 2.0
    ID_YAW = 0
    ID_PITCH = 1

    # Safety limits matching G1 custom head calibration
    YAW_NEUTRAL = 2950
    YAW_MIN = 2300
    YAW_MAX = 3600

    PITCH_NEUTRAL = 1500
    PITCH_MIN = 1100
    PITCH_MAX = 1900

    def __init__(self, device_name, baudrate=1000000):
        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        if not self.portHandler.openPort():
            raise IOError(f"Failed to open port {device_name}")
        if not self.portHandler.setBaudRate(baudrate):
            raise IOError(f"Failed to set baudrate to {baudrate}")
        
        print("[SERVO] Rebooting servos to clear potential hardware error states...")
        self.packetHandler.reboot(self.portHandler, self.ID_YAW)
        self.packetHandler.reboot(self.portHandler, self.ID_PITCH)
        time.sleep(0.5)

        self.enable_torque()
        # Move to center starting position
        self.set_goal_position(self.ID_YAW, self.YAW_NEUTRAL)
        self.set_goal_position(self.ID_PITCH, self.PITCH_NEUTRAL)

    def _check_result(self, dxl_comm_result, dxl_error, message=""):
        if dxl_comm_result != 0:
            print(f"[SERVO ERROR] {message}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        if dxl_error != 0:
            print(f"[SERVO ERROR] {message}: {self.packetHandler.getRxPacketError(dxl_error)}")
            return False
        return True

    def enable_torque(self):
        print("[SERVO] Enabling torque...")
        res1, err1 = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID_YAW, self.ADDR_TORQUE_ENABLE, 1)
        self._check_result(res1, err1, f"Torque Enable ID:{self.ID_YAW}")
        res2, err2 = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID_PITCH, self.ADDR_TORQUE_ENABLE, 1)
        self._check_result(res2, err2, f"Torque Enable ID:{self.ID_PITCH}")

    def disable_torque(self):
        print("[SERVO] Disabling torque...")
        # Move to center before disabling torque to prevent flopping
        self.set_goal_position(self.ID_YAW, self.YAW_NEUTRAL)
        self.set_goal_position(self.ID_PITCH, self.PITCH_NEUTRAL)
        time.sleep(0.5)

        res1, err1 = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID_YAW, self.ADDR_TORQUE_ENABLE, 0)
        self._check_result(res1, err1, f"Torque Disable ID:{self.ID_YAW}")
        res2, err2 = self.packetHandler.write1ByteTxRx(self.portHandler, self.ID_PITCH, self.ADDR_TORQUE_ENABLE, 0)
        self._check_result(res2, err2, f"Torque Disable ID:{self.ID_PITCH}")
        self.portHandler.closePort()

    def set_goal_position(self, motor_id, position):
        if motor_id == self.ID_YAW:
            position = max(self.YAW_MIN, min(int(position), self.YAW_MAX))
        elif motor_id == self.ID_PITCH:
            position = max(self.PITCH_MIN, min(int(position), self.PITCH_MAX))
        
        res, err = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, self.ADDR_GOAL_POSITION, position)
        self._check_result(res, err, f"Set Goal Pos ID:{motor_id} to {position}")

    def handle_command(self, command):
        try:
            motor_str, position_str = command.strip().split(':')
            position_1023 = int(position_str)
            motor_str = motor_str.upper()
            
            if motor_str == 'YAW':
                offset = (position_1023 - 512) * (650.0 / 511.5)
                position_4095 = int(self.YAW_NEUTRAL + offset)
                self.set_goal_position(self.ID_YAW, position_4095)
            elif motor_str == 'PITCH':
                offset = (position_1023 - 512) * (400.0 / 511.5)
                position_4095 = int(self.PITCH_NEUTRAL + offset)
                self.set_goal_position(self.ID_PITCH, position_4095)
        except ValueError as e:
            print(f"[SERVO] Could not parse command '{command}'. Error: {e}")

# --- Socket Server Code ---
def client_handler(conn, addr, command_queue):
    print(f"[SERVO] Accepted connection from {addr}")
    buffer = ""
    with conn:
        while True:
            try:
                data = conn.recv(1024)
                if not data: break
                buffer += data.decode('utf-8')
                while '\n' in buffer:
                    command, buffer = buffer.split('\n', 1)
                    if command:
                        command_queue.put(command)
            except ConnectionResetError: break
    print(f"[SERVO] Connection from {addr} closed.")

def start_servo_server(command_queue, host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen()
        print(f"[SERVO] Driver listening on {host}:{port}")
        while True:
            conn, addr = s.accept()
            thread = threading.Thread(target=client_handler, args=(conn, addr, command_queue))
            thread.daemon = True
            thread.start()

# --- Main Combined Runner ---
def main():
    # 1. Start Image Stream Server
    config_path = os.path.join(this_dir, "head_teleop_config.yaml")
    print(f"[Combined Server] Loading camera configuration from: {config_path}")
    try:
        with open(config_path, "r") as f:
            cam_config = yaml.safe_load(f)
    except Exception as e:
        print(f"[Combined Server] Error loading config file: {e}")
        return

    print("[Combined Server] Starting ImageServer...")
    img_server = ImageServer(cam_config, realsense_enable=True, camera_finder_verbose=False)
    img_server.start()

    # 2. Start Motor Servo Server
    print("[Combined Server] Initializing Motor Servo Driver...")
    driver = None
    command_queue = queue.Queue()
    try:
        # Default serial port is /dev/ttyUSB0, default TCP port is 9999
        driver = CalibratedServoDriver(device_name="/dev/ttyUSB2")
        
        server_thread = threading.Thread(target=start_servo_server, args=(command_queue, '0.0.0.0', 9999))
        server_thread.daemon = True
        server_thread.start()

        print("[Combined Server] Both Camera and Servo servers are running. Listening for commands...")
        while True:
            try:
                command = command_queue.get(timeout=0.1)
                print(f"[SERVO] Executing command: '{command}'")
                driver.handle_command(command)
            except queue.Empty:
                pass
    except KeyboardInterrupt:
        print("\n[Combined Server] Interrupted by user. Shutting down...")
    except Exception as e:
        print(f"[Combined Server] Error in servo control: {e}")
    finally:
        if driver:
            try:
                driver.disable_torque()
            except Exception as e:
                print(f"[Combined Server] Error disabling torque: {e}")
        try:
            print("[Combined Server] Stopping camera server...")
            img_server._stop_event.set()
            img_server._clean_up()
        except Exception as e:
            print(f"[Combined Server] Error cleaning up image server: {e}")
        print("[Combined Server] Shutdown complete.")

if __name__ == "__main__":
    main()
