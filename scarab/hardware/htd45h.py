#!/usr/bin/python3
import time
from serial import Serial, SerialException
import struct
from typing import Union, List
import sys
import os
import subprocess
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import configs.code_config as code_config
import logging.config


neutral = {
    
    3 : 435,
    4 : 610,
    5 : 500,
    8 : 280,
    13 : 305,
    14 : 570,
    9 : 480,
    10 : 550,
    11 : 500,
    2 :  480,
    12 : 330,
    15 : 470,
    16 : 560,
    17 : 500,
    20 : 250,
    21 : 480,
    22 : 560,
    23 : 500
}

class HTD45H:

    LED_OFF = 1
    LED_ON  = 0

    LED_ERROR_NONE                         = 0
    LED_ERROR_OVER_TEMPERATURE             = 1
    LED_ERROR_OVER_VOLTAGE                 = 2
    LED_ERROR_OVER_TEMPERATURE_AND_VOLTAGE = 3
    LED_ERROR_LOCK_ROTOR                   = 4
    LED_ERROR_OVER_TEMPERATE_AND_STALLED   = 5
    LED_ERROR_OVER_VOLTAGE_AND_STALLED     = 6
    LED_ERROR_OVER_ALL                     = 7

    SERVO_FRAME_HEADER         = 0x55
    SERVO_MOVE_TIME_WRITE      = 1
    SERVO_MOVE_TIME_READ       = 2
    SERVO_MOVE_TIME_WAIT_WRITE = 7
    SERVO_MOVE_TIME_WAIT_READ  = 8
    SERVO_MOVE_START           = 11
    SERVO_MOVE_STOP            = 12
    SERVO_ID_WRITE             = 13
    SERVO_ID_READ              = 14
    SERVO_ANGLE_OFFSET_ADJUST  = 17
    SERVO_ANGLE_OFFSET_WRITE   = 18
    SERVO_ANGLE_OFFSET_READ    = 19
    SERVO_ANGLE_LIMIT_WRITE    = 20
    SERVO_ANGLE_LIMIT_READ     = 21
    SERVO_VIN_LIMIT_WRITE      = 22
    SERVO_VIN_LIMIT_READ       = 23
    SERVO_TEMP_MAX_LIMIT_WRITE = 24
    SERVO_TEMP_MAX_LIMIT_READ  = 25
    SERVO_TEMP_READ            = 26
    SERVO_VIN_READ             = 27
    SERVO_POS_READ             = 28
    SERVO_OR_MOTOR_MODE_WRITE  = 29
    SERVO_OR_MOTOR_MODE_READ   = 30
    SERVO_LOAD_OR_UNLOAD_WRITE = 31
    SERVO_LOAD_OR_UNLOAD_READ  = 32
    SERVO_LED_CTRL_WRITE       = 33
    SERVO_LED_CTRL_READ        = 34
    SERVO_LED_ERROR_WRITE      = 35
    SERVO_LED_ERROR_READ       = 36

    def __init__(self, port: str = "/dev/ttyUSB0", Baudrate: int = 115200, Timeout: float = 0.001):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('main_logger')
        self.serial = Serial(port, baudrate=Baudrate, timeout=Timeout)
        self.serial.setDTR(1)
        self.TX_DELAY_TIME = 0.00002 
        self.Header = struct.pack("<BB",0x55,0x55)
        self.port = port

    def reset(self) -> None:
        print('............. Resetting .............')
        time.sleep(0.1)
        self.serial = Serial(self.port, baudrate=115200, timeout=0.001)
        self.serial.setDTR(1)
        time.sleep(0.1)

    # send the packet with header and checksum
    def send_packet(self, packet: bytes) -> None:
        sum = 0
        for item in packet:
            sum = sum + item
        full_packet = bytearray(self.Header + packet + struct.pack("<B",(~sum) & 0xff))
 
        for i in range(3):
            try:
                self.serial.write(full_packet)
                break
            except SerialException as e:
                print(f'!!!!!!!! Send packet Serial ALARM {i+1} !!!!!!!!!!!!!!!!!!!!!')
                print(f'Serial exception: {e}')
            time.sleep(2)
            #os.system("sudo chmod 666 /dev/ttyAMA0")
            subprocess.check_output("sudo chmod 666 /dev/ttyAMA0", shell=True)
            time.sleep(2)
            try:
                self.reset()
            except SerialException as e:
                print('Reset failed')

        time.sleep(self.TX_DELAY_TIME)

    # send packet and return response
    def send_receive_packet(self, packet: bytes, receive_size: int) -> bytes:
        num_attempts = 3
        for _ in range(num_attempts):
            # t_id = packet[0]     
            # t_command = packet[2]
            self.serial.flushInput()
            self.serial.timeout = 0.1
            self.send_packet(packet)
            r_packet = self.read(receive_size + 3)
            if len(r_packet) > 0:
                return r_packet
        raise Exception('Got empty response in {0} attempts'.format(num_attempts))
    
    def read(self, size):
        for i in range(3):
            try:
                r_packet = self.serial.read(size)
                return r_packet
            except SerialException as e:
                print(f'!!!!!!!!!!! READ Serial ALARM {i+1} !!!!!!!!!!!!!!!!!!!!!')
                print(f'Serial exception: {e}')
            time.sleep(2)
            #os.system("sudo chmod 666 /dev/ttyAMA0")
            subprocess.check_output("sudo chmod 666 /dev/ttyAMA0", shell=True)
            time.sleep(2)
            try:
                self.reset()
            except SerialException as e:
                print('Reset failed')
        
    # Move servo from 0 to 1000 (0.24 degree resolution)
    # rate means speed, 0 is instant, 
    # 1000 means it will take 1 second to make a move, 
    # 30000 is very slow
    def move_servo(self, id: int, position: int, rate: int = 1000) -> None:
        packet = struct.pack(
            "<BBBHH",
            id,
            7,
            self.SERVO_MOVE_TIME_WRITE,
            position,
            rate
        )
        self.send_packet(packet)

    # several attempts are made to send servo to a certain angle
    # because sometimes command does not work and target stays unchanged
    def move_servo_to_angle(self, id: int, angle: float, rate: int = 5000) -> None:
        position = neutral[id] + int(angle/0.24)
        self.logger.info(f'Id : {id}. Target required : {position}. Angle: {angle}. Rate: {rate}')
        if position < 0:
            self.logger.error(f'Id : {id}. Target required : {position}. Angle: {angle}. Rate: {rate}')
            position = 0
        num_attempts = 3
        for i in range(num_attempts):
            try:
                packet = struct.pack(
                    "<BBBHH",
                    id,
                    7,
                    self.SERVO_MOVE_TIME_WRITE,
                    position,
                    rate
                )
                self.send_packet(packet)
                target = self.read_servo_target(id)[0]
                if target != position:
                    self.logger.info(f'Id : {id}. Target required : {position}. Target real : {target}')
                    continue
                break
            except Exception as e:
                self.logger.info(f'{i} attempt failed for servo {id}. Position: {position}. Angle: {angle}.\n{e}')
      
    # read target position and rate
    def read_servo_target(self, id: int) -> Union[int, int]:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_MOVE_TIME_READ
        )
        rpacket = self.send_receive_packet(packet, 7)
        s = struct.unpack("<BBBBBHHB", rpacket)
     
        return s[5:7]

    # Move servo to position at rate
    # Waiting for the command SERVO_MOVE_STOP
    def move_servo_wait(self, id: int, position: int, rate: int = 1000) -> None:
        packet = struct.pack(
            "<BBBHH",
            id,
            7,
            self.SERVO_MOVE_TIME_WAIT_WRITE,
            position,
            rate
        )
        self.send_packet(packet)

    # Read the angle and the rate send by move_servo_wait
    def read_servo_target_wait(self, id: int) -> Union[int, int]:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_MOVE_TIME_WAIT_READ
        )
        rpacket = self.send_receive_packet(packet, 7)
        s = struct.unpack("<BBBBBHHB", rpacket)
        return s[5:7]

    # Start a command from move_servo_wait
    def move_servo_start(self, id: int) -> None:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_MOVE_START
        )
        self.send_packet(packet)

    # Stop a command from move_servo_wait
    def move_servo_stop(self, id: int) -> None:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_MOVE_STOP
        )
        self.send_packet(packet)

    # change the ID of servo
    def set_id(self, id: int, newid: int) -> None:
        packet = struct.pack(
            "<BBBB",
            id,
            4,
            self.SERVO_ID_WRITE,
            newid
        )
        self.send_packet(packet)

    # read the servo ID
    def read_id(self, id: int) -> int:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_ID_READ
        )
        rpacket = self.send_receive_packet(packet, 4)
        s = struct.unpack("<BBBBBBB", rpacket)
        return s[5]

    # Change the angle offset without saving it during the next power ON
    # Angle between -125 and 125
    def set_angle_offset_adjust(self, id: int, angle: float) -> None:
        packet = struct.pack(
            "<BBBb",
            id,
            4,
            self.SERVO_ANGLE_OFFSET_ADJUST,
            angle
        )
        self.send_packet(packet)

    # Change the angle offset permanently
    # Angle between -125 and 125
    def set_angle_offset(self, id:int, angle: int) -> None:
        packet = struct.pack(
            "<BBBb",
            id,
            4,
            self.SERVO_ANGLE_OFFSET_WRITE,
            angle
        )
        self.send_packet(packet)

    #lire l'offset de l'angle
    #angle entre -125 et 125
    def read_angle_offset(self,id):
        packet = struct.pack("<BBB",id,3,self.SERVO_ANGLE_OFFSET_READ)
        rpacket = self.send_receive_packet(packet,4)
        s = struct.unpack("<BBBBBbB",rpacket)
        return s[5]

    # Define the minimum and maximum angle of the servo
    # Angle is between 0 and 1000 with resolution of 0.24 degrees
    def set_angle_limit(self, id: int, angle_min: int, angle_max: int) -> None:
        packet = struct.pack(
            "<BBBHH",
            id,
            7,
            self.SERVO_ANGLE_LIMIT_WRITE,
            angle_min,
            angle_max
        )
        self.send_packet(packet)

    # Read the minimum and maximum limit of the allowed angle
    def read_angle_limit(self, id: int) -> Union[int, int]:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_ANGLE_LIMIT_READ
        )
        rpacket = self.send_receive_packet(packet, 7)
        s = struct.unpack("<BBBBBHHB", rpacket)
        return s[5:7]

    # define the minimum and maximum operating voltage of the servo
    # the values are in mv, min = 6500 max = 10000
    def set_voltage_limit(self, id: int, voltage_min: int, voltage_max: int) -> None:
        packet = struct.pack(
            "<BBBHH",
            id,
            7,
            self.SERVO_VIN_LIMIT_WRITE,
            voltage_min,
            voltage_max
        )
        self.send_packet(packet)
    
    # Read the minimum and maximum operating voltage of the servo
    # the values are in mv, min = 6500 max = 10000
    def read_voltage_limit(self, id: int) -> Union[int, int]:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_VIN_LIMIT_READ
        )
        rpacket = self.send_receive_packet(packet, 7)
        s = struct.unpack("<BBBBBHHB", rpacket)
        return s[5:7]

    # Set the maximum operating temperature in celsius
    # default is 85 celsius, min = 50 and max = 100
    def set_temperature_limit(self, id: int, temperature_max: int) -> None:
        packet = struct.pack(
            "<BBBB",
            id,
            4,
            self.SERVO_TEMP_MAX_LIMIT_WRITE,
            temperature_max
        )
        self.send_packet(packet)

    # Read the maximum temperature limit in celsius
    def read_temperature_limit(self, id: int) -> int:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_TEMP_MAX_LIMIT_READ
        )
        rpacket = self.send_receive_packet(packet, 4)
        s = struct.unpack("<BBBBBBB", rpacket)
        return s[5]

    # Read the temperature in celsius
    def read_temperature(self, id: int) -> int:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_TEMP_READ
        )
        rpacket = self.send_receive_packet(packet, 4)
        s = struct.unpack("<BBBBBBB", rpacket)
        return s[5]

    # Read the servo supply voltage in mv
    def read_voltage(self, id: int) -> int:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_VIN_READ
        )
        rpacket = self.send_receive_packet(packet, 5)
        s = struct.unpack("<BBBBBHB", rpacket)
        return s[5]

    # Read servo position
    # the value can be negative then it is signed short
    def read_position(self, id: int) -> int:        
        num_attempts = 3
        for attempt in range(num_attempts):
            try:
                packet = struct.pack(
                    "<BBB",
                    id,
                    3,
                    self.SERVO_POS_READ
                )
                rpacket = self.send_receive_packet(packet, 5)
                s = struct.unpack("<BBBBBhB", rpacket)
                return s[5]
            except Exception as e:
                self.logger.info(f'Can not read values from servo {id}. Attempt {attempt}. \nException : {e}')
                self.reset()
            
        raise Exception('Can not read values from servo {0}'.format(id))

    def read_angle(self, id):
        num_attempts = 5
        for i in range(num_attempts):
            angle = round((self.read_position(id) - neutral[id]) * 0.24 , 2)
            if -150 <= angle <= 150:
                return angle
            self.logger.info(f'Attempt to read angle from servo {id} failed. Value : {angle}')
        raise Exception(f'Could not get correct angle from servo {id} in {num_attempts} attempts.')

    # Motor movement with speed : motor_mode = 1 motor_speed = rate
    # Otherwise set servo mode  : motor_mode = 0 
    def motor_or_servo(self, id: int, motor_mode: int, motor_speed: int) -> None:
        packet = struct.pack(
            "<BBBBBh",
            id,
            7,
            self.SERVO_OR_MOTOR_MODE_WRITE,
            motor_mode,
            0,
            motor_speed
        )
        self.send_packet(packet)

    # Read the mode of servo
    def read_motor_or_servo(self, id: int) -> List[int]:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_OR_MOTOR_MODE_READ
        )
        rpacket = self.send_receive_packet(packet, 7)
        s = struct.unpack("<BBBBBBBhB", rpacket)
        return [s[5],s[7]]

    # Activate or deactivate the engine
    # 0 = motor OFF, 1 = motor ON
    def load_unload(self, id: int, mode: int) -> None:
        packet = struct.pack(
            "<BBBB",
            id,
            4,
            self.SERVO_LOAD_OR_UNLOAD_WRITE,
            mode
        )
        self.send_packet(packet)

    # Read the status of the servo activation
    def read_load_unload(self, id: int) -> int:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_LOAD_OR_UNLOAD_READ
        )
        rpacket = self.send_receive_packet(packet, 4)
        s = struct.unpack("<BBBBBBB", rpacket)
        return s[5]

    # Turn LED on/off
    # 0 = on  => self.LED_ON
    # 1 = OFF => self.LED_OFF
    def set_led(self, id: int, led_state: int) -> None:
        packet = struct.pack(
            "<BBBB",
            id,
            4,
            self.SERVO_LED_CTRL_WRITE,
            led_state
        )
        self.send_packet(packet)

    # Read the status of the LED
    # 0 = LED active
    # 1 = LED OFF
    def read_led(self, id: int) -> int:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_LED_CTRL_READ
        )
        rpacket = self.send_receive_packet(packet, 4)
        s = struct.unpack("<BBBBBBB", rpacket)
        return s[5]

    # Activate an error on the alarm led
    def set_led_error(self, id: int, led_error: int):
        packet = struct.pack(
            "<BBBB",
            id,
            4,
            self.SERVO_LED_ERROR_WRITE,
            led_error
        )
        self.send_packet(packet)

    def read_led_error(self, id: int) -> int:
        packet = struct.pack(
            "<BBB",
            id,
            3,
            self.SERVO_LED_ERROR_READ
        )
        rpacket = self.send_receive_packet(packet, 4)
        s = struct.unpack("<BBBBBBB", rpacket)
        return s[5]
   
    def read_values(self, id):
        try:
            pos = self.read_position(id)
            temp = self.read_temperature(id)
            volt = self.read_voltage(id)
            target = self.read_servo_target(id)
            print('{5:2d}. Pos: {0:5d}. Target: {1:5d}, {2:5d}. Temp: {3:5d}. Volt: {4:5d}'
                  .format(pos, target[0], target[1], temp, volt, id))
        except:
            print('Could not read values from servo {0}'.format(id))

    @staticmethod
    def _checksum(packet: list[int]) -> int:
        s = ~sum(packet[2:])
        return s % 256
    
    def _send_packet(self, packet: list[int]) -> None:
        packet = [0x55, 0x55, *packet]
        packet.append(self._checksum(packet))
        for i in range(3):
            try:
                self.serial.write(packet)
                break
            except SerialException as e:
                print(f'!!!!!!!! _Send packet Serial ALARM {i+1} !!!!!!!!!!!!!!!!!!!!!')
                print(f'Serial exception: {e}')
            time.sleep(2)
            #os.system("sudo chmod 666 /dev/ttyAMA0")
            subprocess.check_output("sudo chmod 666 /dev/ttyAMA0", shell=True)
            time.sleep(2)
            try:
                self.reset()
            except SerialException as e:
                print('Reset failed')
                #self.serial.write(packet)

    def enable_torque(self, id) -> None:
        packet = [id, 4, 31, 1]
        self._send_packet(packet)
        self._torque_enabled = True

    def disable_torque(self, id) -> None:
        print('Disable torque command received')
        packet = [id, 4, 31, 0]
        self._send_packet(packet)
        self._torque_enabled = False


def get_position(servo, id):
    pos = servo.read_position(id)
    target = servo.read_servo_target(id)
    return 'Angle: {0:5.2f}. Pos: {1:3d}. Target: {2:3d}'.format(convert_position_to_angle(neutral[id], pos), pos, target[0])


def get_state(servo, id):
    pos = servo.read_position(id)
    temp = servo.read_temperature(id)
    volt = servo.read_voltage(id)
    target = servo.read_servo_target(id)
    return 'Pos: {0:5d}. Angle: {5:5.2f}. Target: {1:5d}, {2:5d}. Temp: {3:5d}. Volt: {4:5d}'.format(pos, target[0], target[1], temp, volt, convert_position_to_angle(neutral[id], pos))

def convert_position_to_angle(start, value):
    return round((value - start) * 0.24 , 2)

def read_values(m0, servo):
    try:
        pos = m0.read_position(servo)
        temp = m0.read_temperature(servo)
        volt = m0.read_voltage(servo)
        target = m0.read_servo_target(servo)
        overheating = 'OK'
        if temp > 60:
            overheating = 'Critical'
        elif temp > 55:
            overheating = 'Danger'
        elif temp > 50:
            overheating = 'Warning'
        return '{5:2d}. Pos: {0:5d}. Target: {1:5d}, {2:3d}. Temp: {3:5d}. Volt: {4:5d}. Overheating: {6}'.format(
            pos, target[0], target[1], temp, volt, servo, overheating)
    except:
        return 'Could not read values from servo {0}'.format(servo)


if __name__ == '__main__':      
    #m1 = HTD45H(port='/dev/ttyAMA0') # 5-8   # 1-4
    #m2 = HTD45H(port='/dev/ttyAMA2') # 9-12  # 5-8
    m3 = HTD45H(port='/dev/ttyAMA3') # 13-16 # 9-12
    m4 = HTD45H(port='/dev/ttyAMA4') # 1-4   # 13-16
    
    #for i in range(24):
    #    m3.read_values(i)
    #    time.sleep(0.0002)

    for i in [3, 4, 5, 8, 13, 14, 21, 22, 23]:
        m3.read_values(i)
        time.sleep(0.0002)
    for i in [2, 9, 10, 11, 12, 15, 16, 17, 20]:
        m4.read_values(i)
        time.sleep(0.0002)

    #m3.set_id(1, 8)
    #test_servo = 17
    #m4.move_servo_to_angle(test_servo, 30, 3000)
    #time.sleep(3)
    #m3.disable_torque(test_servo)
    