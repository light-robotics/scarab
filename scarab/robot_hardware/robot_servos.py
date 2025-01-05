import time
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from hardware.htd45h import HTD45H, read_values
import logging
import configs.config as config
import configs.code_config as code_config
import logging.config
logging.config.dictConfig(code_config.logger_config)
from copy import deepcopy

from cybernetic_core.geometry.angles import RobotPosition, build_position_from_servos

class RobotServos:
    def __init__(self):
        self.m3 = HTD45H(port='/dev/ttyAMA3') # 5-8   # 1-4
        self.m4 = HTD45H(port='/dev/ttyAMA4') # 1-4   # 13-16
        self.speed = 500
        self.min_speed = 700
        self.max_speed = 0 # 130 # 0 is instant, 10000 is very slow
        self.diff_from_target_limit = config.robot.diff_from_target_limit # when it's time to start next movement
        self.diff_from_prev_limit = config.robot.diff_from_prev_limit # 1.0 # start next movement if we're stuck

        self.logger = logging.getLogger('main_logger')
        
        # 0.16 sec / 60 degrees for 7.4V+
        # 0.18 sec / 60 degrees for 6V+
        # my max speed is for 45 degrees
        # that means that max speed should be 120 for 7.4V+ and 135 for 6V+
        self.servos = [2, 3, 4, 5, 8, 9, 10, 11, 14, 15, 16, 17, 20, 21, 22, 23]

    def servo_controller(self, servo_number: int) -> HTD45H:
        return self.__getattribute__(f"m{config.servos_boards[servo_number]}")
    
    def print_status(self):
        for i in config.servos_boards.keys():
            board = self.servo_controller(i)
            board.read_values(i)
            time.sleep(0.0002)
    
    def set_speed(self, new_speed):
        if new_speed > 10000 or new_speed < self.max_speed:
            raise Exception(f'Invalid speed value {new_speed}. Should be between {self.max_speed} and 10000')
        self.speed = new_speed
        self.logger.info(f'RobotServos. Speed set to {self.speed}')
    
    def get_current_angles(self) -> RobotPosition:
        angles = {}
        for joint, servo_num in config.servos_mapping.items():
            sc = self.servo_controller(servo_num)
            angles[joint] = sc.read_angle(servo_num)

        current_position = RobotPosition(**angles)
        
        self.logger.info(f'Read current angles : {current_position}')
        
        return current_position
    
    def enable_torque(self):
        for i in config.servos_boards.keys():
            board = self.servo_controller(i)
            board.enable_torque(i)

    def disable_torque(self):
        for i in config.servos_boards.keys():
            board = self.servo_controller(i)
            board.disable_torque(i)

    def send_command_to_servos(self, rp: RobotPosition, rate):
        for joint, servo_value in rp.servo_values.items():
            servo_num = config.servos_mapping[joint]
            sc = self.servo_controller(servo_num)
            #if servo_num == 2:
            #    continue
            sc.move_servo_to_angle(servo_num, servo_value, rate)

    def set_servo_values_paced(self, angles):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        prev_angles = self.get_current_angles()

        self.send_command_to_servos(angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {angles}')
        #time.sleep(0.8 * rate / 1000)
        time.sleep(0.05)
        adjustment_done = False
        
        for s in range(50):
            self.logger.info(f'Step {s}')
            #with open("/fenix/fenix/wrk/gyroaccel_data.txt", "r") as f:
            #    ga_data = f.readline()
            #self.logger.info(f"GA_DATA: {ga_data}")
            #time.sleep(0.02)
            
            current_angles = self.get_current_angles()
            self.logger.info(f'current angles: {current_angles}')
            # if diff from prev angles or target angles is small - continue
            diff_from_target = self.get_angles_diff(angles, current_angles)
            diff_from_prev = self.get_angles_diff(current_angles, prev_angles)

            self.logger.info(f'Diff from prev  : {diff_from_prev[0]}')
            self.logger.info(f'Diff from target: {diff_from_target[0]}')
     
            if diff_from_target[1] < self.diff_from_target_limit:                
                self.logger.info(f'Ready to move further')
                break
            
            elif diff_from_prev[1] < self.diff_from_prev_limit and \
                    not adjustment_done:

                if diff_from_target[1] > 2 * self.diff_from_target_limit:
                    print('-----------ALARM-----------')
                    self.logger.info('-----------ALARM-----------')
                
                self.logger.info(f'Command sent : {angles}')
                if diff_from_target[1] > self.diff_from_target_limit * 3:
                    self.logger.info(f'We"re in trouble, too large diff : {diff_from_target[1]}')
                    break
                else:
                    #adjusted_angles = [round(target + (-1.5 * diff if abs(diff) > self.diff_from_target_limit else 0), 1) for target, diff in zip(angles, diff_from_target[0])]
                    adjusted_angles = [round(target + (-1.5 * diff), 1) for target, diff in zip(angles.to_servo(), diff_from_target[0])]
                    
                    self.logger.info(f'Adjusting to : {adjusted_angles}')
                    fp_adjusted = build_position_from_servos(adjusted_angles)
                    adjustment_done = True
                    self.send_command_to_servos(fp_adjusted, 0)
                    #time.sleep(0.03)
                    break

            elif diff_from_prev[1] < self.diff_from_prev_limit and \
                    adjustment_done:
                self.logger.info(f'Unreachable. Moving further')
                break

            prev_angles = deepcopy(current_angles)
    
    def set_servo_values_paced_wo_adjustment(self, angles):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        
        self.send_command_to_servos(angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {angles}')
        #time.sleep(0.8 * rate / 1000)
        time.sleep(0.05)

        prev_angles = self.get_current_angles()
        for s in range(50):
            self.logger.info(f'Step {s}')
            #time.sleep(0.02)
            
            current_angles = self.get_current_angles()
            self.logger.info(f'current angles: {current_angles}')
            # if diff from prev angles or target angles is small - continue
            diff_from_target = self.get_angles_diff(angles, current_angles)
            diff_from_prev = self.get_angles_diff(current_angles, prev_angles)

            self.logger.info(f'Diff from prev  : {diff_from_prev[0]}')
            self.logger.info(f'Diff from target: {diff_from_target[0]}')
     
            if diff_from_target[1] < self.diff_from_target_limit:                
                self.logger.info(f'Ready to move further')
                break
            
            elif diff_from_prev[1] < self.diff_from_prev_limit:
                self.logger.info(f'Unreachable. Moving further')
                break

            prev_angles = current_angles[:]

    def set_servo_values_paced_wo_feedback_w_adjustment(self, angles):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        
        current_angles = self.get_current_angles()
        self.logger.info(f'current angles: {current_angles}')

        diff_from_target = self.get_angles_diff(angles, current_angles)

        adjusted_angles = [round(target + (0.1 * diff), 1) for target, diff in zip(angles, diff_from_target[0])]

        self.send_command_to_servos(adjusted_angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {adjusted_angles}')
        time.sleep(rate / 1000)
    
    def set_servo_values_paced_wo_feedback(self, angles):
        _, max_angle_diff = self.get_angles_diff(angles)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        
        self.send_command_to_servos(angles, rate)
        self.logger.info(f'Command sent. Rate: {rate}, angles: {angles}')
        time.sleep(rate / 1000)
        return self.get_current_angles()
            
    def set_servo_values_not_paced(self, angles):
        # every command is executed over fixed time (1 sec for speed = 1000)
        self.send_command_to_servos(angles, int(self.speed * 0.9))
        wait_time = max(0, self.speed / 1000 - config.movement.command_advance_ms)
        self.logger.info(f'Wait time : {wait_time}, speed : {int(self.speed * 0.9)}')
        time.sleep(wait_time)

    def set_servo_values_for_running(self, angles, rate=config.speed.run):
        wait_time = max(0, rate / 1000 - config.movement.command_advance_ms)
        self.logger.info(f'Wait time : {wait_time}')
        
        self.send_command_to_servos(angles, rate)
        
        time.sleep(wait_time)
        self.logger.info(f'[DIFF] Diff from target:')
        self.get_angles_diff(angles)

    def set_servo_values_not_paced_v2(self, fp: RobotPosition, prev_fp: RobotPosition = None):
        # every command is executed over a computed time, depending on the angle
        _, max_angle_diff = self.get_angles_diff(fp, prev_fp)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        wait_time = max(0, rate / 1000)

        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        self.logger.info(f'Wait time : {wait_time}')
        
        self.send_command_to_servos(fp, rate)
        
        time.sleep(wait_time)
        self.logger.info(f'[DIFF] Diff with target:')
        self.get_angles_diff(fp)
    
    def get_angles_diff(self, target_position: RobotPosition, test_position: RobotPosition = None):
        if test_position is None:
            test_position = self.get_current_angles()

        angles_diff = []
        #for current, target in zip(test_position.to_servo(), target_position.to_servo()):
        for current, target in zip(
            test_position.servo_values.values(), 
            target_position.servo_values.values()
        ):
            angles_diff.append(round(current - target, 2))
        max_angle_diff = max([abs(x) for x in angles_diff])
        self.logger.info(f'[DIFF] Max : {max_angle_diff}. Avg : {sum([abs(x) for x in angles_diff])/16}. Sum : {sum([abs(x) for x in angles_diff])}')
        return angles_diff, max_angle_diff


if __name__ == '__main__':
    scarab = RobotServos()

    position = RobotPosition(**{'l1t': 45.0, 'l1a': -17.46, 'l1b': -16.84, 'l2t': 0.0, 'l2a': -19.71, 'l2b': -4.29, 'l3t': -45.0, 'l3a': -17.46, 'l3b': -16.84, 'l4t': 45.0, 'l4a': -17.46, 'l4b': -16.84, 'l5t': 0.0, 'l5a': -19.71, 'l5b': -4.29, 'l6t': -45.0, 'l6a': -17.46, 'l6b': -16.84})
    scarab.print_status()

    rp = RobotPosition(
        l1a=0, l1b=0, l1t=0, 
        l2a=0, l2b=0, l2t=0, 
        l3a=0, l3b=0, l3t=0, 
        l4a=0, l4b=0, l4t=0, 
        l5a=0, l5b=0, l5t=0, 
        l6a=0, l6b=0, l6t=0
    )
    """
    scarab.send_command_to_servos(position, 2000)
    time.sleep(2)
    scarab.disable_torque()
    scarab.get_current_angles()
    """