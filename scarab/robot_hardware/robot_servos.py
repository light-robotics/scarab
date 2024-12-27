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
        current_position = RobotPosition(
            leg1_gamma=self.m1.read_angle(2),
            leg1_beta=self.m1.read_angle(3),
            leg1_alpha=self.m1.read_angle(4),
            leg1_tetta=self.m1.read_angle(5),

            leg2_gamma=self.m4.read_angle(8),
            leg2_beta=self.m4.read_angle(9),
            leg2_alpha=self.m4.read_angle(10),
            leg2_tetta=self.m4.read_angle(11),

            leg3_gamma=self.m4.read_angle(14),
            leg3_beta=self.m4.read_angle(15),
            leg3_alpha=self.m4.read_angle(16),
            leg3_tetta=self.m4.read_angle(17),

            leg4_gamma=self.m1.read_angle(20),
            leg4_beta=self.m1.read_angle(21),
            leg4_alpha=self.m1.read_angle(22),
            leg4_tetta=self.m1.read_angle(23),
        )
        
        self.logger.info(f'Read current angles : {current_position}')
        
        return current_position
    
    def angles_are_close(self, target_angles):
        """
        compares self angles to target angles
        if they are different, return false
        """
        current_angles = self.get_current_angles()
        """
        j = 1
        for m in [self.m1, self.m2, self.m3, self.m4]:            
            for _ in range(4):
                current_angles.append(m.readAngle(j))
                j += 1
        print('Current angles :')
        print(current_angles)
        """
        for i in range(16):
            if abs(current_angles[i] - target_angles[i]) > 2:
                print('Angles {0} diff too big. {1}, {2}'.format(i, current_angles[i], target_angles[i]))
                return False

        return True

    def enable_torque(self):
        for i in config.servos_boards.keys():
            board = self.servo_controller(i)
            board.enable_torque(i)

    def disable_torque(self):
        for i in config.servos_boards.keys():
            board = self.servo_controller(i)
            board.disable_torque(i)

    def set_servo_values(self, angles, rate=0):
        print('Sending values \n{0}'.format(angles))
        #self.get_angles_diff(angles)
        j = 1
        for m in [self.m1, self.m2, self.m3, self.m4]:
            for _ in range(4):
                m.move_servo_to_angle(j, angles[j-1], rate)
                j += 1
    
    def send_command_to_servos(self, rp: RobotPosition, rate):
        for joint, servo_value in rp.servo_values:
            servo_num = config.servos_mapping[joint]
            sc = self.servo_controller(servo_num)
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
        wait_time = max(0, self.speed / 1000 - config.fenix['movement_command_advance_ms'])
        self.logger.info(f'Wait time : {wait_time}, speed : {int(self.speed * 0.9)}')
        time.sleep(wait_time)

    def set_servo_values_not_paced_v2(self, fp: RobotPosition, prev_fp: RobotPosition = None):
        # every command is executed over a computed time, depending on the angle
        _, max_angle_diff = self.get_angles_diff(fp, prev_fp)
        rate = round(max(self.speed * max_angle_diff / 45, self.max_speed)) # speed is normalized
        wait_time = max(0, rate / 1000 - config.fenix['movement_command_advance_ms'])

        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        self.logger.info(f'Wait time : {wait_time}')
        
        self.send_command_to_servos(fp, rate)
        
        time.sleep(wait_time)
        self.logger.info(f'[DIFF] Diff with target:')
        self.get_angles_diff(fp)

    def set_servo_values_overshoot(self, angles, prev_angles=None):
        angles_diff, max_angle_diff = self.get_angles_diff(angles, prev_angles)
        rate = round(max(self.speed * (1 + config.fenix['movement_overshoot_coefficient']) * max_angle_diff / 45, self.max_speed)) # speed is normalized
        wait_time = max(0, rate / 1000 - config.fenix['movement_command_advance_ms'])

        self.logger.info(f'max_angle_diff: {max_angle_diff}, self.speed : {self.speed}, self.speed * max_angle_diff / 45 : {self.speed * max_angle_diff / 45}')
        self.logger.info(f'Wait time : {wait_time}')

        adjusted_angles = [round(target + (config.fenix['movement_overshoot_coefficient'] * diff), 1) for target, diff in zip(angles, angles_diff)]

        self.logger.info(f'{angles} ->\n{adjusted_angles}')
        
        self.send_command_to_servos(adjusted_angles, rate)
        
        time.sleep(wait_time)
        self.logger.info(f'[DIFF] Diff from target:')
        self.get_angles_diff(angles)
    
    def get_angles_diff(self, target_position: RobotPosition, test_position: RobotPosition = None):
        if test_position is None:
            test_position = self.get_current_angles()

        angles_diff = []
        for current, target in zip(test_position.to_servo(), target_position.to_servo()):
            angles_diff.append(round(current - target, 2))
        max_angle_diff = max([abs(x) for x in angles_diff])
        self.logger.info(f'[DIFF] Max : {max_angle_diff}. Avg : {sum([abs(x) for x in angles_diff])/16}. Sum : {sum([abs(x) for x in angles_diff])}')
        return angles_diff, max_angle_diff


if __name__ == '__main__':
    scarab = RobotServos()

    """
    fnx.set_speed(2000)
    sequence = [
        [-65.37, 30.75, -34.63, -10.55, -22.04, 37.07, 15.02, 18.02, -15.2, 42.89, 3.69, -0.26, -18.72, 38.35, 13.62, -20.17],
        [44.51882068166497, 14.398429391637588, -82.79813097435527, -20.637939780612253, -26.762858610560755, 12.719663051904275, -128.64048416277242, 26.401895199628335, -135.24095796267952, 4.079459501331462, -131.7573745682841, 13.68223214772406, 114.59728860411596, 9.837685342396234, -134.63935227779214, 33.598245106471474]
    ]
    # 19.42853486276713, -74.1493912084663, -65.2791436543008   
    for angles in sequence:     
        fnx.set_servo_values_paced(angles)
    
    #time.sleep(2)
    fnx.disable_torque()
    """
    scarab.print_status()

    rp = RobotPosition(
        l1a=0, l1b=0, l1t=0, 
        l2a=0, l2b=0, l2t=0, 
        l3a=0, l3b=0, l3t=0, 
        l4a=0, l4b=0, l4t=0, 
        l5a=0, l5b=0, l5t=0, 
        l6a=0, l6b=0, l6t=0
    )
    scarab.send_command_to_servos(rp, 2000)
    time.sleep(2)
    scarab.disable_torque()
