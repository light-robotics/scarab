import time
import datetime
import pickle
from typing import Callable, Optional, Union
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cybernetic_core.kinematics import Kinematics
from cybernetic_core.sequence_getter import VirtualRobot
from cybernetic_core.geometry.angles import AnglesException
from robot_hardware.robot_servos import RobotServos
from core.utils.multiphase_moves import CommandsForwarder
import configs.code_config as code_config
from configs import config as cfg
import logging.config
from copy import deepcopy


class MovementProcessor:
    def __init__(self):
        logging.config.dictConfig(code_config.logger_config)
        self.logger = logging.getLogger('main_logger')
        self.logger.info('==================START==================')

        self.max_processed_command_id = 0
        
        fk = Kinematics()
        self.vf = VirtualRobot(self.logger)
        self.cf = CommandsForwarder()
        self.rs = RobotServos()
        #self.ftfs = FenixTofs()
        #self.ftc = FenixTofCamera()
        
        self.robot_position = fk.current_position

        
        self.speed = 400
        self.body_speed = cfg.speed.body

    def read_command(self) -> Optional[Union[str, int]]:        
        with open(code_config.movement_command_file, 'r') as f:
            contents = f.readline().split(',')

        if len(contents) != 3:
            return None

        command_id = int(contents[0])
        command = contents[1].strip()

        # this commands are not excluded even if id is the same
        repeating_commands = [
            'forward_two_legged',
            'backward_two_legged',
            'strafe_left_two_legged',
            'strafe_right_two_legged', 
            'up', 
            'down',
            'body_forward',
            'body_backward',
            'body_left',
            'body_right',            
            'turn_left_two_legged',
            'turn_right_two_legged'
            ]        

        if self.max_processed_command_id == 0:
            self.max_processed_command_id = command_id
        elif self.max_processed_command_id == command_id and \
            command not in repeating_commands:
            # command has already been processed
            #print(f'Command {contents} has already been processed')
            return None

        self.max_processed_command_id = command_id
        return command, int(contents[2])

    def execute_command(self, command: str, speed: int, kwargs=None) -> None:
        if self.speed != speed:
            self.rs.set_speed(speed)
            self.speed = speed
            print(f'Setting speed to {speed}')

        # first we finish movements that are in progress
        if command in self.cf.moves:
            next_move = self.cf.get_move(command)
            print(f'Executing move: {next_move}')
            self.run_sequence(next_move)
        else:
            if self.cf.current_status:
                next_move = self.cf.get_move(command)
                print(f'Status: {self.cf.current_status}. Executing move: {next_move}')
                self.run_sequence(next_move)
            print(f'Executing move: {command}')
            if command == 'none':
                time.sleep(0.1)
            else:    
                self.run_sequence(command, kwargs)

    def move_function_dispatch(self, command: str) -> Callable:
        if command in ['hit_1', 'hit_2', 'forward_one_legged']:
            self.logger.info('Using function set_servo_values_paced')
            return self.rs.set_servo_values_paced
        elif command in ['forward_1', 'forward_2', 'forward_3', 'forward_22', 'forward_32']:
            self.logger.info('Using function set_servo_values_for_running')
            return self.rs.set_servo_values_for_running
            #return self.rs.set_servo_values_paced
        else:
            self.logger.info('Using function set_servo_values_not_paced_v2')
            return self.rs.set_servo_values_not_paced_v2
            #return self.rs.set_servo_values_paced
                        
    def run_sequence(self, command: str, kwargs=None) -> None:        
        self.logger.info(f'[MOVE] Started run_sequence : {datetime.datetime.now()}')
        try:            
            self.logger.info(f'MOVE. Trying command {command}')
            before_sequence_time = datetime.datetime.now()
            #sequence, new_position = get_sequence_for_command_cached(command, self.robot_position)
            sequence, new_position = self.vf.get_sequence(command, self.robot_position, kwargs)
            
            if sequence is None:
                self.logger.info(f'MOVE. Command aborted')
                return
            self.logger.info(f'[TIMING] Sequence calculation took : {datetime.datetime.now() - before_sequence_time}')
            self.robot_position = deepcopy(new_position)
        except (ValueError, AnglesException) as e:
            print(f'MOVE Failed. Could not process command - {str(e)}')
            self.logger.info(f'MOVE Failed. Could not process command - {str(e)}')
            time.sleep(0.3)
            return
        
        self.logger.info(f'[MOVE] Started: {datetime.datetime.now()}')    
        start_time = datetime.datetime.now()
        #prev_angles = None
        move_function = self.move_function_dispatch(command)

        original_move_function = move_function
        for move_snapshot in sequence:
            angles = move_snapshot.angles_snapshot
            #if move_snapshot.move_type == 'body' and self.speed != self.body_speed:
            #    self.rs.set_speed(self.body_speed)
            if move_snapshot.move_type == 'body':
                self.rs.set_speed(self.body_speed)
            else:
                self.rs.set_speed(self.speed)
                        
            if move_snapshot.move_type == 'touch':
                self.logger.info('[MP] Using function set_servo_values_touching')
                move_function = self.rs.set_servo_values_touching
            else:
                move_function = original_move_function

            self.logger.info(f'[MP] Moving to {angles}. Move type: {move_snapshot.move_type}')
            self.logger.info(f'Speed: {self.rs.speed}')
            move_function(angles)

        self.logger.info(f'[MOVE] finished: {datetime.datetime.now()}')
        self.logger.info(f'[TIMING] Step took : {datetime.datetime.now() - start_time}')

    def move(self):
        try:
            while True:
                command_read = self.read_command()
                
                if command_read is None:
                    time.sleep(0.1)
                    continue

                command, speed = command_read
                if command == 'exit':
                    break

                if command == 'disable_torque':
                    self.rs.disable_torque()
                    
                else:
                    #try:
                        self.execute_command(command, speed)
                    #except Exception as e:
                    #    self.rs.disable_torque()
                    #    print(e)
                        #raise Exception

        except KeyboardInterrupt:
            print('Movement stopped')

if __name__ == '__main__':
    MP = MovementProcessor()
    MP.move()
