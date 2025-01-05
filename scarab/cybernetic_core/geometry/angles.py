import math
from functools import lru_cache
from typing import List

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from cybernetic_core.geometry.lines import Point

from configs.config import leg, servos_mapping
import configs.code_config as code_config
import logging.config

class DistanceException(Exception):
    pass

class AnglesException(Exception):
    pass

class GeometryException(Exception):
    pass

class RobotPositionLeg():
    def __init__(self, alpha, beta, tetta):
        self.alpha = alpha
        self.beta = beta
        self.tetta = tetta
    
    def __repr__(self):
        return f'a: {self.alpha}, b: {self.beta}, t: {self.tetta}'

class RobotPosition():
    valid_fields = (
        "l1a", "l1b", "l1t", 
        "l2a", "l2b", "l2t", 
        "l3a", "l3b", "l3t", 
        "l4a", "l4b", "l4t", 
        "l5a", "l5b", "l5t", 
        "l6a", "l6b", "l6t"
    )
    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            if k not in self.valid_fields:
                raise TypeError(f'Wrong attribute: {k}')
            self.__setattr__(k, v)
        #leg1 = RobotPositionLeg(self.l1a, self.l1b, self.l1t)
        #leg2 = RobotPositionLeg(self.l2a, self.l2b, self.l2t)
        #leg3 = RobotPositionLeg(self.l3a, self.l3b, self.l3t)
        #leg4 = RobotPositionLeg(self.l4a, self.l4b, self.l4t)
        #leg5 = RobotPositionLeg(self.l5a, self.l5b, self.l5t)
        #leg6 = RobotPositionLeg(self.l6a, self.l6b, self.l6t)
                           
        #self.legs = {
        #    1: leg1, 2: leg2, 3: leg3, 4: leg4, 5: leg5, 6: leg6,
        #}
    
    @property
    def servo_values(self):
        return self.__dict__

    def __hash__(self):
        return hash(self.servo_values.values())

    def __repr__(self):
        return self.__dict__.__repr__()

    #def __repr__(self):
    #    return f'\n1: {self.legs[1]}\n2: {self.legs[2]}\n3: {self.legs[3]}\n4: {self.legs[4]}\n5: {self.legs[5]}\n6: {self.legs[6]}\n'

def build_position_from_servos(servo_angles: List[float]) -> RobotPosition:
    # incoming angles: beta, alpha, tetta for leg 1 to 6
    beta1, alpha1, tetta1, beta2, alpha2, tetta2, beta3, alpha3, tetta3, beta4, alpha4, tetta4, beta5, alpha5, tetta5, beta6, alpha6, tetta6 = servo_angles
    return RobotPosition(
        leg1_alpha=alpha1,
        leg1_beta=beta1,
        leg1_tetta=tetta1,

        leg2_alpha=alpha2,
        leg2_beta=beta2,
        leg2_tetta=tetta2,

        leg3_alpha=alpha3,
        leg3_beta=beta3,
        leg3_tetta=tetta3,

        leg4_alpha=alpha4,
        leg4_beta=beta4,
        leg4_tetta=tetta4,

        leg5_alpha=alpha5,
        leg5_beta=beta5,
        leg5_tetta=tetta5,

        leg6_alpha=alpha6,
        leg6_beta=beta6,
        leg6_tetta=tetta6,
    )

@lru_cache(maxsize=None)
def find_angles(Cx, Cy, logger):
    a, b = leg.a, leg.b
    dist = math.sqrt(Cx ** 2 + Cy ** 2)
    if dist > a + b:
        raise Exception('No decisions. Full distance : {0}'.format(dist))

    alpha1 = math.acos((a ** 2 + dist ** 2 - b ** 2) / (2 * a * dist))
    beta1 = math.acos((a ** 2 + b ** 2 - dist ** 2) / (2 * a * b))
    beta = math.pi - beta1

    alpha2 = math.atan2(Cy, Cx)
    alpha = alpha1 + alpha2

    logger.info(f'Alpha1: {round(math.degrees(alpha1), 2)}\nAlpha2: {round(math.degrees(alpha2), 2)}\nBeta1: {round(math.degrees(beta1), 2)}')

    return alpha, beta

def calculate_leg_angles(C: Point, logger):    
    tetta = math.atan2(C.y, C.x)

    l = round(math.sqrt(C.x ** 2 + C.y ** 2), 2)
    delta_z = round(C.z, 2)
    #print(f'Trying l {l} and delta_z {delta_z}')
    alpha, beta = find_angles(l, delta_z, logger)
    #print(f'Success : {[math.degrees(tetta), math.degrees(alpha), math.degrees(beta)]}')

    return tetta, alpha, beta

def calculate_C_point(tetta: float, alpha: float, beta: float) -> Point:
    B_xz = [leg.a * math.cos(alpha),
            leg.a * math.sin(alpha)]
    C_xz = [B_xz[0] + leg.b * math.cos(alpha - beta),
            B_xz[1] + leg.b * math.sin(alpha - beta)]

    C = Point(round(C_xz[0] * math.cos(tetta), 2),
                    round(C_xz[0] * math.sin(tetta), 2),
                    round(C_xz[1], 2))

    return C

def convert_alpha(alpha: float) -> float:
    alpha_converted = round(math.degrees(alpha), 2)
    return alpha_converted

def convert_alpha_to_kinematic(alpha_deg: float) -> float:
    return round(math.radians(alpha_deg), 4)

def convert_beta(beta: float) -> float:
    return round(math.degrees(beta) - 90, 2)

def convert_beta_to_kinematic(beta_deg: float) -> float:
    return round(math.radians(beta_deg + 90), 4)

def convert_tetta(tetta: float, leg_number: int) -> float:
    # virtual model to real servos
    tetta_degrees = math.degrees(tetta)
    leg_number = int(leg_number)
    if leg_number == 1:
        tetta_degrees = 90 - tetta_degrees
    elif leg_number == 6:
        tetta_degrees = - (90 + tetta_degrees)
    elif leg_number == 2:
        tetta_degrees = 90 - tetta_degrees
    elif leg_number == 3:
        tetta_degrees = 90 - tetta_degrees
    elif leg_number == 4:
        tetta_degrees = - (tetta_degrees + 90)
    elif leg_number == 5:
        tetta_degrees = - (tetta_degrees + 90)
            
    return round(tetta_degrees, 2)

def convert_tetta_to_kinematic(tetta_deg: float, leg_number: int) -> float:
    # real servos to virtual model
    # WTF!
    if leg_number in [4, 5, 6]:
        tetta_deg -= 180
    
    tetta_radians = math.radians(tetta_deg)
    
    return round(tetta_radians, 4)

def get_converter_function(joint: str, back=False):
    if 'a' in joint:
        if back:
            return convert_alpha_to_kinematic
        else:
            return convert_alpha
    if 'b' in joint:
        if back:
            return convert_beta_to_kinematic
        else:
            return convert_beta
    if 't' in joint:
        if back:
            return convert_tetta_to_kinematic
        else:
            return convert_tetta

def convert_legs_angles_C(rp_in: RobotPosition, logger=None) -> RobotPosition:
    angles = {}
    for joint, servo_value in rp_in.servo_values.items():
        converter_func = get_converter_function(joint)
        if 't' in joint:
            leg_num = joint[1]
            angles[joint] = converter_func(servo_value, leg_num)
        else:
            angles[joint] = converter_func(servo_value)

    return RobotPosition(**angles)

def convert_legs_angles_to_kinematic_C(rp_in: RobotPosition, logger=None) -> RobotPosition:
    angles = {}
    for joint, servo_value in rp_in.servo_values.items():
        converter_func = get_converter_function(joint, back=True)
        if 't' in joint:
            leg_num = joint[1]
            angles[joint] = converter_func(servo_value, leg_num)
        else:
            angles[joint] = converter_func(servo_value)

    return RobotPosition(**angles)

# ----------------------
# moves for Fenix
def get_angle_by_coords(x1, y1):
    l = math.sqrt(x1 ** 2 + y1 ** 2)
    initial_angle = math.asin(abs(y1) / l)
    if x1 >= 0 and y1 >= 0:
        return initial_angle
    if x1 >= 0 and y1 < 0:
        return 2 * math.pi - initial_angle
    if x1 < 0 and y1 >= 0:
        return math.pi - initial_angle
    if x1 < 0 and y1 < 0:
        return initial_angle + math.pi

def turn_on_angle(start_x, start_y, x1, y1, angle):
    print(f'x1, y1 : {round(x1, 2)}, {round(y1, 2)}')
    l = math.sqrt((x1 - start_x) ** 2 + (y1 - start_y) ** 2)
    initial_angle = get_angle_by_coords((x1 - start_x), (y1 - start_y))
    result_angle = angle + initial_angle
    print(f'{math.degrees(initial_angle)} -> {math.degrees(result_angle)}')

    return round(start_x + math.cos(result_angle) * l, 2), \
           round(start_y + math.sin(result_angle) * l, 2)


if __name__ == '__main__':
    logging.config.dictConfig(code_config.logger_config)
    logger = logging.getLogger('main_logger')

    #fp = RobotPosition('physical', 44.51882068166497, 14.398429391637588, -82.79813097435527, -20.637939780612253, -26.762858610560755, 12.719663051904275, -128.64048416277242, 26.401895199628335, -135.24095796267952, 4.079459501331462, -131.7573745682841, 13.68223214772406, 114.59728860411596, 9.837685342396234, -134.63935227779214, 33.598245106471474)
    #print(fp.legs[1].alpha)
    # DeltaX: 16.71. DeltaZ: 12.72
    print(find_angles(16.71, 12.72, logger))
