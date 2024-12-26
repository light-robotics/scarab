import math
from functools import lru_cache
from typing import List

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from cybernetic_core.geometry.lines import Point

from configs.config import leg
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
    def __init__(self, 
            leg1_tetta,
            leg1_alpha,
            leg1_beta,

            leg2_tetta,
            leg2_alpha,
            leg2_beta,

            leg3_tetta,
            leg3_alpha,
            leg3_beta,

            leg4_tetta,
            leg4_alpha,
            leg4_beta,

            leg5_tetta,
            leg5_alpha,
            leg5_beta,

            leg6_tetta,
            leg6_alpha,
            leg6_beta
            ):
        leg1 = RobotPositionLeg(leg1_alpha, leg1_beta, leg1_tetta)
        leg2 = RobotPositionLeg(leg2_alpha, leg2_beta, leg2_tetta)
        leg3 = RobotPositionLeg(leg3_alpha, leg3_beta, leg3_tetta)
        leg4 = RobotPositionLeg(leg4_alpha, leg4_beta, leg4_tetta)
        leg5 = RobotPositionLeg(leg5_alpha, leg5_beta, leg5_tetta)
        leg6 = RobotPositionLeg(leg6_alpha, leg6_beta, leg6_tetta)
                           
        self.legs = {
            1: leg1, 2: leg2, 3: leg3, 4: leg4, 5: leg5, 6: leg6,
        }

    def to_servo(self):
        return [
            self.legs[1].beta, self.legs[1].alpha, self.legs[1].tetta,
            self.legs[2].beta, self.legs[2].alpha, self.legs[2].tetta,
            self.legs[3].beta, self.legs[3].alpha, self.legs[3].tetta,
            self.legs[4].beta, self.legs[4].alpha, self.legs[4].tetta,
            self.legs[5].beta, self.legs[5].alpha, self.legs[5].tetta,
            self.legs[6].beta, self.legs[6].alpha, self.legs[6].tetta,
        ]

    def __hash__(self):
        return hash(self.to_servo())

    def __repr__(self):
        return f'\n1: {self.legs[1]}\n2: {self.legs[2]}\n3: {self.legs[3]}\n4: {self.legs[4]}\n5: {self.legs[5]}\n6: {self.legs[6]}\n'

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

def calculate_leg_angles(O: Point, C: Point, logger):
    tetta = math.atan2(C.y - O.y, C.x - O.x)
    A = Point(O.x + leg.d * math.cos(tetta),
                O.y + leg.d * math.sin(tetta),
                O.z)
    logger.info(f'{math.degrees(tetta)}, {A}')

    l = round(math.sqrt((C.x - A.x) ** 2 + (C.y - A.y) ** 2), 2)
    delta_z = round(C.z - O.z, 2)
    logger.info(f'Trying l {l} and delta_z {delta_z}')
    alpha, beta = find_angles(l, delta_z, logger)
    logger.info(f'Success : {math.degrees(alpha)}, {math.degrees(beta)}')

    return tetta, alpha, beta

def calculate_C_point(O: Point, tetta: float, alpha: float, beta: float) -> Point:
    A = Point(O.x + leg.d * math.cos(tetta),
                O.y + leg.d * math.sin(tetta),
                O.z)

    B_xz = [leg.a * math.cos(alpha),
            leg.a * math.sin(alpha)]
    C_xz = [B_xz[0] + leg.b * math.cos(alpha - beta),
            B_xz[1] + leg.b * math.sin(alpha - beta)]

    C = Point(round(A.x + C_xz[0] * math.cos(tetta), 2),
                    round(A.y + C_xz[0] * math.sin(tetta), 2),
                    round(A.z + C_xz[1], 2))

    return C

def convert_alpha(alpha: float) -> float:
    alpha_converted = round(math.degrees(alpha), 2)
    return alpha_converted

def convert_alpha_to_kinematic(alpha_deg: float) -> float:
    return round(math.radians(alpha_deg), 4)

def convert_beta(beta: float) -> float:
    return -round(math.degrees(beta) + 90, 2)

def convert_beta_to_kinematic(beta_deg: float) -> float:
    return round(math.radians(-beta_deg - 90), 4)

def convert_tetta(tetta: float, leg_number: int) -> float:
    # virtual model to real servos
    tetta_degrees = math.degrees(tetta)
    
    if leg_number == 1:
        tetta_degrees -= 45
    elif leg_number == 2:
        tetta_degrees += 45
    elif leg_number == 3:
        tetta_degrees += 135
    elif leg_number == 4:
        tetta_degrees -= 135
    
    return round(tetta_degrees, 2)

def convert_tetta_to_kinematic(tetta_deg: float, leg_number: int) -> float:
    # real servos to virtual model
    if leg_number == 1:
        tetta_deg += 45        
    elif leg_number == 2:
        tetta_deg -= 45
    elif leg_number == 3:
        tetta_deg -= 135
    elif leg_number == 4:
        tetta_deg += 135
    
    tetta_radians = math.radians(tetta_deg)
    
    return round(tetta_radians, 4)
    
def convert_legs_angles_C(fp_in: RobotPosition, logger=None) -> RobotPosition:
    # input: 16 angles in RADIANS
    # output: 16 converted angles in DEGREES
    # now tetta, alpha, beta one leg after another
    #print(f'Before conversion: {fp_in}')
    fp = RobotPosition(
        leg1_alpha=convert_alpha(fp_in.legs[1].alpha),
        leg1_beta=convert_beta(fp_in.legs[1].beta),
        leg1_tetta=convert_tetta(fp_in.legs[1].tetta, 1),

        leg2_alpha=convert_alpha(fp_in.legs[2].alpha),
        leg2_beta=convert_beta(fp_in.legs[2].beta),
        leg2_tetta=convert_tetta(fp_in.legs[2].tetta, 2),

        leg3_alpha=convert_alpha(fp_in.legs[3].alpha),
        leg3_beta=convert_beta(fp_in.legs[3].beta),
        leg3_tetta=convert_tetta(fp_in.legs[3].tetta, 3),

        leg4_alpha=convert_alpha(fp_in.legs[4].alpha),
        leg4_beta=convert_beta(fp_in.legs[4].beta),
        leg4_tetta=convert_tetta(fp_in.legs[4].tetta, 4),

        leg5_alpha=convert_alpha(fp_in.legs[5].alpha),
        leg5_beta=convert_beta(fp_in.legs[5].beta),
        leg5_tetta=convert_tetta(fp_in.legs[5].tetta, 4),

        leg6_alpha=convert_alpha(fp_in.legs[6].alpha),
        leg6_beta=convert_beta(fp_in.legs[6].beta),
        leg6_tetta=convert_tetta(fp_in.legs[6].tetta, 4),
    )    
    #print(f'Converted: {fp}')

    """
    if not tettas_correct([
        fp.legs[1].tetta, 
        fp.legs[2].tetta, 
        fp.legs[3].tetta, 
        fp.legs[4].tetta
        ], 
        logger=logger):
        raise AnglesException('Bad tettas')
    """
    return fp

def convert_legs_angles_to_kinematic_C(fp_in: RobotPosition, logger=None) -> RobotPosition:
    # input: 16 angles in DEGREES
    # output: 16 converted angles in RADIANS
    # now tetta, alpha, beta one leg after another
    #print(f'convert_legs_angles_to_kinematic. Before {legs_angles}')
    fp = RobotPosition(
        leg1_alpha=convert_alpha_to_kinematic(fp_in.legs[1].alpha),
        leg1_beta=convert_beta_to_kinematic(fp_in.legs[1].beta),
        leg1_tetta=convert_tetta_to_kinematic(fp_in.legs[1].tetta, 1),

        leg2_alpha=convert_alpha_to_kinematic(fp_in.legs[2].alpha),
        leg2_beta=convert_beta_to_kinematic(fp_in.legs[2].beta),
        leg2_tetta=convert_tetta_to_kinematic(fp_in.legs[2].tetta, 2),

        leg3_alpha=convert_alpha_to_kinematic(fp_in.legs[3].alpha),
        leg3_beta=convert_beta_to_kinematic(fp_in.legs[3].beta),
        leg3_tetta=convert_tetta_to_kinematic(fp_in.legs[3].tetta, 3),

        leg4_alpha=convert_alpha_to_kinematic(fp_in.legs[4].alpha),
        leg4_beta=convert_beta_to_kinematic(fp_in.legs[4].beta),
        leg4_tetta=convert_tetta_to_kinematic(fp_in.legs[4].tetta, 4),

        leg5_alpha=convert_alpha_to_kinematic(fp_in.legs[5].alpha),
        leg5_beta=convert_beta_to_kinematic(fp_in.legs[5].beta),
        leg5_tetta=convert_tetta_to_kinematic(fp_in.legs[5].tetta, 4),

        leg6_alpha=convert_alpha_to_kinematic(fp_in.legs[6].alpha),
        leg6_beta=convert_beta_to_kinematic(fp_in.legs[6].beta),
        leg6_tetta=convert_tetta_to_kinematic(fp_in.legs[6].tetta, 4)
    )

    return fp

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
