import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..')))

from angles import (
    convert_alpha,
    convert_alpha_to_kinematic,
    convert_beta,
    convert_beta_to_kinematic,
    convert_tetta,
    convert_tetta_to_kinematic,
    RobotPosition,
    convert_legs_angles_C,
    convert_legs_angles_to_kinematic_C,
)

def test_convert_alpha():
    for angle in [-60, -30, 0, 30, 60]:
        assert round(convert_alpha(convert_alpha_to_kinematic(angle)), 2) == angle
    
def test_convert_betta():
    for angle in [-60, -30, 0, 30, 60]:
        assert round(convert_beta(convert_beta_to_kinematic(angle)), 2) == angle

def test_convert_tetta():
    for angle in [-90, -60, -30, 0, 30, 60, 90]:
        for leg_number in [1, 2, 3, 4, 5, 6]:
            assert round(convert_tetta(convert_tetta_to_kinematic(angle, leg_number), leg_number), 2) == angle

def test_convert_position():
    position = RobotPosition(**{
        'l1t': 0.0, 'l1a': -17.46, 'l1b': -16.84, 
        'l2t': 0.0, 'l2a': -19.71, 'l2b': -4.29, 
        'l3t': -45.0, 'l3a': -17.46, 'l3b': -16.84, 
        'l4t': 45.0, 'l4a': -17.46, 'l4b': -16.84, 
        'l5t': 0.0, 'l5a': -19.71, 'l5b': -4.29, 
        'l6t': -45.0, 'l6a': -17.46, 'l6b': -16.84
    })
    position_altered = convert_legs_angles_C(convert_legs_angles_to_kinematic_C(position))
    for k, v in position.__dict__.items():
        print(k)
        assert v == position_altered.__dict__[k]

"""
if __name__ == '__main__':
    position = RobotPosition(**{
        'l1t': 0.0, 'l1a': -17.46, 'l1b': -16.84, 
        'l2t': 0.0, 'l2a': -19.71, 'l2b': -4.29, 
        'l3t': -45.0, 'l3a': -17.46, 'l3b': -16.84, 
        'l4t': 45.0, 'l4a': -17.46, 'l4b': -16.84, 
        'l5t': 0.0, 'l5a': -19.71, 'l5b': -4.29, 
        'l6t': -45.0, 'l6a': -17.46, 'l6b': -16.84
    })
    print(convert_legs_angles_to_kinematic_C(position))
    #print(convert_tetta_to_kinematic(position.l1t, 1))
"""