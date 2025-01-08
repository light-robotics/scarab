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
        assert round(convert_alpha_to_kinematic(convert_alpha(angle)), 2) == angle
    
def test_convert_betta():
    for angle in [-60, -30, 0, 30, 60]:
        assert round(convert_beta_to_kinematic(convert_beta(angle)), 2) == angle

def test_convert_tetta():
    for angle in [-60, -30, 0, 30, 60]:
        for leg_number in [1, 2, 3, 4, 5, 6]:
            assert round(convert_tetta_to_kinematic(convert_tetta(angle, leg_number), leg_number), 2) == angle

def test_convert_position():
    position = RobotPosition(**{
        'l1t': 45.0, 'l1a': -17.46, 'l1b': -16.84, 
        'l2t': 0.0, 'l2a': -19.71, 'l2b': -4.29, 
        'l3t': -45.0, 'l3a': -17.46, 'l3b': -16.84, 
        'l4t': 45.0, 'l4a': -17.46, 'l4b': -16.84, 
        'l5t': 0.0, 'l5a': -19.71, 'l5b': -4.29, 
        'l6t': -45.0, 'l6a': -17.46, 'l6b': -16.84
    })
    assert convert_legs_angles_to_kinematic_C(convert_legs_angles_C(position)) == position
