from dataclasses import dataclass

class leg:
    a = 15
    b = 16
    d = 0

class robot:
    horizontal_x = 15
    horizontal_y = 15
    vertical = 10

    touch_down = -6
    touch_down_iterations = 5
    touch_up = 27
    feedback_body_up = 27
    reset_touch_up = 22
    reset_feedback_body_up = 20
    
    balance_iterations = 5

    fb_pre_balance_value = 3
    fb_one_leg_balance_value = -6
    fb_balance_value = -3 # body
    fb_inward_delta = 2

    x_offset = 0
    y_offset = 0
    middle_leg_offset = 5

    leg_up = 9

    diff_from_target_limit = 3.5
    diff_from_prev_limit = 0.5

    balance_offset = 2.5

class movement:
    command_advance_ms = 0 #0.05

class speed:
    run = 500
    walk = 500
    hit = 250
    ripple_gait = 500
    wave_gait = 350
    touch = 600
    balance = 1000
    feedback_body = 700
    feedback_legs = 600

@dataclass
class limit:
    min: int
    max: int

class angles_limits:
    alpha = limit(-70, 90)
    beta = limit(-145, 0)
    tetta = limit(-90, 90)

servos_mapping = {
    "l1t" : 11,
    "l1a" : 10,
    "l1b" : 9,

    "l2t" : 2,  # temporarily offline
    "l2a" : 20,
    "l2b" : 12,

    "l3t" : 17,
    "l3a" : 16,
    "l3b" : 15,

    "l4t" : 23,
    "l4a" : 22,
    "l4b" : 21,

    "l5t" : 13,
    "l5a" : 8,
    "l5b" : 14,

    "l6t" : 5,
    "l6a" : 4,
    "l6b" : 3,
}

servos_boards = {
    3: 3, 4: 3, 5: 3, 8: 3, 13: 3, 14: 3, 21: 3, 22: 3, 23: 3,
    2: 4, 9: 4, 10: 4, 11: 4, 12: 4, 15: 4, 16: 4, 17: 4, 20: 4,
}

class moves:
    up_or_down_cm = 2
    move_body_cm = 2
    forward_body_1_leg_cm = 6
    forward_body_2_leg_cm = 7
    reposition_cm = 1

@dataclass
class xy:
    x: int
    y: int

class modes:
    run_mode = xy(9, 9)
    walking_mode = xy(10, 10)
    sentry_mode = xy(10, 10)
    battle_mode = xy(10, 10)

class files:
    movement = '/scarab/scarab/wrk/movement_command.txt'
    neopixel = '/scarab/scarab/wrk/neopixel_command.txt'
    gyroaccel = '/scarab/scarab/wrk/gyroaccel.txt'

class gyroaccel:
    readings = 10
    pitch_bias = -0.5
    roll_bias = 6
