from dataclasses import dataclass

class leg:
    a = 15
    b = 15.5
    d = 5

class robot:
    horizontal_x = 16
    horizontal_y = 16
    vertical = 10

    leg_up = 5
   
@dataclass
class limit:
    min: int
    max: int

class angles_limits:
    alpha = limit(-70, 90)
    beta = limit(-145, 0)
    gamma = limit(-110, 110)

