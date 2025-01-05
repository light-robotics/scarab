import RPi.GPIO as GPIO
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from robot_hardware.neopixel_commands_setter import NeopixelCommandsSetter
import time


def button_changed_front_left(button):
    global front_left_down
    if GPIO.input(button) == GPIO.LOW:
        print("Left leg down.")
        front_left_down = True
    else:
        print("Left leg up.")
        front_left_down = False

def button_changed_front_right(button):
    global front_right_down
    if GPIO.input(button) == GPIO.LOW:
        print("Right leg down.")
        front_right_down = True
    else:
        print("Right leg up.")
        front_right_down = False

def button_changed_back_right(button):
    global back_right_down
    if GPIO.input(button) == GPIO.LOW:
        print("Btn3 leg down.")
        back_right_down = True
    else:
        print("Btn3 leg up.")
        back_right_down = False

def button_changed_back_left(button):
    global back_left_down
    if GPIO.input(button) == GPIO.LOW:
        print("Btn4 leg down.")
        back_left_down = True
    else:
        print("Btn4 leg up.")
        back_left_down = False

def button_changed_middle_left(button):
    global middle_left_down
    if GPIO.input(button) == GPIO.LOW:
        print("Btn5 leg down.")
        middle_left_down = True
    else:
        print("Btn5 leg up.")
        middle_left_down = False

def button_changed_middle_right(button):
    global middle_right_down
    if GPIO.input(button) == GPIO.LOW:
        print("Btn6 leg down.")
        middle_right_down = True
    else:
        print("Btn6 leg up.")
        middle_right_down = False



class FenixEnders():
    FRONT_LEFT = 24
    FRONT_RIGHT = 23
    MIDDLE_LEFT = 20
    MIDDLE_RIGHT = 26
    BACK_RIGHT = 27
    BACK_LEFT = 22
    
    btn_function_dispatcher = {
        FRONT_LEFT: button_changed_front_left,
        FRONT_RIGHT: button_changed_front_right,
        MIDDLE_LEFT: button_changed_middle_left,
        MIDDLE_RIGHT: button_changed_middle_right,
        BACK_LEFT: button_changed_back_left,
        BACK_RIGHT: button_changed_back_right,
    }
    
    def __init__(self):
        self.neopixel = NeopixelCommandsSetter()

        GPIO.setmode(GPIO.BCM)
        for btn in (
            self.FRONT_LEFT,
            self.FRONT_RIGHT,
            self.MIDDLE_LEFT,
            self.MIDDLE_RIGHT,
            self.BACK_LEFT,
            self.BACK_RIGHT
            ):
            print(self.btn_function_dispatcher[btn])
            GPIO.setup(btn, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.add_event_detect(btn, GPIO.BOTH, callback=self.btn_function_dispatcher[btn], bouncetime=10)

    def run(self):
        global front_left_down, front_right_down, middle_right_down, back_right_down, back_left_down, middle_left_down
        prev_value = None
        print("Press CTRL-C to exit.")
        try:
            while True:
                result_legs = ''
                if front_right_down:
                    result_legs += '1'
                else:
                    result_legs += '0'
                
                if middle_right_down:
                    result_legs += '1'
                else:
                    result_legs += '0'
                
                if back_right_down:
                    result_legs += '1'
                else:
                    result_legs += '0'

                if back_left_down:
                    result_legs += '1'
                else:
                    result_legs += '0'
                
                if middle_left_down:
                    result_legs += '1'
                else:
                    result_legs += '0'
                
                if front_left_down:
                    result_legs += '1'
                else:
                    result_legs += '0'

                if result_legs != prev_value:
                    self.neopixel.issue_command(result_legs)
                prev_value = result_legs
                time.sleep(0.01)

        finally:
            GPIO.cleanup()

if __name__ == '__main__':
    global front_left_down, front_right_down, back_right_down, back_left_down, middle_left_down, middle_right_down
    front_left_down, front_right_down, back_right_down, back_left_down, middle_left_down, middle_right_down = False, False, False, False, False, False
    fe = FenixEnders().run()
