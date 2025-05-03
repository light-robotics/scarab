# scarab
Scarab hexapod robot.
YouTube channel: https://www.youtube.com/channel/UC5iMcYcLpUnzzhuc-a_uPiQ
Email: light.robotics.2020@gmail.com

To run scarab:
- sudo /scarab/venv/bin/python /scarab/scarab/run/neopixel_commands_reader.py
- sudo /scarab/venv/bin/python /scarab/scarab/robot_hardware/robot_dualsense.py
- python /scarab/scarab/core/movement_processor.py
Feedback versions:
- python /scarab/scarab/core/movement_processor_feedback.py
- python /scarab/scarab/robot_hardware/enders2.py
- python /scarab/scarab/robot_hardware/read_mpu6050.py

Useful:
 - vcgencmd measure_temp
 - dmesg