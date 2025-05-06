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

Video streaming:
- sudo ffmpeg -s 1024x576 -f video4linux2 -i /dev/video0 -f mpegts -codec:v mpeg1video -b:v 4000k -r 30 http://{nexus_ip}:8081/12345/1024/576/

Useful:
 - vcgencmd measure_temp
 - dmesg