"""
User configuration and application entry point.

IMPORTANT:
Edit ONLY the CONFIGURATION section for your own robot.
The arm_robot package contains the reusable robot library and GUI.
"""

from arm_robot import Robot4DOF, RobotGUI


# ==========================================================
# CONFIGURATION
# ==========================================================
# Edit these values for your own 4-DOF robot.

a1 = 26
a2 = 127
a3 = 104
a4 = 100

d1 = 54

com_port = "COM7"
baudrate = 115200

home_position = [90, 90, 90, 0]


# ==========================================================
# CREATE ROBOT
# ==========================================================

robot = Robot4DOF(
    a1=a1,
    a2=a2,
    a3=a3,
    a4=a4,
    d1=d1,
    com_port=com_port,
    baudrate=baudrate,
    home_position=home_position,
)


# ==========================================================
# RUN GUI
# ==========================================================

if __name__ == "__main__":
    gui = RobotGUI(robot)
    gui.run()
