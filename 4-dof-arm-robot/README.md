# 4-DOF Arm Robot Library

A reusable Python library and GUI controller for a 4-DOF robotic arm.

The project separates the reusable robot core from the user configuration:

```text
4-dof-arm-robot/
│
├── arm_robot/
│   ├── __init__.py
│   └── robot.py
│
└── my_robot_project/
    └── main.py
```

## What is inside?

### `arm_robot/robot.py`

This is the reusable library. It contains:

- 4-DOF inverse kinematics
- Elbow-UP and Elbow-DOWN solutions
- Serial communication
- Smooth joint interpolation
- Cartesian movement
- Home position movement
- Tkinter GUI

You normally do **not** need to modify this file.

### `my_robot_project/main.py`

This is the user-facing file.

Change only the configuration section:

```python
a1 = 26
a2 = 127
a3 = 104
a4 = 100

d1 = 54

com_port = "COM7"
baudrate = 115200

home_position = [90, 90, 90, 0]
```

Then run the program.

---

# Installation

## Requirements

- Python 3.9+
- NumPy
- PySerial
- Tkinter

On Ubuntu/Debian, Tkinter may need to be installed separately:

```bash
sudo apt update
sudo apt install python3-tk
```

Install Python dependencies:

```bash
python -m pip install numpy pyserial
```

On systems where `python` points to Python 2, use:

```bash
python3 -m pip install numpy pyserial
```

---

# Download from GitHub

After downloading or cloning the repository:

```bash
git clone https://github.com/YOUR_USERNAME/robot4dof-kit.git
cd robot4dof-kit
```

Install dependencies:

```bash
python -m pip install -r requirements.txt
```

Run:

```bash
python my_robot_project/main.py
```

> If you download the ZIP from GitHub, extract it first and then run the same command from the project folder.

---

# Using it as a Python library

The reusable package can also be imported directly:

```python
from arm_robot import Robot4DOF

robot = Robot4DOF(
    a1=26,
    a2=127,
    a3=104,
    a4=100,
    d1=54,
    com_port="COM7",
    baudrate=115200,
    home_position=[90, 90, 90, 0],
)
```

Calculate inverse kinematics:

```python
solutions = robot.calculate_ik(
    x=150,
    y=0,
    z=150,
    phi=0,
)

print("Elbow UP:", solutions[0])
print("Elbow DOWN:", solutions[1])
```

Connect and move:

```python
robot.connect()

try:
    robot.go_home()

    robot.move_to_position(
        x=150,
        y=0,
        z=150,
        phi=0,
        solution="down",
    )
finally:
    robot.disconnect()
```

---

# GUI

The GUI is already included inside `arm_robot.robot`.

You can use it from your own program:

```python
from arm_robot import Robot4DOF, RobotGUI

robot = Robot4DOF(
    a1=26,
    a2=127,
    a3=104,
    a4=100,
    d1=54,
    com_port="COM7",
    baudrate=115200,
    home_position=[90, 90, 90, 0],
)

gui = RobotGUI(robot)
gui.run()
```

The GUI does not contain a second copy of the IK or Serial code. It calls the library methods.

---

# Important

## Robot dimensions

The current IK model uses:

- `a1`
- `a2`
- `a3`
- `a4`
- `d1`

Do not replace `a3` or `a4` with `L3` or `L4`.

## Arduino serial format

The Python library sends:

```text
q1,q2,q3,q4
```

followed by a newline.

Example:

```text
90.00,80.50,100.00,5.00
```

Your Arduino code should read the four comma-separated joint angles using the same format.

## Reachability

If the requested Cartesian position is outside the mathematical workspace, the IK function raises:

```text
ValueError: Target position is outside the robot workspace.
```

---

# Project design

The package is intentionally kept small:

```text
arm_robot/
    __init__.py
    robot.py
```

The library code is separated from the user project:

```text
my_robot_project/
    main.py
```

This means users can change their robot dimensions and serial settings without modifying the reusable library.

---

# License

MIT License.

You may use, modify, and distribute this project.

