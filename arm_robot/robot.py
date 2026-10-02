import time
import tkinter as tk
from tkinter import ttk

import numpy as np
import serial


class Robot4DOF:
    """
    Core 4-DOF robot controller.

    Handles:
    - Inverse kinematics
    - Serial communication
    - Joint interpolation
    - Cartesian movement
    - Home position
    """

    def __init__(
        self,
        a1,
        a2,
        a3,
        a4,
        d1,
        com_port,
        baudrate,
        home_position,
    ):
        self.a1 = float(a1)
        self.a2 = float(a2)
        self.a3 = float(a3)
        self.a4 = float(a4)
        self.d1 = float(d1)

        self.com_port = com_port
        self.baudrate = int(baudrate)

        self.home_position = np.asarray(home_position, dtype=float)
        if self.home_position.shape != (4,):
            raise ValueError("home_position must contain exactly 4 angles.")

        self.current_position = self.home_position.copy()
        self.serial = None

    # ==========================================================
    # INVERSE KINEMATICS
    # ==========================================================

    def calculate_ik(self, x, y, z, phi_deg):
        """
        Calculate the two IK solutions.

        Returns:
            numpy array with shape (2, 4):
            [q1, q2, q3, q4] in degrees.

            Row 0 = elbow UP
            Row 1 = elbow DOWN
        """
        x = float(x)
        y = float(y)
        z = float(z)
        phi = np.deg2rad(float(phi_deg))

        q1 = np.arctan2(y, x)

        # Wrist center
        xw = x - self.a4 * np.cos(q1) * np.cos(phi)
        yw = y - self.a4 * np.sin(q1) * np.cos(phi)
        zw = z - self.a4 * np.sin(phi)

        # Recalculate q1 from wrist center
        q1 = np.arctan2(yw, xw)

        # Planar distances
        r = np.sqrt(xw**2 + yw**2) - self.a1
        s = zw - self.d1

        # Cosine law
        denominator = 2 * self.a2 * self.a3
        if np.isclose(denominator, 0):
            raise ValueError("a2 and a3 must be non-zero.")

        D = (
            r**2 + s**2 - self.a2**2 - self.a3**2
        ) / denominator

        # A target is unreachable if D is outside [-1, 1].
        if D < -1.0 or D > 1.0:
            raise ValueError(
                "Target position is outside the robot workspace."
            )

        D = np.clip(D, -1.0, 1.0)

        # Elbow UP
        q3_up = np.arctan2(np.sqrt(1 - D**2), D)
        q2_up = (
            np.arctan2(s, r)
            - np.arctan2(
                self.a3 * np.sin(q3_up),
                self.a2 + self.a3 * np.cos(q3_up),
            )
        )

        # Elbow DOWN
        q3_down = np.arctan2(-np.sqrt(1 - D**2), D)
        q2_down = (
            np.arctan2(s, r)
            - np.arctan2(
                self.a3 * np.sin(q3_down),
                self.a2 + self.a3 * np.cos(q3_down),
            )
        )

        # Joint 4
        q4_up = phi - (q2_up + q3_up)
        q4_down = phi - (q2_down + q3_down)

        return np.array(
            [
                np.rad2deg([q1, q2_up, q3_up, q4_up]),
                np.rad2deg([q1, q2_down, q3_down, q4_down]),
            ],
            dtype=float,
        )

    def get_elbow_up_solution(self, x, y, z, phi_deg):
        return self.calculate_ik(x, y, z, phi_deg)[0]

    def get_elbow_down_solution(self, x, y, z, phi_deg):
        return self.calculate_ik(x, y, z, phi_deg)[1]

    # ==========================================================
    # SERIAL
    # ==========================================================

    def connect(self):
        """Open the configured serial port."""
        if self.serial is not None and self.serial.is_open:
            return

        self.serial = serial.Serial(
            self.com_port,
            self.baudrate,
            timeout=1,
        )
        time.sleep(2)

    def disconnect(self):
        """Close the serial connection."""
        if self.serial is not None and self.serial.is_open:
            self.serial.close()

    @property
    def is_connected(self):
        return self.serial is not None and self.serial.is_open

    # ==========================================================
    # SEND ANGLES
    # ==========================================================

    def send_angles(self, angles):
        """
        Send four joint angles to Arduino.

        Format:
            q1,q2,q3,q4\\n
        """
        if not self.is_connected:
            raise RuntimeError("Robot is not connected.")

        angles = np.asarray(angles, dtype=float)

        if angles.shape != (4,):
            raise ValueError("Exactly 4 joint angles are required.")

        data = ",".join(f"{angle:.2f}" for angle in angles) + "\n"
        self.serial.write(data.encode())

    # ==========================================================
    # MOVEMENT
    # ==========================================================

    def move_to_angles(self, target, steps=50, delay=0.05):
        """
        Smoothly move from current joint position to target.
        """
        target = np.asarray(target, dtype=float)

        if target.shape != (4,):
            raise ValueError("Exactly 4 target angles are required.")

        if steps < 1:
            raise ValueError("steps must be at least 1.")

        start = self.current_position.copy()

        for i in range(steps + 1):
            position = start + (target - start) * i / steps
            self.send_angles(position)
            time.sleep(delay)

        self.current_position = target.copy()

    def move_to_position(
        self,
        x,
        y,
        z,
        phi,
        solution="down",
        steps=50,
        delay=0.05,
    ):
        """
        Calculate IK and move to a Cartesian target.

        solution:
            "up" or "down"
        """
        solutions = self.calculate_ik(x, y, z, phi)

        solution = solution.lower()

        if solution == "up":
            target = solutions[0]
        elif solution == "down":
            target = solutions[1]
        else:
            raise ValueError("solution must be 'up' or 'down'.")

        self.move_to_angles(target, steps=steps, delay=delay)
        return target

    def go_home(self, steps=50, delay=0.05):
        """Move the robot to its configured home position."""
        self.move_to_angles(
            self.home_position,
            steps=steps,
            delay=delay,
        )


class RobotGUI:
    """
    Simple Tkinter GUI for Robot4DOF.

    The GUI uses Robot4DOF methods and does not duplicate IK or
    serial logic.
    """

    def __init__(self, robot, title="4-DOF Robot IK Controller"):
        self.robot = robot
        self.solutions = None

        self.root = tk.Tk()
        self.root.title(title)
        self.root.geometry("430x650")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        self._build_gui()

    def _build_gui(self):
        frame = ttk.Frame(self.root, padding=20)
        frame.pack(fill="both", expand=True)

        ttk.Label(frame, text="X").grid(row=0, column=0, padx=5, pady=5)
        self.entry_x = ttk.Entry(frame)
        self.entry_x.grid(row=0, column=1)
        self.entry_x.insert(0, "150")

        ttk.Label(frame, text="Y").grid(row=1, column=0, padx=5, pady=5)
        self.entry_y = ttk.Entry(frame)
        self.entry_y.grid(row=1, column=1)
        self.entry_y.insert(0, "0")

        ttk.Label(frame, text="Z").grid(row=2, column=0, padx=5, pady=5)
        self.entry_z = ttk.Entry(frame)
        self.entry_z.grid(row=2, column=1)
        self.entry_z.insert(0, "150")

        ttk.Label(frame, text="Phi").grid(row=3, column=0, padx=5, pady=5)
        self.entry_phi = ttk.Entry(frame)
        self.entry_phi.grid(row=3, column=1)
        self.entry_phi.insert(0, "0")

        ttk.Button(
            frame,
            text="Connect Robot",
            command=self.connect_robot,
        ).grid(row=4, columnspan=2, pady=10)

        ttk.Button(
            frame,
            text="Compute IK",
            command=self.compute_ik,
        ).grid(row=5, columnspan=2, pady=5)

        self.result_up = ttk.Label(frame, text="UP:")
        self.result_up.grid(row=6, columnspan=2, pady=10)

        self.result_down = ttk.Label(frame, text="DOWN:")
        self.result_down.grid(row=7, columnspan=2, pady=10)

        self.solution_var = tk.StringVar(value="DOWN")

        ttk.Radiobutton(
            frame,
            text="UP",
            variable=self.solution_var,
            value="UP",
            command=self.update_selected,
        ).grid(row=8, column=0)

        ttk.Radiobutton(
            frame,
            text="DOWN",
            variable=self.solution_var,
            value="DOWN",
            command=self.update_selected,
        ).grid(row=8, column=1)

        self.selected_label = ttk.Label(frame, text="Selected:")
        self.selected_label.grid(row=9, columnspan=2, pady=10)

        ttk.Button(
            frame,
            text="Send to Robot",
            command=self.send_to_robot,
        ).grid(row=10, columnspan=2, pady=5)

        ttk.Button(
            frame,
            text="Go Home",
            command=self.go_home,
        ).grid(row=11, columnspan=2, pady=5)

        self.status_label = ttk.Label(
            frame,
            text="Robot disconnected"
        )
        self.status_label.grid(row=12, columnspan=2, pady=15)

    @staticmethod
    def _format_solution(name, angles):
        return (
            f"{name}:\n"
            f"q1 = {angles[0]:.2f}°\n"
            f"q2 = {angles[1]:.2f}°\n"
            f"q3 = {angles[2]:.2f}°\n"
            f"q4 = {angles[3]:.2f}°"
        )

    def connect_robot(self):
        try:
            self.robot.connect()
            self.status_label.config(text="Robot connected")
            self.robot.go_home()
        except Exception as exc:
            self.status_label.config(
                text=f"Connection error: {exc}"
            )

    def compute_ik(self):
        try:
            x = float(self.entry_x.get())
            y = float(self.entry_y.get())
            z = float(self.entry_z.get())
            phi = float(self.entry_phi.get())

            self.solutions = self.robot.calculate_ik(
                x, y, z, phi
            )

            self.result_up.config(
                text=self._format_solution(
                    "UP",
                    self.solutions[0],
                )
            )

            self.result_down.config(
                text=self._format_solution(
                    "DOWN",
                    self.solutions[1],
                )
            )

            self.update_selected()

        except Exception as exc:
            self.result_up.config(text=f"Error: {exc}")
            self.result_down.config(text="")
            self.selected_label.config(text="Selected:")

    def update_selected(self):
        if self.solutions is None:
            return

        index = 0 if self.solution_var.get() == "UP" else 1
        angles = self.solutions[index]

        self.selected_label.config(
            text=self._format_solution("Selected", angles)
        )

    def send_to_robot(self):
        if self.solutions is None:
            self.status_label.config(
                text="Calculate IK first."
            )
            return

        try:
            index = 0 if self.solution_var.get() == "UP" else 1
            target = self.solutions[index]

            self.robot.move_to_angles(target)

            self.status_label.config(
                text="Robot moved successfully"
            )

        except Exception as exc:
            self.status_label.config(
                text=f"Error: {exc}"
            )

    def go_home(self):
        try:
            self.robot.go_home()
            self.status_label.config(
                text="Robot returned to home position"
            )
        except Exception as exc:
            self.status_label.config(
                text=f"Error: {exc}"
            )

    def run(self):
        self.root.mainloop()

    def close(self):
        try:
            self.robot.disconnect()
        finally:
            self.root.destroy()
