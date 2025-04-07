import tkinter as tk
from tkinter import ttk
import rclpy
from threading import Thread
from arm_control_node import ArmControlNode
from motor_control_node import WheelRPMCalculator


class RoverControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Rover Control GUI")

        # ROS 2 Nodes
        rclpy.init()
        self.arm_node = ArmControlNode()
        self.motor_node = WheelRPMCalculator()

        # Arm Control Section
        arm_frame = ttk.LabelFrame(root, text="Arm Control", padding=10)
        arm_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Label(arm_frame, text="Control Type:").grid(row=0, column=0, sticky="w")
        self.arm_control_type = ttk.Combobox(arm_frame, values=["move_steps", "set_max_speed", "set_raw_position"])
        self.arm_control_type.grid(row=0, column=1, sticky="ew")

        ttk.Label(arm_frame, text="Motor ID:").grid(row=1, column=0, sticky="w")
        self.arm_motor_id = ttk.Spinbox(arm_frame, from_=0, to=3, width=5)
        self.arm_motor_id.grid(row=1, column=1, sticky="ew")

        ttk.Label(arm_frame, text="Parameter:").grid(row=2, column=0, sticky="w")
        self.arm_parameter = ttk.Entry(arm_frame)
        self.arm_parameter.grid(row=2, column=1, sticky="ew")

        self.arm_send_button = ttk.Button(arm_frame, text="Send Command", command=self.send_arm_command)
        self.arm_send_button.grid(row=3, column=0, columnspan=2, pady=5)

        # Motor Control Section
        motor_frame = ttk.LabelFrame(root, text="Motor Control", padding=10)
        motor_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        ttk.Label(motor_frame, text="Wheel RPMs (FL, FR, RL, RR):").grid(row=0, column=0, sticky="w")
        self.motor_rpms = ttk.Entry(motor_frame)
        self.motor_rpms.grid(row=0, column=1, sticky="ew")

        self.motor_send_button = ttk.Button(motor_frame, text="Send RPMs", command=self.send_motor_command)
        self.motor_send_button.grid(row=1, column=0, columnspan=2, pady=5)

        # Quit Button
        self.quit_button = ttk.Button(root, text="Quit", command=self.quit_gui)
        self.quit_button.grid(row=2, column=0, pady=10)

    def send_arm_command(self):
        """
        Sends a command to the Arm Control Node based on GUI input.
        """
        control_type = self.arm_control_type.get()
        motor_id = int(self.arm_motor_id.get())
        parameter = float(self.arm_parameter.get())

        try:
            self.arm_node.send_arm_command(control_type, motor_id, parameter)
        except Exception as e:
            print(f"Failed to send arm command: {e}")

    def send_motor_command(self):
        """
        Sends RPMs to the Motor Control Node based on GUI input.
        """
        try:
            rpms = [float(rpm) for rpm in self.motor_rpms.get().split(",")]
            if len(rpms) != 4:
                raise ValueError("Please provide exactly 4 RPM values (FL, FR, RL, RR).")
            self.motor_node.send_rpms_to_arduino(*rpms)
        except Exception as e:
            print(f"Failed to send motor command: {e}")

    def quit_gui(self):
        """
        Cleans up and exits the GUI.
        """
        self.arm_node.destroy_node()
        self.motor_node.destroy_node()
        rclpy.shutdown()
        self.root.quit()


def main():
    root = tk.Tk()
    gui = RoverControlGUI(root)

    # Run ROS 2 in a separate thread
    def ros_spin():
        while rclpy.ok():
            rclpy.spin_once(gui.arm_node, timeout_sec=0.1)
            rclpy.spin_once(gui.motor_node, timeout_sec=0.1)

    ros_thread = Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    root.mainloop()


if __name__ == "__main__":
    main()