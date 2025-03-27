import rclpy
from rclpy.node import Node
import serial  # For serial communication

class ArmControlNode(Node):
    def __init__(self):
        super().__init__('arm_control_node')

        # Set up serial communication (adjust port and baud rate)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)

        self.get_logger().info("Arm Control Node initialized and ready to send commands.")

    def send_arm_command(self, control_type, motor_id, parameter):
        """
        Constructs and sends a command to the Arduino based on the control type.

        :param control_type: The type of control (e.g., 'move_steps', 'set_max_speed', 'set_raw_position').
        :param motor_id: The ID of the motor to control (integer).
        :param parameter: The parameter for the command (integer or float).
        """
        try:
            if control_type == 'move_steps':
                command = f"C {motor_id} {int(parameter)}"
            elif control_type == 'set_max_speed':
                command = f"R {motor_id} {float(parameter):.2f}"
            elif control_type == 'set_raw_position':
                command = f"P {motor_id} {int(parameter)}"
            else:
                self.get_logger().error(f"Unknown control type: {control_type}")
                return

            # Send the constructed command to the Arduino
            self.serial_port.write(f"*{command}&".encode())
            self.get_logger().info(f"Sent command: *{command}&")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArmControlNode()

    try:
        # Example usage of send_arm_command
        node.send_arm_command('move_steps', 1, 500)  # Move motor 1 by 500 steps
        node.send_arm_command('set_max_speed', 2, 0.8)  # Set motor 2 speed to 80%
        node.send_arm_command('set_raw_position', 0, 1000)  # Move motor 0 to position 1000
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()