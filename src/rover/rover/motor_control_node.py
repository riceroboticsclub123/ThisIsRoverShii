import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
import serial  # Import serial communication module

class WheelRPMCalculator(Node):
    def __init__(self):
        super().__init__('wheel_rpm_calculator')
        
        # Parameters for the rover
        self.declare_parameter('wheel_radius', 0.05)  # Radius of the wheel (m)
        self.declare_parameter('wheel_base', 0.3)     # Distance between left and right wheels (m)
        self.declare_parameter('axle_length', 0.4)    # Distance between front and rear wheels (m)
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.axle_length = self.get_parameter('axle_length').get_parameter_value().double_value

        # Set up serial communication (adjust to your port and baud rate)
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600)  # Example for Linux, adjust as needed
        
        # Subscriber to receive velocity commands (Twist message)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher to send RPMs for each wheel
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'wheel_rpms',
            10
        )

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities from the Twist message
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        
        # Calculate wheel velocities
        v_fl = vx - vy - omega * (self.wheel_base + self.axle_length) / 2  # Front Left
        v_fr = vx + vy + omega * (self.wheel_base + self.axle_length) / 2  # Front Right
        v_rl = vx + vy - omega * (self.wheel_base + self.axle_length) / 2  # Rear Left
        v_rr = vx - vy + omega * (self.wheel_base + self.axle_length) / 2  # Rear Right
        
        # Convert linear velocities to angular velocities (rad/s)
        omega_fl = v_fl / self.wheel_radius
        omega_fr = v_fr / self.wheel_radius
        omega_rl = v_rl / self.wheel_radius
        omega_rr = v_rr / self.wheel_radius
        
        # Convert angular velocities to RPM
        rpm_fl = omega_fl * 60 / (2 * np.pi)
        rpm_fr = omega_fr * 60 / (2 * np.pi)
        rpm_rl = omega_rl * 60 / (2 * np.pi)
        rpm_rr = omega_rr * 60 / (2 * np.pi)
        
        # Publish the RPMs as a Float64MultiArray
        rpm_msg = Float64MultiArray()
        rpm_msg.data = [rpm_fl, rpm_fr, rpm_rl, rpm_rr]
        self.publisher.publish(rpm_msg)
        
        # Log the RPMs
        self.get_logger().info(f'Wheel RPMs: FL={rpm_fl:.2f}, FR={rpm_fr:.2f}, RL={rpm_rl:.2f}, RR={rpm_rr:.2f}')

        # Send the RPMs to Arduino (via serial communication)
        self.send_rpms_to_arduino(rpm_fl, rpm_fr, rpm_rl, rpm_rr)

    def send_rpms_to_arduino(self, rpm_fl, rpm_fr, rpm_rl, rpm_rr):
        # Format the RPM data into a string and send it to the Arduino
        message = f"{rpm_fl:.2f},{rpm_fr:.2f},{rpm_rl:.2f},{rpm_rr:.2f}\n"
        for index, element in enumerate(message.split(',')):
            command = "M" + str(index) + ":set_velocity_rpm:" + str(element) + "&"
            self.serial_port.write(command.encode())  # Send data to Arduino over serial
        self.get_logger().info(f"Sent RPM data to Arduino: {message.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = WheelRPMCalculator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
