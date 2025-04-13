import serial, time

class SerialControlDemo:
    def __init__(self, port='/dev/ttyACM0', baud_rate=9600):
        """
        Initializes the serial connection.
        :param port: The serial port to connect to (e.g., '/dev/ttyACM0' or 'COM3').
        :param baud_rate: The baud rate for the serial connection.
        """
        self.serial_port = serial.Serial(port, baud_rate)
        print(f"Connected to serial port {port} at {baud_rate} baud.")

    def send_command(self, command_id, motor_id, parameter, unit=None):
        """
        Constructs and sends a command in the specified format.
        :param command_id: The command ID (e.g., 'C', 'R', 'P', 'V').
        :param motor_id: The motor ID (integer).
        :param parameter: The parameter for the command (integer or float).
        :param unit: Optional unit for the command (integer).
        """
        # Construct the command string
        if unit is not None:
            command = f"*{command_id}{motor_id} {parameter} {unit}&"
        else:
            command = f"*{command_id}{motor_id} {parameter}&"

        # Send the command through serial
        self.serial_port.write(command.encode())
        print(f"Sent command: {command}")

    def close(self):
        """
        Closes the serial connection.
        """
        self.serial_port.close()
        print("Serial connection closed.")
    
    def move_wheels(self, start_speed, end_speed, step_size, spin=False, unit=1):
        """
        Initiate both side wheels' motor speed to start_speed, increment by
         step_size during each iteration, stop until end_speed is reached
        :param start_speed: The start speed (float).
        :param end_speed: The end speed (float).
        :param step_size: How much to increment speed during each iteration.
        :param unit: The unit to which the speed will be set (integer). 1 stands
        for RPM (rotations per minute), 2 stands for percentage (-100 to 100).
        """
        for speed in range(start_speed, end_speed, step_size):
            if spin:
                print(f"Set left side motor speed to {speed} RPM, right side to {-speed} RPM.")
                self.send_command('S', speed, -speed, unit)
                print("Movement ends, pause 1 second.")
            else:
                print(f"Set both sides motor speed to {speed} RPM.")
                self.send_command('S', speed, speed, unit)
                print("Movement ends, pause 1 second.")
            time.sleep(1) # pause 1 second
    
    def move_arms(self, start_motor, end_motor, max_speed, move_steps):
        """
        Set rover arms motor maximum speed to max_speed, then move each joint's
        arm by move_steps.
        :param start_motor: The ID of the joint that moves first (integer).
        :param end_motor: The ID of the joint that moves last (integer).
        :param max_speed: The maximum speed we set to the rover arms motor (float).
        :param move_steps: The number of steps you want to move (integer).
        """
        for motor in range(start_motor, end_motor+1):
            print(f"Set rover arm joint #{motor} max speed to {max_speed*100}%.")
            self.send_command('R', motor, max_speed)
            print(f"Move rover arm joint #{motor} by {move_steps} steps.")
            self.send_command('C', motor, move_steps)
            print("Movement ends, pause 1 second.")
            time.sleep(1) # pause 1 second

def main():
    # Initialize the serial control demo
    demo = SerialControlDemo(port='/dev/ttyACM0', baud_rate=9600)

    try:
        print("Demo code for moving the rover wheels, starting now:")
        # Move forward
        print("Move forward!")
        demo.move_wheels(1, 6, 1)
        # Move backward
        print("Move backward!")
        demo.move_wheels(-1, -6, -1)
        # Spinning
        print("Try spinning!")
        demo.move_wheels(1, 6, 1, True)
        
        print("Demo code for moving the rover arms, starting now:")
        # Move arms
        demo.move_arms(0, 3, 0.8, 500)
        
        print("Demo code for drilling, starting now:")
        # Drill
        demo.send_command('L', 255.0)

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Close the serial connection
        demo.close()


if __name__ == "__main__":
    main()