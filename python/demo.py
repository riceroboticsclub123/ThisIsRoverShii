import serial

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


def main():
    # Initialize the serial control demo
    demo = SerialControlDemo(port='/dev/ttyACM0', baud_rate=9600)

    try:
        # # Example commands
        # print("Sending example commands...")

        # # Move motor 1 by 500 steps
        # demo.send_command('C', 1, 500)

        # # Set motor 2 max speed to 80% (unitless)
        # demo.send_command('R', 2, 0.8)

        # # Move motor 0 to position 1000 steps
        # demo.send_command('P', 0, 1000)

        # # Set motor 3 velocity to 100 RPM (unit = 1 for RPM)
        # demo.send_command('V', 3, 100, 1)
        
        # ---------------------------------------------------
        # Demo code for moving the rover wheels
        
        # Move forward
        print("Move forward!")
        demo.send_command('S', 1, 1, 1)  # Set both side motor speed at 1 RPM (unit = 1 for RPM)
        demo.send_command('S', 2, 2, 1)  # Set both side motor speed at 2 RPM (unit = 1 for RPM)
        demo.send_command('S', 3, 3, 1)  # Set both side motor speed at 3 RPM (unit = 1 for RPM)
        demo.send_command('S', 4, 4, 1)  # Set both side motor speed at 4 RPM (unit = 1 for RPM)
        demo.send_command('S', 5, 5, 1)  # Set both side motor speed at 5 RPM (unit = 1 for RPM)
        
        # Move backward
        print("Move backward!")
        demo.send_command('S', -1, -1, 1)  # Set both side motor speed at -1 RPM (unit = 1 for RPM)
        demo.send_command('S', -2, -2, 1)  # Set both side motor speed at -2 RPM (unit = 1 for RPM)
        demo.send_command('S', -3, -3, 1)  # Set both side motor speed at -3 RPM (unit = 1 for RPM)
        demo.send_command('S', -4, -4, 1)  # Set both side motor speed at -4 RPM (unit = 1 for RPM)
        demo.send_command('S', -5, -5, 1)  # Set both side motor speed at -5 RPM (unit = 1 for RPM)
        
        # Try spinning!
        print("Try spinning! (with increasing speed)")
        demo.send_command('S', 1, -1, 1)  # Set leftside motor speed at 1 and right side at -1 RPM (unit = 1 for RPM)
        demo.send_command('S', 2, -2, 1)  # Set leftside motor speed at 2 and right side at -2 RPM (unit = 1 for RPM)
        demo.send_command('S', 3, -3, 1)  # Set leftside motor speed at 3 and right side at -3 RPM (unit = 1 for RPM)
        demo.send_command('S', 4, -4, 1)  # Set leftside motor speed at 4 and right side at -4 RPM (unit = 1 for RPM)
        demo.send_command('S', 5, -5, 1)  # Set leftside motor speed at 5 and right side at -5 RPM (unit = 1 for RPM)
        
        print("Now with different speeds on left and right sides!")
        demo.send_command('S', 2, -1, 1)  # Set leftside motor speed at 2 and right side at -1 RPM (unit = 1 for RPM)
        demo.send_command('S', 3, -2, 1)  # Set leftside motor speed at 3 and right side at -2 RPM (unit = 1 for RPM)
        demo.send_command('S', 4, -3, 1)  # Set leftside motor speed at 4 and right side at -3 RPM (unit = 1 for RPM)
        demo.send_command('S', 5, -4, 1)  # Set leftside motor speed at 5 and right side at -4 RPM (unit = 1 for RPM)
        
        # ---------------------------------------------------
        # Demo code for moving the rover arms
        
        
        
        

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Close the serial connection
        demo.close()


if __name__ == "__main__":
    main()