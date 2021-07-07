import rclpy
import rclpy.node
from gpiozero import LED, Button


class DAQServer(rclpy.Node):
    def __init__(self):
        super().__init__('daq_server')
        self.declare_parameter("input_pins", [])
        self.declare_parameter("output_pins", [])

        self.input_pins = {}
        self.output_pins = {}
        self.assigned_pins = []

    def configure_pins(self):
        # Use params to configure the pins wanted and if they should be inputs or outputs (button or LED)
        input_pins = self.get_parameter("input_pins").get_parameter_value().integer_array_value
        output_pins = self.get_parameter("output_pins").get_parameter_value().integer_array_value

        for pin in input_pins:
            # Check pin number is not already assigned
            if pin not in self.assigned_pins:
                self.get_logger().info("Setting up pin {} as an input".format(pin))
                self.input_pins[pin] = Button(pin, pull_up=True)
                self.assigned_pins.append(pin)
            else:
                self.get_logger().warn("Tried to configure pin {} which has already been configured".format(pin))

        for pin in output_pins:
            # Check pin number is not already assigned
            if pin not in self.assigned_pins:
                self.get_logger().info("Setting up pin {} as an input".format(pin))
                self.output_pins[pin] = LED(pin)
                self.assigned_pins.append(pin)
            else:
                self.get_logger().warn("Tried to configure pin {} which has already been configured".format(pin))

