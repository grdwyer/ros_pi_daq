import time

import rclpy
from rclpy.node import Node
from gpiozero import LED, Button
from daq_interfaces.msg import GPIOValues
from daq_interfaces.srv import SetGPIOValues, ModifyGPIOSetup


class DAQServer(Node):
    def __init__(self):
        super().__init__('daq_server')
        self.declare_parameter("input_pins", [])
        self.declare_parameter("output_pins", [])
        self.declare_parameter("update_rate", 0.01)  # 100 hz

        self.input_pins = {}
        self.output_pins = {}
        self.assigned_pins = []

        self.publisher_inputs = self.create_publisher(GPIOValues, "/daq_server/inputs", 10)
        self.publisher_outputs = self.create_publisher(GPIOValues, "/daq_server/outputs", 10)
        self.service_set_output = self.create_service(SetGPIOValues, "/daq_server/set_output", self.callback_set_output)
        self.service_setup_pins = self.create_service(ModifyGPIOSetup, "/daq_server/configure_pins",
                                                      self.callback_configure_pins)
        self.timer_publish_input = None
        self.get_logger().info("Configuring GPIO pins")
        self.configure_pins()

    def destroy_node(self) -> bool:
        super().destroy_node()
        for pin in self.output_pins:
            if self.output_pins[pin].is_lit:
                self.output_pins[pin].off()
                # time.sleep(0.1)
            # self.output_pins[pin].close()

        for pin in self.input_pins:
            self.input_pins[pin].close()

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
                self.get_logger().info("Setting up pin {} as an output".format(pin))
                self.output_pins[pin] = LED(pin)
                self.assigned_pins.append(pin)
            else:
                self.get_logger().warn("Tried to configure pin {} which has already been configured".format(pin))

        if len(self.input_pins) > 1:
            self.get_logger().info("Configured pins now starting input callbacks")
            update_rate = self.get_parameter("update_rate").get_parameter_value().double_value
            self.timer_publish_input = self.create_timer(update_rate, self.callback_get_state)

    def callback_set_output(self, request: SetGPIOValues.Request, response: SetGPIOValues.Response):
        for pin, value in zip(request.gpio.name, request.gpio.value):
            # Check pin exists in output dict
            if pin in self.output_pins:
                if value == 0:
                    self.output_pins[pin].off()
                    response.success = True
                elif value == 1:
                    self.output_pins[pin].on()
                    response.success = True
                else:
                    self.get_logger().warn("Received request to set pin {} to {} which is neither 0 or 1".format(pin,
                                                                                                                 value))
                    response.message += "\nfailed to set pin {} as value was {}".format(pin, value)
                    response.success = False
            else:
                self.get_logger().warn("Received request to set pin {} which is not setup as an output".format(pin))
                response.message += "\nfailed to set pin {} as it is not setup as an output".format(pin)
                response.success = False

        return response

    def callback_get_state(self):
        msg = GPIOValues()
        for pin in self.input_pins:
            msg.name.append(pin)
            msg.value.append(self.input_pins[pin].value)

        self.publisher_inputs.publish(msg)

        msg = GPIOValues()
        for pin in self.output_pins:
            msg.name.append(pin)
            msg.value.append(self.output_pins[pin].value)

        self.publisher_outputs.publish(msg)

    def callback_configure_pins(self, request: ModifyGPIOSetup.Request, response: ModifyGPIOSetup.Response):
        # Check the pin numbers are the same length as operations otherwise return false
        if len(request.pin_numbers) == len(request.operations):
            fails = 0
            for pin, operation in zip(request.pin_numbers, request.operations):
                # Check the pin and request makes sense either
                #    * in assigned_pins and remove
                #    * not in assigned and input or output

                if pin in self.assigned_pins and operation == ModifyGPIOSetup.Request.REMOVE:
                    # Request to remove pin received
                    # Find the pin in input or output
                    if pin in self.input_pins:
                        self.input_pins[pin].close()
                        self.input_pins.pop(pin)
                        self.assigned_pins.remove(pin)
                        msg = "Pin {} removed from input pins \n".format(pin)
                        response.message += msg
                        self.get_logger().info(msg)
                    elif pin in self.output_pins:
                        if self.output_pins[pin].is_lit: self.output_pins[pin].off()
                        self.output_pins[pin].close()
                        self.output_pins.pop(pin)
                        self.assigned_pins.remove(pin)
                        msg = "Pin {} removed from output pins".format(pin)
                        response.message += msg
                        self.get_logger().info(msg)
                    else:
                        self.assigned_pins.remove(pin)
                        msg = "Pin {} was assigned but not found as input or output, removing from assigned pins".format(pin)
                        response.message += msg
                        self.get_logger().warn(msg)
                        fails +=1

                elif pin not in self.assigned_pins and operation == ModifyGPIOSetup.Request.INPUT:
                    msg = "Adding pin {} as Input".format(pin)
                    self.get_logger().info(msg)
                    response.message += msg
                    self.input_pins[pin] = Button(pin, pull_up=True)
                    self.assigned_pins.append(pin)

                elif pin not in self.assigned_pins and operation == ModifyGPIOSetup.Request.OUTPUT:
                    msg = "Adding pin {} as output".format(pin)
                    self.get_logger().info(msg)
                    response.message += msg
                    self.output_pins[pin] = LED(pin)
                    self.assigned_pins.append(pin)

                # Incorrect setups (can't be done things for warnings)
                elif pin in self.assigned_pins and (operation == ModifyGPIOSetup.Request.OUTPUT or
                                                    operation == ModifyGPIOSetup.Request.INPUT):
                    msg = "Pin {} is already assigned, please remove the pin first".format(pin)
                    self.get_logger().warn(msg)
                    response.message += msg
                    fails += 1

                elif pin not in self.assigned_pins and operation == ModifyGPIOSetup.Request.REMOVE:
                    msg = "Pin {} is not assigned and therefore can't be removed".format(pin)
                    self.get_logger().warn(msg)
                    response.message += msg
                    fails += 1

                else:
                    msg = "This case had not been considered, request was pin {} and operation {}\nOutput is 1, " \
                          "Input is 2 and Remove is 3".format(pin, operation)
                    self.get_logger().warn(msg)
                    response.message += msg
                    fails += 1

            if fails > 0:
                response.success = False
                self.get_logger().warn("Failed to modify {} pins out of {} requests".format(fails, len(request.pin_numbers)))
            else:
                response.success = True
        else:
            msg = "Recieved {} pins to change and {} operations. The values need to be the same.".format(
                len(request.pin_numbers), len(request.operations))
            self.get_logger().warn(msg)
            response.message += msg
            response.success = False
        return response


def main(args=None):
    try:
        rclpy.init(args=args)

        server = DAQServer()
        rclpy.spin(server)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
