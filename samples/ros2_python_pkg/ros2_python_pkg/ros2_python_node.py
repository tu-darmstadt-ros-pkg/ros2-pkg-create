import sys
from typing import Any, Optional, Union

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import (FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult)
from std_msgs.msg import Int32


class Ros2PythonNode(Node):

    def __init__(self):
        """Constructor"""

        super().__init__("ros2_python_node")

        self.auto_reconfigurable_params = []
        self.parameters_callback = None
        self.subscriber = None
        self.publisher = None

        self.param = 1.0
        self.param = self.declareAndLoadParameter(name="param",
                                                  param_type=rclpy.Parameter.Type.DOUBLE,
                                                  description="TODO",
                                                  default=self.param,
                                                  from_value=0.0,
                                                  to_value=10.0,
                                                  step_value=0.1)

        self.setup()

    def declareAndLoadParameter(self,
        name: str,
        param_type: rclpy.Parameter.Type,
        description: str,
        default: Optional[Any] = None,
        add_to_auto_reconfigurable_params: bool = True,
        is_required: bool = False,
        read_only: bool = False,
        from_value: Optional[Union[int, float]] = None,
        to_value: Optional[Union[int, float]] = None,
        step_value: Optional[Union[int, float]] = None,
        additional_constraints: str = "") -> Any:
        """Declares and loads a ROS parameter

        Args:
            name (str): name
            param_type (rclpy.Parameter.Type): parameter type
            description (str): description
            default (Optional[Any], optional): default value
            add_to_auto_reconfigurable_params (bool, optional): enable reconfiguration of parameter
            is_required (bool, optional): whether failure to load parameter will stop node
            read_only (bool, optional): set parameter to read-only
            from_value (Optional[Union[int, float]], optional): parameter range minimum
            to_value (Optional[Union[int, float]], optional): parameter range maximum
            step_value (Optional[Union[int, float]], optional): parameter range step
            additional_constraints (str, optional): additional constraints description

        Returns:
            Any: parameter value
        """

        # declare parameter
        param_desc = ParameterDescriptor()
        param_desc.description = description
        param_desc.additional_constraints = additional_constraints
        param_desc.read_only = read_only
        if from_value is not None and to_value is not None:
            if param_type == rclpy.Parameter.Type.INTEGER:
                step_value = step_value if step_value is not None else 1
                param_desc.integer_range = [IntegerRange(from_value=from_value, to_value=to_value, step=step_value)]
            elif param_type == rclpy.Parameter.Type.DOUBLE:
                step_value = step_value if step_value is not None else 1.0
                param_desc.floating_point_range = [FloatingPointRange(from_value=from_value, to_value=to_value, step=step_value)]
            else:
                self.get_logger().warn(f"Parameter type of parameter '{name}' does not support specifying a range")
        self.declare_parameter(name, param_type, param_desc)

        # load parameter
        try:
            param = self.get_parameter(name).value
            self.get_logger().info(f"Loaded parameter '{name}': {param}")
        except rclpy.exceptions.ParameterUninitializedException:
            if is_required:
                self.get_logger().fatal(f"Missing required parameter '{name}', exiting")
                sys.exit(1) # TODO: rclpy shutdown?
            else:
                self.get_logger().warn(f"Missing parameter '{name}', using default value: {default}")
                param = default

        # add parameter to auto-reconfigurable parameters
        if add_to_auto_reconfigurable_params:
            # TODO: this probably doesn't work
            self.auto_reconfigurable_params.append((name, param))

        return param

    def parametersCallback(self,
                           parameters: rclpy.Parameter) -> SetParametersResult:
        """Handles reconfiguration when a parameter value is changed

        Args:
            parameters (rclpy.Parameter): parameters

        Returns:
            SetParametersResult: parameter change result
        """

        for param in parameters:
            for auto_reconfigurable_param in self.auto_reconfigurable_params:
                if param.name == auto_reconfigurable_param[0]:
                    # TODO: this probably doesn't work
                    auto_reconfigurable_param[1] = param.value
                    break

        result = SetParametersResult()
        result.successful = True

        return result

    def setup(self):
        """Sets up subscribers, publishers, etc. to configure the node"""

        # callback for dynamic parameter configuration
        self.parameters_callback = self.add_on_set_parameters_callback(
            self.parametersCallback)

        # subscriber for handling incoming messages
        self.subscriber = self.create_subscription(Int32,
                                                   "~/input",
                                                   self.topicCallback,
                                                   qos_profile=10)
        self.get_logger().info(f"Subscribed to '{self.subscriber.topic_name}'")

        # publisher for publishing outgoing messages
        self.publisher = self.create_publisher(Int32,
                                               "~/output",
                                               qos_profile=10)
        self.get_logger().info(f"Publishing to '{self.publisher.topic_name}'")

    def topicCallback(self, msg: Int32):
        """Processes messages received by a subscriber

        Args:
            msg (Int32): message
        """

        self.get_logger().info(f"Message received: '{msg.data}'")


def main():

    rclpy.init()
    rclpy.spin(Ros2PythonNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
