import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters

class BaseNode(Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)

        self.client = self.create_client(GetParameters,
                                         '/global_parameters_node/get_parameters')

        self.client.wait_for_service(3)
        if self.client.service_is_ready():
            # Define references in global params file
            pass
        else:
            raise RuntimeError("Can't connect to global_parameters_node.")


    def get_global_parameter(self, name):
        try:
            request = GetParameters.Request()
            request.names = [name]
            self.client.wait_for_service()
            result = self.client.call(request)

        except Exception as e:
            self.get_logger().error("service call failed %r" % (e,))
            raise RuntimeError("Failed to get global parameters.")

        else:
            param = result.values[0]
            self.get_logger().info("Got global param: %s" % (param.string_value,))

        return param.string_value

