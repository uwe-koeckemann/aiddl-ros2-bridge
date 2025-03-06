#! /usr/bin/env python3
import atexit
import logging
import rclpy
from rclpy.node import Node

from aiddl_core.container import Container
from aiddl_external_grpc_python.converter import Converter
from aiddl_external_grpc_python.function import FunctionServer
from aiddl_external_ros2.util import load_class_from_string


class ServiceCallServer(FunctionServer, Node):
    def __init__(self):
        Node.__init__(self, 'aiddl_service_call')
        self.declare_parameter('grpc_port', 8066)
        self.declare_parameter('converter_class', "none")
        self.declare_parameter('ros_class', "none")
        self.declare_parameter('ros_topic', "topic")
        self.declare_parameter('verbose', False)

        grpc_port = self.get_parameter('grpc_port').get_parameter_value().integer_value
        converter_class_str = self.get_parameter('converter_class').get_parameter_value().string_value
        ros_srv_class_str = self.get_parameter('ros_service_class').get_parameter_value().string_value
        ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        self.get_logger().info(f'Starting service for {ros_srv_class_str} on topic "{ros_topic}" from AIDDL gRPC service port {grpc_port}')

        converter_class = load_class_from_string(converter_class_str)
        ros_srv_class = load_class_from_string(ros_srv_class_str)

        FunctionServer.__init__(self, grpc_port)
        self.service_client = self.create_client(ros_srv_class, ros_topic)
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        c = Container()
        conv = Converter(c)
        self.f_convert = lambda x: converter_class.aiddl2ros(conv.pb2aiddl(x))
        
        self.f_in = lambda x: converter_class.request_aiddl2ros(conv.pb2aiddl(x))
        self.f_out = lambda x: converter_class.result_ros2aiddl(x)
        container = Container()
        self.converter = Converter(container)

    def Call(self, request, context):
        args = self.f_in(request)
        out = self.service_client.call(args)
        answer = self.f_out(out)      
        return self.converter.aiddl2pb(answer)


def main(args=None):
    rclpy.init(args=args)
    server = ServiceCallServer()

    def exit_handler():
        server.get_logger().info('Closing down...')
        server.server.stop(2).wait()

    atexit.register(exit_handler)
    self.get_logger().info('Starting server...')
    server.start()

    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
