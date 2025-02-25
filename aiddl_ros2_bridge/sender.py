#! /usr/bin/env python3
import atexit
import logging
import rclpy
from rclpy.node import Node

from aiddl_core.container import Container
from aiddl_external_grpc_python.converter import Converter
from aiddl_external_grpc_python.sender import SenderServer
import aiddl_external_grpc_python.generated.empty_pb2 as empty_pb2
from aiddl_external_ros2.util import load_class_from_string


class TopicSenderServer(SenderServer, Node):
    def __init__(self):
        Node.__init__(self, 'aiddl_sender')
        self.declare_parameter('grpc_port', 8061)
        self.declare_parameter('converter_class', "none")
        self.declare_parameter('ros_class', "none")
        self.declare_parameter('ros_topic', "topic")
        self.declare_parameter('verbose', False)

        grpc_port = self.get_parameter('grpc_port').get_parameter_value().integer_value
        converter_class_str = self.get_parameter('converter_class').get_parameter_value().string_value
        ros_msg_class_str = self.get_parameter('ros_class').get_parameter_value().string_value
        ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        logging.info(f'Starting sender for {ros_msg_class_str} to topic "{ros_topic}" from AIDDL gRPC receiver port {grpc_port}')

        converter_class = load_class_from_string(converter_class_str)
        ros_msg_class = load_class_from_string(ros_msg_class_str)

        SenderServer.__init__(self, grpc_port)
        self.pub = self.create_publisher(ros_msg_class, ros_topic, 10)
        c = Container()
        conv = Converter(c)
        self.f_convert = lambda x: converter_class.aiddl2ros(conv.pb2aiddl(x))

    def Send(self, request, context):
        if self.verbose:
            logging.info(f"Sending: {request}")
        ros_msg = self.f_convert(request)
        if self.verbose:
            logging.info(f"Converted to: {ros_msg}")
        self.pub.publish(ros_msg)
        return empty_pb2.Empty()


def main(args=None):
    rclpy.init(args=args)
    logging.info('Creating sender server...')
    server = TopicSenderServer()

    def exit_handler():
        logging.info('Closing down...')
        server.server.stop(2).wait()
        logging.info('Done.')

    atexit.register(exit_handler)
    logging.info('Starting server...')
    server.start()
    logging.info('Running.')

    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
