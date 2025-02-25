import logging
import os
import atexit
import rclpy
from rclpy.node import Node

import aiddl_external_grpc_python.generated.receiver_pb2 as receiver_pb2
from aiddl_core.container import Container
from aiddl_external_grpc_python.converter import Converter
from aiddl_external_grpc_python.receiver import ReceiverServer
from aiddl_external_ros2.util import load_class_from_string


class TopicReceiverServer(ReceiverServer, Node):
    def __init__(self):
        Node.__init__(self, 'aiddl_receiver')

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

        converter_class = load_class_from_string(converter_class_str)
        ros_msg_class = load_class_from_string(ros_msg_class_str)

        super(TopicReceiverServer, self).__init__(grpc_port)

        print(f'Starting receiver for {ros_msg_class_str} from topic "{ros_topic}" to AIDDL gRPC receiver port {grpc_port}')
    
        self.f_convert = lambda x: converter_class.ros2aiddl(x)
        # rospy.Subscriber(ros_topic, ros_msg_type, self._callback)
        self.subscription = self.create_subscription(ros_msg_class, ros_topic, self._callback, 10)
        self.queue_lock = False
        self.message_queue = []
        container = Container()
        self.converter = Converter(container)

    def _callback(self, data):
        while self.queue_lock:
            pass

        self.queue_lock = True
        print(data)
        self.message_queue.append(data)
        self.queue_lock = False

    def Receive(self, request, context):
        sort_by = request.sort_by
        pull_order = request.pull_order
        receive_max = request.pull_max
        flush_queue = request.flush_queue

        while self.queue_lock:
            pass
        self.queue_lock = True

        n_receive = receive_max
        if n_receive == -1:
            n_receive = len(self.message_queue)

        pulled = []

        while len(pulled) < n_receive and len(self.message_queue) > 0:
            if pull_order == receiver_pb2.OLDEST_FIRST:
                item = self.message_queue[0]
                del self.message_queue[0]
            elif pull_order == receiver_pb2.NEWEST_FIRST:
                item = self.message_queue[-1]
                del self.message_queue[-1]
            pulled.append(item)
        if flush_queue:
            self.message_queue = []

        self.queue_lock = False

        if sort_by == receiver_pb2.OLDEST_FIRST and pull_order == receiver_pb2.NEWEST_FIRST:
            pulled.reverse()
        elif sort_by == receiver_pb2.NEWEST_FIRST and pull_order == receiver_pb2.OLDEST_FIRST:
            pulled.reverse()

        answer = [self.converter.aiddl2pb(self.f_convert(x)) for x in pulled]
        return receiver_pb2.Messages(messages=answer)


def main(args=None):    
    rclpy.init(args=args)

    server = TopicReceiverServer()

    def exit_handler():
        print('Closing down...')
        server.server.stop(2).wait()
        print('Done.')
        
    atexit.register(exit_handler)

    print('Starting server...')
    server.start()
    print('Running.')
    rclpy.spin(server)


if __name__ == '__main__':
    main()
