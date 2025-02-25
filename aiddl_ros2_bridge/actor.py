#! /usr/bin/env python3

import logging
import rclpy
import atexit

from rclpy.node import Node
from rclpy.action import ActionClient

from aiddl_core.container import Container
from aiddl_external_grpc_python.converter import Converter
from aiddl_external_grpc_python.actor import ActorServer
import aiddl_external_grpc_python.generated.actor_pb2 as actor_pb2
import aiddl_external_grpc_python.generated.aiddl_pb2 as aiddl_pb2
from aiddl_external_ros2.util import load_class_from_string

from action_msgs.msg import GoalStatus


class ActionlibActorServer(ActorServer, Node):
    def __init__(self):
        Node.__init__(self, 'aiddl_action_client')

        self.declare_parameter('grpc_port', 8061)
        self.declare_parameter('converter_class', "none")
        self.declare_parameter('ros_class', "none")
        self.declare_parameter('ros_topic', "topic")
        self.declare_parameter('verbose', False)

        grpc_port = self.get_parameter('grpc_port').get_parameter_value().integer_value
        converter_class_str = self.get_parameter('converter_class').get_parameter_value().string_value

        ros_action_class_str = self.get_parameter('ros_class').get_parameter_value().string_value
        ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        logging.info(
            f'Starting actor for {ros_action_class_str} to topic "{ros_topic}" from AIDDL gRPC receiver port {grpc_port}')

        converter_class = load_class_from_string(converter_class_str)

        ros_action_class = load_class_from_string(ros_action_class_str)

        self._action_client = ActionClient(self, ros_action_class, ros_topic)

        ActorServer.__init__(self, grpc_port)

        c = Container()
        self.conv = Converter(c)

        self.current_id = 0
        self.topic = ros_topic
        self.ros_goal_future = None
        self.status_history = {}
        self.f_extract_request = lambda x: converter_class.request_aiddl2ros(self.conv.pb2aiddl(x))
        self.f_extract_result = lambda x: converter_class.result_ros2aiddl(x)
        self.f_extract_feedback = lambda x: converter_class.feedback_ros2aiddl(x)
        self.feedback = None
        self.result = None
        self.goal_status = None

        def f_is_supported(goal):
            try:
                self.f_extract_request(goal)
                return True
            except Exception as e:
                return False

        self.f_is_supported = f_is_supported

    def _feedback_handler(self, fb):
        print("Handling feedback...")
        print(fb)
        self.goal_status = GoalStatus.STATUS_EXECUTING
        if self.f_extract_feedback is None:
            return None
        self.feedback = self.f_extract_feedback(fb)

    def _response_handler(self, future):
        print("Handling response...")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_status = GoalStatus.STATUS_ERROR
            return

        self.goal_status = GoalStatus.STATUS_EXECUTING
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_handler)

    def _result_handler(self, future):
        print("Handling result...")
        result = future.result()
        print(result)
        self.goal_status = GoalStatus.STATUS_SUCCEEDED # result.status
        self.result = self.f_extract_result(result)

    def IsSupported(self, request, context):
        is_supported = self.f_is_supported(request)
        print('Is %s supported? %s' % (str(request), str(is_supported)))
        r = actor_pb2.Supported(is_supported=is_supported)
        print('Response:', r)
        return r

    def Dispatch(self, request, context):
        print("DISPATCH")
        self.current_id += 1
        self._action_client.wait_for_server()
        goal = self.f_extract_request(request)
        print(goal)
        self.ros_goal_send_future = self._action_client.send_goal_async(goal, self._feedback_handler)
        self.ros_goal_send_future.add_done_callback(self._response_handler)
        self.goal_status = GoalStatus.STATUS_ACCEPTED
        return self.currentGoalToStatus()

    def GetStatus(self, request, context):
        print("GET STATUS")
        return self.currentGoalToStatus()

    def Cancel(self, request, context):
        return actor_pb2.Status(
            id=self.next_id,
            status=2,
            feedback=aiddl_pb2.AiddlStr(""),
            msg=""
        )

    def currentGoalToStatus(self):
        pb_status = None
        # status = self._action_client.status
        # status = self.ros_goal_future.result().status
        status = self.goal_status
        # status = self.ros_goal_future.result().status
        print(f"STATUS: {status}")
        if status == GoalStatus.STATUS_ACCEPTED:
            pb_status = actor_pb2.PENDING
        elif status == GoalStatus.STATUS_EXECUTING:
            pb_status = actor_pb2.ACTIVE
        elif status == GoalStatus.STATUS_CANCELED:
            pb_status = actor_pb2.PREEMPTED
        elif status == GoalStatus.STATUS_SUCCEEDED:
            pb_status = actor_pb2.SUCCEEDED
        elif status == GoalStatus.STATUS_ABORTED:
            pb_status = actor_pb2.RECALLED
        elif status == GoalStatus.STATUS_CANCELLING:
            pb_status = actor_pb2.RECALLING
        elif status == GoalStatus.STATUS_UNKNOWN:
            pb_status = actor_pb2.ERROR
        else:
            pb_status = actor_pb2.ERROR
            print("Unknown status:", status)

        feedback = ""
        if self.result is not None:
            feedback = str(self.result)
        elif self.feedback is not None:
            feedback = str(self.feedback)

        r = actor_pb2.Status(
            id=self.current_id,
            state=pb_status,
            feedback=self.conv.aiddl2pb(feedback),
            msg=""
        )
        self.status_history[self.current_id] = r
        return r


def main(args=None):

    rclpy.init(args=args)

    print('Creating actor server')
    server = ActionlibActorServer()
    def exit_handler():
        print('Closing down...')
        server.server.stop(2).wait()
        print('Done.')
    atexit.register(exit_handler)

    print('Starting server...')
    server.start()
    print('Running.')
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
