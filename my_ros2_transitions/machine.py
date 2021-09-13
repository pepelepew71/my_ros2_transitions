#!/usr/bin/env python3

import threading

from transitions import Machine, State
from transitions_gui import WebMachine
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Empty


class MyNode(Node):

    def __init__(self, fsm):
        """
        """
        super().__init__(node_name="state_machine_node")
        self.fsm = fsm
        self.get_logger().info("state machine node starting ...")
        self.pub = self.create_publisher(msg_type=String, topic="demo_topic", qos_profile=10)
        self.create_service(srv_type=Empty, srv_name="/srv_goto_work", callback=self._cb_srv_goto_work)
        self.create_service(srv_type=Empty, srv_name="/srv_goto_idle", callback=self._cb_srv_goto_idle)
        self.create_service(srv_type=Empty, srv_name="/srv_print_state", callback=self._cb_srv_print_state)
        self.create_service(srv_type=Empty, srv_name="/srv_force_idle", callback=self._cb_srv_force_idle)

    def _cb_srv_goto_work(self, req: Empty.Request, res: Empty.Response):
        self.get_logger().info("_cb_srv_goto_work")
        self.fsm.goto_work()
        return res

    def _cb_srv_goto_idle(self, req: Empty.Request, res: Empty.Response):
        self.get_logger().info("_cb_srv_goto_idle")
        self.fsm.goto_idle()
        return res

    def _cb_srv_print_state(self, req: Empty.Request, res: Empty.Response):
        self.get_logger().info("_cb_srv_print_state " + self.fsm.state)
        return res

    def _cb_srv_force_idle(self, req: Empty.Request, res: Empty.Response):
        self.get_logger().info("_cb_srv_force_idle")
        return res


class MyFSM(object):

    states = [
        State(name="idle", on_enter="enter_idle", ignore_invalid_triggers=True),
        State(name="work", on_enter="enter_work", ignore_invalid_triggers=True),
    ]

    transitions = [
        {"trigger": "goto_work", "source": "idle", "dest": "work"},
        {"trigger": "goto_idle", "source": "work", "dest": "idle"},
    ]

    def __init__(self):
        self.node = MyNode(fsm=self)
        self.machine = WebMachine(model=self, states=__class__.states,
            transitions=__class__.transitions, initial="idle", name="My FSM")

    def enter_idle(self):
        msg = String()
        msg.data = "enter state idle"
        self.node.pub.publish(msg=msg)
        self.node.get_logger().info(msg.data)

    def enter_work(self):
        msg = String()
        msg.data = "enter state work"
        self.node.pub.publish(msg=msg)
        self.node.get_logger().info(msg.data)

        # rate = self.node.create_rate(1)
        # while rclpy.ok():
        #     rate.sleep()
        #     if self.stop_state:
        #         self.stop_state = False
        #         break
        #     else:
        #         msg.data = "keep working"
        #         self.node.pub.publish(msg=msg)
        # self.go_to_idle()


def main():
    rclpy.init(args=None)
    fsm = MyFSM()
    rclpy.spin(node=fsm.node)
    # thread = threading.Thread(target=rclpy.spin, args=(fsm.node, ), daemon=True)
    # thread.start()
    # thread.join()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
