#!/usr/bin/env python3

from datetime import datetime

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
        super().__init__(node_name="fsm_node")
        self.fsm = fsm
        self.get_logger().info("fsm node start")
        self.pub = self.create_publisher(msg_type=String, topic="demo_topic", qos_profile=10)
        self.create_service(srv_type=Empty, srv_name="/srv_goto_work", callback=self._cb_srv_goto_work)
        self.create_service(srv_type=Empty, srv_name="/srv_goto_idle", callback=self._cb_srv_goto_idle)
        self.create_service(srv_type=Empty, srv_name="/srv_print_state", callback=self._cb_srv_print_state)

    def _cb_srv_goto_work(self, req: Empty.Request, res: Empty.Response):
        """
        """
        self.get_logger().info("_cb_srv_goto_work")
        self.fsm.goto_work()
        return res

    def _cb_srv_goto_idle(self, req: Empty.Request, res: Empty.Response):
        """
        """
        self.get_logger().info("_cb_srv_goto_idle")
        self.fsm.goto_idle()
        return res

    def _cb_srv_print_state(self, req: Empty.Request, res: Empty.Response):
        """
        """
        self.get_logger().info("_cb_srv_print_state " + self.fsm.state)
        return res

    def _cb_srv_force_idle(self, req: Empty.Request, res: Empty.Response):
        """
        """
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
        """
        """
        self.node = MyNode(fsm=self)
        self.machine = WebMachine(model=self, states=__class__.states,
            transitions=__class__.transitions, initial="idle", name="My FSM")
        self.state_work_timer = None

    def enter_idle(self):
        """
        """
        self.node.get_logger().info("enter state idle")
        if self.state_work_timer:
            self.state_work_timer.cancel()

    def enter_work(self):
        """
        """
        self.node.get_logger().info("enter state work")
        self.state_work_timer = self.node.create_timer(timer_period_sec=1.0, callback=self._cb_state_work_timer)

    def _cb_state_work_timer(self):
        """
        """
        msg = String()
        msg.data = datetime.now().strftime("%H:%M:%S")
        self.last_job = str(msg.data)
        self.node.pub.publish(msg=msg)


def main():
    rclpy.init(args=None)
    fsm = MyFSM()
    rclpy.spin(node=fsm.node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
