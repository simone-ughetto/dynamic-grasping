# Copyright 2024 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from lifecycle_msgs.srv import GetState

from mocap4r2_control_msgs.msg import Control
from mocap4r2_control_msgs.msg import MocapInfo

import rclpy
from rclpy.qos import QoSDurabilityPolicy, QoSProfile

from ros2cli.plugin_system import PLUGIN_SYSTEM_VERSION
from ros2cli.plugin_system import satisfies_version


def get_mocap_systems(node):
    clock = node.get_clock()
    start_time = clock.now()
    elapsed = clock.now() - start_time

    qos_profile = QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10)

    mocap_systems = []
    node.create_subscription(
        MocapInfo,
        '/mocap4r2_environment',
        lambda msg: mocap_systems.append(msg.mocap4r2_source),
        qos_profile)

    while rclpy.ok() and elapsed.nanoseconds / 1e9 < 0.5:
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = clock.now() - start_time

    return mocap_systems


def get_lc_status(node, mocap_lc_node):
    srv_name = '/' + mocap_lc_node + '/get_state'
    srv = node.create_client(GetState, srv_name)

    if not srv.wait_for_service(timeout_sec=1.0):
        node.get_logger().error('Mocap System ' + mocap_lc_node +
                                ' does not support MOCAP4ROS2 Control System')
        return (None, False)

    req = GetState.Request()

    future = srv.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    return (future.result().current_state, True)


def send_start_control(node, mocap_lc_node):
    return send_control(node, mocap_lc_node, Control.START, Control.ACK_START)


def send_stop_control(node, mocap_lc_node):
    return send_control(node, mocap_lc_node, Control.STOP, Control.ACK_STOP)


def send_control(node, mocap_lc_node, command, ack_command):
    clock = node.get_clock()
    start_time = clock.now()
    elapsed = clock.now() - start_time

    publisher = node.create_publisher(Control, '/mocap4r2_control', 10)
    msg = Control()
    msg.control_type = command
    msg.stamp = node.get_clock().now().to_msg()
    msg.mocap4r2_source = 'mocap4r2cli'
    msg.session_id = 'cli_session'
    msg.mocap4r2_systems = [mocap_lc_node]
    publisher.publish(msg)

    response = []
    node.create_subscription(
        Control,
        '/mocap4r2_control',
        lambda msg: response.append(msg),
        10)

    while rclpy.ok() and elapsed.nanoseconds / 1e9 < 0.5:
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = clock.now() - start_time

    ok = response[0].control_type == ack_command and response[0].mocap4r2_source == mocap_lc_node

    return ok


class VerbExtension:
    """
    The extension point for 'mocap4r2' verb extensions.

    The following properties must be defined:
    * `NAME` (will be set to the entry point name)

    The following methods must be defined:
    * `main`

    The following methods can be defined:
    * `add_arguments`
    """

    NAME = None
    EXTENSION_POINT_VERSION = '0.1'

    def __init__(self):
        super(VerbExtension, self).__init__()
        satisfies_version(PLUGIN_SYSTEM_VERSION, '^0.1')

    def add_arguments(self, parser, cli_name):
        pass

    def main(self, *, args):
        raise NotImplementedError()
