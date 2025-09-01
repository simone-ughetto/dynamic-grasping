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

from lifecycle_msgs.msg import State as LCState

from mocap4r2cli.api import MocapNameCompleter
from mocap4r2cli.verb import get_lc_status, get_mocap_systems, send_stop_control, VerbExtension

from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy


class StopVerb(VerbExtension):
    """Stop a MOCAP4ROS2 system."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)
        arg = parser.add_argument(
            'mocap_system',
            help='Name of the MOCAP4ROS2 system to stop')
        arg.completer = MocapNameCompleter()

    def main(self, *, args):
        with NodeStrategy(args) as node:
            mocap_systems = get_mocap_systems(node)

            if args.mocap_system in mocap_systems:
                lc_status, ok = get_lc_status(node, args.mocap_system)
                if ok:
                    if lc_status.id == LCState.PRIMARY_STATE_INACTIVE:
                        node.get_logger().warn(args.mocap_system + ' already inactive')
                    elif lc_status.id != LCState.PRIMARY_STATE_ACTIVE:
                        node.get_logger().warn('Unable to stop ' + args.mocap_system)
                    else:
                        stop_ok = send_stop_control(node, args.mocap_system)
                        lc_status, ok = get_lc_status(node, args.mocap_system)

                        if not stop_ok or not ok or lc_status.id != LCState.PRIMARY_STATE_INACTIVE:
                            node.get_logger().error('Error stopping ' + args.mocap_system)
                        else:
                            node.get_logger().info(args.mocap_system + ' stopped')
                else:
                    node.get_logger().error('Error getting status of ' + args.mocap_system)

            else:
                node.get_logger().error('Mocap System ' + args.mocap_system +
                                        ' not in detected MOCAP4ROS2 Control System')
