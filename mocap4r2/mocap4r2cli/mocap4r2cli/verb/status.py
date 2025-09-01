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

from mocap4r2cli.verb import get_lc_status, get_mocap_systems, VerbExtension

from ros2cli.node.strategy import add_arguments
from ros2cli.node.strategy import NodeStrategy


class StatusVerb(VerbExtension):
    """Prints the status of the MOCAP4ROS2 systems."""

    def add_arguments(self, parser, cli_name):
        add_arguments(parser)

    def main(self, *, args):
        with NodeStrategy(args) as node:
            mocap_systems = get_mocap_systems(node)

            for mocap_system in mocap_systems:
                lc_status, ok = get_lc_status(node, mocap_system)

                if ok:
                    status = 'UNKNOWN'
                    if (lc_status.id == LCState.PRIMARY_STATE_UNKNOWN or
                            lc_status.id == LCState.PRIMARY_STATE_FINALIZED):
                        status = 'NOT READY'
                    elif lc_status.id == LCState.PRIMARY_STATE_UNCONFIGURED:
                        status = 'NOT YET AVAILABLE'
                    elif lc_status.id == LCState.PRIMARY_STATE_INACTIVE:
                        status = 'READY'
                    elif lc_status.id == LCState.PRIMARY_STATE_ACTIVE:
                        status = 'ACTIVE'
                    print(mocap_system + '\t' + status)
                else:
                    print(mocap_system + '\tERROR')
