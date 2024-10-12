# Copyright (c) 2019 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Copied from https://github.com/ros-visualization/rqt_reconfigure/tree/jazzy/src/rqt_reconfigure because rqt_reconfigure comes with 360MB of dependencies

from threading import Event

from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import DescribeParameters
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import ListParameters
from rcl_interfaces.srv import SetParameters

from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_parameter_events

class AsyncServiceCallFailed(Exception):

    def __init__(self, message='asynchronous service call failed', hint=''):
        self.message = message if not hint else message + ': ' + hint
        super().__init__(self.message)


class ParamClient(object):

    def __init__(self, node, remote_node_name, param_change_callback=None):

        self._node = node
        self._remote_node_name = remote_node_name
        self._get_params_client = self._node.create_client(
            GetParameters, '{remote_node_name}/get_parameters'.format_map(locals())
        )
        self._set_params_client = self._node.create_client(
            SetParameters, '{remote_node_name}/set_parameters'.format_map(locals())
        )
        self._list_params_client = self._node.create_client(
            ListParameters, '{remote_node_name}/list_parameters'.format_map(locals())
        )
        self._describe_params_client = self._node.create_client(
            DescribeParameters, '{remote_node_name}/describe_parameters'.format_map(locals())
        )
        self._param_events_subscription = self._node.create_subscription(
            ParameterEvent, '/parameter_events', self._on_parameter_event,
            qos_profile_parameter_events
        )
        self._param_change_callback = param_change_callback

    def _on_parameter_event(self, event):
        if event.node != self._remote_node_name:
            return
        if self._param_change_callback is not None:
            self._param_change_callback(
                [Parameter.from_parameter_msg(p) for p in event.new_parameters],
                [Parameter.from_parameter_msg(p) for p in event.changed_parameters],
                [Parameter.from_parameter_msg(p) for p in event.deleted_parameters]
            )

    def list_parameters(self):
        list_params_request = ListParameters.Request()
        list_params_response = self._call_service(self._list_params_client, list_params_request)
        return list_params_response.result.names

    def get_parameters(self, names):
        get_params_request = GetParameters.Request()
        get_params_request.names = names
        get_params_response = self._call_service(self._get_params_client, get_params_request)
        return [
            Parameter.from_parameter_msg(ParameterMsg(name=name, value=value))
            for name, value in zip(names, get_params_response.values)
        ]

    def describe_parameters(self, names):
        describe_params_request = DescribeParameters.Request()
        describe_params_request.names = names
        describe_params_response = self._call_service(self._describe_params_client,
                                                      describe_params_request)
        return describe_params_response.descriptors

    def set_parameters(self, parameters):
        set_params_request = SetParameters.Request()
        set_params_request.parameters = [p.to_parameter_msg() for p in parameters]
        return self._call_service(self._set_params_client, set_params_request)

    def close(self):
        self._node.destroy_subscription(self._param_events_subscription)
        self._node.destroy_client(self._describe_params_client)
        self._node.destroy_client(self._list_params_client)
        self._node.destroy_client(self._set_params_client)
        self._node.destroy_client(self._get_params_client)

    def _call_service(self, client, request, timeout=1.0):
        if not client.service_is_ready():
            if not client.wait_for_service(timeout):
                raise AsyncServiceCallFailed(hint='timed out waiting for service')

        # It is possible that a node has the parameter services but is not
        # spinning. In that is the case, the client call will time out.
        event = Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: event.set())

        event.wait(timeout)

        result = future.result()
        if result is None:
            raise AsyncServiceCallFailed(hint='the target node may not be spinning')

        return future.result()


def create_param_client(node, remote_node_name, param_change_callback=None):
    return ParamClient(node, remote_node_name, param_change_callback)


def _has_parameters(node, node_name, node_namespace):
    # Get all of the services provided by a node (node_name)
    for service_name, service_types in node.get_service_names_and_types_by_node(
            node_name, node_namespace):

        # Make sure the node supports the ListParameters service
        if 'rcl_interfaces/srv/ListParameters' in service_types:
            return True
    return False


def find_nodes_with_params(node):
    names_and_namespaces = node.get_node_names_and_namespaces()
    node_list = []
    for node_name, node_namespace in names_and_namespaces:
        if _has_parameters(node, node_name, node_namespace):
            full_name = node_namespace.rstrip('/') + '/' + node_name
            node_list.append(full_name)
    return node_list
