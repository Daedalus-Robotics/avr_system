import subprocess
import os

import rclpy
from rclpy.node import Node

from avr_system_interfaces.srv import SystemTrigger, SetNetworkConfig


class SystemNode(Node):
    def __init__(self) -> None:
        super().__init__('system', namespace='system')

        self.declare_parameter('service_name', '')
        self.service_name = self.get_parameter('service_name').get_parameter_value().string_value
        if self.service_name == '':
            raise ValueError('service_name parameter is not set')

        self.declare_parameter('enable_subsystem', False)
        self.subsystem_enabled = self.get_parameter('enable_subsystem').get_parameter_value().bool_value

        self.shutdown_srv = self.create_service(
            SystemTrigger,
            'shutdown',
            self.shutdown
        )
        self.restart_launch_srv = self.create_service(
            SystemTrigger,
            'restart',
            self.restart_launch
        )
        self.set_network_srv = self.create_service(
            SetNetworkConfig,
            'set_network',
            self.set_network
        )

        if self.subsystem_enabled:
            self.sub_shutdown_client = self.create_client(
                SystemTrigger,
                'subsystem/shutdown'
            )
            self.sub_restart_launch_client = self.create_client(
                SystemTrigger,
                'subsystem/restart'
            )

        self.get_logger().info('Ready')

    def shutdown(
            self,
            request: SystemTrigger.Request,
            response: SystemTrigger.Response
    ) -> SystemTrigger.Response:
        if request.include_subsystem and self.subsystem_enabled:
            self.sub_shutdown_client.call_async(SystemTrigger())

        success = os.system('(sleep 5; sudo shutdown now) &') == 0

        response.success = success
        response.message = 'shutting down in 5 seconds' if success else 'failed to shutdown'

        return response

    def restart_launch(
            self,
            request: SystemTrigger.Request,
            response: SystemTrigger.Response
    ) -> SystemTrigger.Response:
        if request.include_subsystem and self.subsystem_enabled:
            self.sub_restart_launch_client.call_async(SystemTrigger())

        proc = subprocess.Popen(['sudo', 'systemctl', 'restart', self.service_name], stdout=subprocess.PIPE)
        try:
            proc.communicate(timeout=10)
        except TimeoutExpired:
            proc.kill()
            proc.communicate()

        response.success = proc.returncode == 0

        return response

    @staticmethod
    def set_network(
            request: SetNetworkConfig.Request,
            response: SetNetworkConfig.Response
    ) -> SetNetworkConfig.Response:
        proc = subprocess.Popen(['sudo', 'nmcli', 'c', 'up', request.name], stdout=subprocess.PIPE)
        try:
            proc.communicate(timeout=10)
        except TimeoutExpired:
            proc.kill()
            proc.communicate()

        response.success = proc.returncode == 0

        return response


def main() -> None:
    rclpy.init()
    try:
        node = SystemNode()
        executor = rclpy.executors.MultiThreadedExecutor()
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
