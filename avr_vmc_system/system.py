from threading import Thread
from time import sleep
import subprocess
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from avr_vmc_system_interfaces.srv import SetNetworkConfig


class SystemNode(Node):
    def __init__(self) -> None:
        super().__init__('system', namespace='system')

        self.estop_publisher = self.create_publisher(Empty, '/estop', 10)
        self.pcc_restart_client = self.create_client(Trigger, '/pcc/reset')

        while not self.pcc_restart_client.wait_for_service(timeout_sec=2.5):
            self.get_logger().info('pcc reset service not available, waiting again...')

        self.shutdown_vmc_srv = self.create_service(
            Trigger,
            'shutdown_vmc',
            self.shutdown_vmc
        )
        self.restart_pcc_srv = self.create_service(
            Trigger,
            'restart_pcc',
            self.restart_pcc
        )
        self.restart_launch_srv = self.create_service(
            Trigger,
            'restart_launch',
            self.restart_launch
        )
        self.set_network_srv = self.create_service(
            SetNetworkConfig,
            'set_network',
            self.set_network
        )

    def shutdown_vmc(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        #estop_publisher.publish(Empty())

        #sleep(1)

        os.system('(sleep 5; sudo shutdown now) &')

        response.success = True
        response.message = 'shutting down in 5 seconds'
 
        return response

    def restart_pcc(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self.pcc_restart_client.call_async(request)

        response.success = True
        response.message = 'request sent'
 
        return response

    def restart_launch(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        proc = subprocess.Popen(["sudo","systemctl","restart","vmc"], stdout=subprocess.PIPE)
        try:
            proc.communicate(timeout=10)
        except TimeoutExpired:
            proc.kill()
            proc.communicate()

        response.success = proc.returncode == 0
        response.message = f"{proc.returncode}"
 
        return response

    def set_network(self, request: SetNetworkConfig.Request, response: SetNetworkConfig.Response) -> SetNetworkConfig.Response:
        proc = subprocess.Popen(["sudo","nmcli","c","up", request.name], stdout=subprocess.PIPE)
        try:
            proc.communicate(timeout=10)
        except TimeoutExpired:
            proc.kill()
            proc.communicate()

        response.success = proc.returncode == 0
 
        return response


def main() -> None:
    rclpy.init()
    node = SystemNode()
    executor = rclpy.executors.MultiThreadedExecutor()
    rclpy.spin(node, executor)


if __name__ == '__main__':
    main()
