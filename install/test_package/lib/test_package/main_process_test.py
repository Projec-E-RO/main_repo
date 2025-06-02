#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from test_package.srv import IsSuccess
import sys

class ButtonServiceClient(Node):
    def __init__(self):
        super().__init__('button_service_client')
        ## 임시 메인프로세스 노드 - 3개 서버와 통신
        
        # 서비스 클라이언트 3개 생성
        self.client1 = self.create_client(IsSuccess, 'button_service1')
        self.client2 = self.create_client(IsSuccess, 'button_service2')
        self.client3 = self.create_client(IsSuccess, 'button_service3')
        
        # 서비스들이 준비될 때까지 대기
        self.get_logger().info('Waiting for services...')
        while not self.client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('button_service1 not available, waiting...')
        while not self.client2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('button_service2 not available, waiting...')
        while not self.client3.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('button_service3 not available, waiting...')
        
        self.get_logger().info('All services are ready!')

    def send_request_to_all(self, button_type):
        """3개 서버 모두에게 요청 보내기"""
        self.get_logger().info(f'Sending request to all servers: {button_type}')
        
        # 3개 서버에 동시에 요청
        future1 = self.send_request(self.client1, button_type, "Server1")
        future2 = self.send_request(self.client2, button_type, "Server2")
        future3 = self.send_request(self.client3, button_type, "Server3")
        
        return [future1, future2, future3]

    def send_request(self, client, button_type, server_name):
        """특정 서버에 요청 보내기"""
        request = IsSuccess.Request()
        request.button_type = button_type
        
        self.get_logger().info(f'Sending to {server_name}: {button_type}')
        
        # 비동기 서비스 호출
        future = client.call_async(request)
        return future

def main():
    rclpy.init()
    
    # 명령행 인수 확인
    if len(sys.argv) != 2:
        print('Usage: ros2 run test_package button_service_client <button_type>')
        print('Example: ros2 run test_package button_service_client button1')
        print('Available: button1, button2, button3, button_up, button_down, bottle')
        return
    
    button_type = sys.argv[1]
    node = ButtonServiceClient()
    
    try:
        # 3개 서버 모두에게 요청 보내기
        futures = node.send_request_to_all(button_type)
        
        # 모든 응답 대기
        for i, future in enumerate(futures, 1):
            rclpy.spin_until_future_complete(node, future)
            
            if future.result() is not None:
                response = future.result()
                node.get_logger().info(f'Server{i} Result: success = {response.success}')
            else:
                node.get_logger().error(f'Server{i} call failed')
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
