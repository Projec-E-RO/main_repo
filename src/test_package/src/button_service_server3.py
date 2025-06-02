#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from test_package.srv import IsSuccess

class ButtonServiceServer3(Node):
    def __init__(self):
        super().__init__('button_service_server3')
        
        # 서비스 서버 생성
        self.srv = self.create_service(
            IsSuccess, 
            'button_service3', 
            self.button_service_callback
        )
        
        self.get_logger().info('Button Service Server3 is ready')

    def button_service_callback(self, request, response):
        """서비스 요청을 처리하는 콜백 함수"""
        self.get_logger().info(f'Server3 - Incoming request: {request.button_type}')
        
        # 서버3은 button3, bottle만 처리
        if request.button_type in ['button3', 'bottle']:
            response.success = True
            self.get_logger().info(f'Server3 - Successfully processed: {request.button_type}')
        else:
            response.success = False
            self.get_logger().info(f'Server3 - Cannot handle: {request.button_type}')
        
        return response

def main():
    rclpy.init()
    node = ButtonServiceServer3()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
