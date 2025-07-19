#!/usr/bin/env python3

"""
Mock服务模块，用于模拟ROS服务调用。
在实际机器人硬件不可用时提供模拟响应。
"""

import threading
from typing import Callable, Dict, Any, Optional
import time


class MockResponse:
    """模拟的ROS服务响应。"""
    
    def __init__(self, success: bool = True, message: str = "Mock service call succeeded"):
        """初始化响应。"""
        self.success = success
        self.message = message


class MockClient:
    """模拟的ROS服务客户端。"""
    
    def __init__(self):
        """初始化客户端。"""
        pass
    
    def wait_for_service(self, timeout: float = None) -> bool:
        """
        等待服务可用。
        
        Args:
            timeout: 超时时间，以秒为单位。
        
        Returns:
            布尔值，表示服务是否可用。
        """
        return True
    
    def call_async(self, request: Dict[str, Any]) -> threading.Event:
        """
        异步调用服务。
        
        Args:
            request: 请求参数。
        
        Returns:
            事件对象，可以用于等待响应。
        """
        event = threading.Event()
        threading.Thread(target=self._simulate_call, args=(event, request), daemon=True).start()
        return event
    
    def _simulate_call(self, event: threading.Event, request: Dict[str, Any]) -> None:
        """
        模拟服务调用。
        
        Args:
            event: 事件对象，用于标记响应已准备好。
            request: 请求参数。
        """
        # 模拟服务调用延迟
        time.sleep(0.1)
        # 设置响应
        self.response = MockResponse()
        # 触发事件
        event.set()


class JointControlClient(MockClient):
    """模拟的关节控制服务客户端。"""
    
    def __init__(self):
        """初始化客户端。"""
        super().__init__()


class VacuumCmdClient(MockClient):
    """模拟的真空吸盘命令服务客户端。"""
    
    def __init__(self):
        """初始化客户端。"""
        super().__init__()


def create_mock_response(success: bool = True, message: str = "Mock service call succeeded") -> MockResponse:
    """
    创建模拟响应。
    
    Args:
        success: 是否成功。
        message: 响应消息。
    
    Returns:
        模拟响应对象。
    """
    return MockResponse(success, message) 