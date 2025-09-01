import asyncio
import websockets
import socket
import time
import signal
import sys
from google.protobuf.any_pb2 import Any

# Import protobuf generated classes
from rovlink_proto.package_pb2 import Package
from rovlink_proto.device_pb2 import DeviceType
from rovlink_proto.rovlink.rovlink_pb2 import RovlinkFrame, RovFrameType, RovDeviceType
from rovlink_proto.rovlink.sensor_pb2 import RovSensorEulerAngleData as ProtoEulerAngleData, RovSensorWaterTempDepthPressData as ProtoWaterTempDepthData

# Import original rovlink for decoding raw data
from rovlink import *

# Configuration
robot_ws_url = 'ws://192.168.137.5:8098'
# robot_ws_url = 'ws://127.0.0.1:8098'

# WSL TCP server
server_2_ip = '127.0.0.1'
server_2_port = 9001
server_2_address = (server_2_ip, server_2_port)

class ConnectionMonitor:
    """连接状态监控器"""
    def __init__(self):
        self.last_message_time = time.time()
        self.message_count = 0
        self.error_count = 0
        self.connection_start_time = time.time()
    
    def on_message_received(self):
        """记录消息接收"""
        self.last_message_time = time.time()
        self.message_count += 1
    
    def on_error(self):
        """记录错误"""
        self.error_count += 1
    
    def is_healthy(self, timeout=30):
        """检查连接是否健康"""
        return (time.time() - self.last_message_time) < timeout
    
    def get_stats(self):
        """获取连接统计信息"""
        uptime = time.time() - self.connection_start_time
        return {
            'uptime': uptime,
            'message_count': self.message_count,
            'error_count': self.error_count,
            'messages_per_second': self.message_count / uptime if uptime > 0 else 0,
            'last_message_age': time.time() - self.last_message_time
        }

class RovLinkProtoDataDecoder:
    def __init__(self):
        # Initialize original decoder for raw data processing
        self.raw_decoder = RovLinkDataDecoder()
        
        # 直接的传感器数据解码器实例（用于处理protobuf内的原始rovlink数据）
        self.bno055 = RovSensorEulerAngleData()
        self.m10 = RovSensorWaterTempDepthPressData()
        self.cab_sensor = RovSensorCabTempHumidPressData()  # 舱内传感器
        
        # Protobuf sensor data objects (保留用于其他可能的protobuf数据)
        self.euler_angle_data = ProtoEulerAngleData()
        self.water_temp_depth_data = ProtoWaterTempDepthData()
        
        # Flags to track if we have received data
        self.has_euler_data = False
        self.has_water_data = False
        self.has_cab_data = False
        
    def decode_protobuf_package(self, data):
        """Decode protobuf Package and extract rovlink data"""
        has_sensor_data = False
        
        try:
            package = Package()
            package.ParseFromString(data)
            
            if package.HasField('rovlink_frame'):
                rovlink_frame = package.rovlink_frame
                
                # 处理完整的10字节rovlink帧数据
                if rovlink_frame.raw_data and len(rovlink_frame.raw_data) == 10:
                    self.raw_decoder.decode(rovlink_frame.raw_data)
                    has_sensor_data = True
                
                # 处理基于帧类型的数据 - 这些传感器数据使用protobuf内存储的rovlink原始数据格式
                elif rovlink_frame.type == RovFrameType.SENSOR_EULER_ANGLE:
                    if rovlink_frame.raw_data and len(rovlink_frame.raw_data) >= 6:
                        # 提取6字节的传感器数据
                        sensor_data = rovlink_frame.raw_data[:6]
                        self.bno055.decode(bytearray(sensor_data))
                        self.has_euler_data = True
                        has_sensor_data = True
                
                elif rovlink_frame.type == RovFrameType.SENSOR_WATER_TEMP_DEPTH_PRESS:
                    if rovlink_frame.raw_data and len(rovlink_frame.raw_data) >= 6:
                        # 提取6字节的传感器数据
                        sensor_data = rovlink_frame.raw_data[:6]
                        self.m10.decode(bytearray(sensor_data))
                        self.has_water_data = True
                        has_sensor_data = True
                
                elif rovlink_frame.type == 1:  # SENSOR_CAB_TEMP_HUMID_PRESS
                    if rovlink_frame.raw_data and len(rovlink_frame.raw_data) >= 6:
                        sensor_data = rovlink_frame.raw_data[:6]
                        self.cab_sensor.decode(bytearray(sensor_data))
                        self.has_cab_data = True
                        has_sensor_data = True
                    
        except Exception as e:
            print(f"Error decoding protobuf package: {e}")
            import traceback
            traceback.print_exc()
        
        return has_sensor_data
    
    def get_data(self):
        """Return sensor data in the same format as original"""
        # 使用最新接收到的数据，优先使用直接解码的传感器数据
        # 因为protobuf中的数据是分别发送的，不是完整的10字节帧
        
        # 欧拉角数据 - 优先使用直接解码的数据
        if self.has_euler_data:
            roll = self.bno055.roll
            pitch = self.bno055.pitch  
            yaw = self.bno055.yaw
        else:
            # 回退到原始解码器数据
            roll = self.raw_decoder.bno055.roll
            pitch = self.raw_decoder.bno055.pitch
            yaw = self.raw_decoder.bno055.yaw
            
        # 水温深度数据 - 优先使用直接解码的数据
        if self.has_water_data:
            depth = self.m10.depth / 100
            temperature = self.m10.temperature / 100
        else:
            # 回退到原始解码器数据
            depth = self.raw_decoder.m10.depth / 100
            temperature = self.raw_decoder.m10.temperature / 100
            
        return roll, pitch, yaw, depth, temperature

async def health_check_loop(websocket, monitor):
    """健康检查循环"""
    try:
        while True:
            await asyncio.sleep(30)  # 每30秒检查一次
            
            if not monitor.is_healthy():
                stats = monitor.get_stats()
                print(f"⚠️  Connection unhealthy: last message {stats['last_message_age']:.1f}s ago")
                
                # 尝试发送ping来测试连接
                try:
                    pong_waiter = await websocket.ping()
                    await asyncio.wait_for(pong_waiter, timeout=5)
                    print("Ping successful, connection recovered")
                except asyncio.TimeoutError:
                    print("Ping timeout, connection may be dead")
                    break
                except Exception as e:
                    print(f"Ping failed: {e}")
                    break
            else:
                # 连接健康，可选择性地显示统计信息
                pass
                
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print(f"Health check error: {e}")

class RovLinkDataDecoder:
    """Original decoder for backward compatibility"""
    def __init__(self):
        self.rovlink_sensor_data = RovlinkFullFrame()
        self.rovlink_sensor_data.header = 0xfd
        self.rovlink_sensor_data.device = RovDeviceType.COMM_CAB
        self.rovlink_sensor_data.valid = False
        self.rovlink_sensor_data.sensor = True

        self.bno055 = RovSensorEulerAngleData()
        self.m10 = RovSensorWaterTempDepthPressData()

    def decode(self, data):
        if len(data) != 10:
            print(f"Received raw data length {len(data)} is not 10, skipping...")
            return
        
        self.rovlink_sensor_data.decode(data)
        if self.rovlink_sensor_data.opcode == RovlinkFrameType.SENSOR_EULER_ANGLE:
            self.bno055.decode(self.rovlink_sensor_data.payload)
            
        elif self.rovlink_sensor_data.opcode == RovlinkFrameType.SENSOR_WATER_TEMP_DEPTH_PRESS:
            self.m10.decode(self.rovlink_sensor_data.payload)

async def websocket_client():
    """强健的WebSocket客户端，具有自动重连和错误恢复功能"""
    data_decoder = RovLinkProtoDataDecoder()
    monitor = ConnectionMonitor()
    tcp_socket = None
    reconnect_delay = 1  # 初始重连延迟
    max_reconnect_delay = 30  # 最大重连延迟
    connection_count = 0
    
    while True:
        websocket = None
        try:
            connection_count += 1
            print(f"Attempting WebSocket connection #{connection_count}...")
            
            # 尝试连接TCP服务器
            if tcp_socket is None:
                try:
                    tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    tcp_socket.settimeout(5)  # 设置TCP连接超时
                    tcp_socket.connect(server_2_address)
                    print(f"Connected to TCP server at {server_2_ip}:{server_2_port}")
                except (ConnectionRefusedError, socket.timeout, OSError) as e:
                    print(f"TCP server not available: {e}, continuing without TCP forwarding...")
                    if tcp_socket:
                        tcp_socket.close()
                    tcp_socket = None
            
            # 创建WebSocket连接，使用更保守的参数
            websocket = await websockets.connect(
                robot_ws_url,
                ping_interval=10,  # 更频繁的ping
                ping_timeout=5,    # 更短的ping超时
                close_timeout=5,   # 更短的关闭超时
                max_size=2**20,    # 1MB最大消息大小
                read_limit=2**16,  # 64KB读取缓冲区
                write_limit=2**16, # 64KB写入缓冲区
                compression=None   # 禁用压缩以减少延迟
            )
            
            print(f"✓ WebSocket connected to {robot_ws_url}")
            reconnect_delay = 1  # 重置重连延迟
            monitor = ConnectionMonitor()  # 重置监控器
            
            # 启动健康检查任务
            health_check_task = asyncio.create_task(health_check_loop(websocket, monitor))
            
            try:
                # 消息处理循环
                async for message in websocket:
                    try:
                        monitor.on_message_received()
                        
                        # 每500条消息显示一次连接状态
                        if monitor.message_count % 500 == 0:
                            stats = monitor.get_stats()
                            print(f"Connection stats: {monitor.message_count} msgs, "
                                  f"{stats['messages_per_second']:.1f} msg/s, "
                                  f"uptime: {stats['uptime']:.1f}s")
                        
                        # 处理二进制protobuf数据
                        if isinstance(message, bytes):
                            # 解码protobuf消息，检查是否有传感器数据
                            has_sensor_data = data_decoder.decode_protobuf_package(message)
                            
                            # 只有在接收到传感器数据时才发送TCP（模拟原始客户端行为）
                            if has_sensor_data:
                                # 获取当前数据并格式化
                                rov_data_list = data_decoder.get_data()
                                send_data = ':'.join(str(item) for item in rov_data_list)
                                print(send_data)
                                
                                # 发送到TCP服务器（如果可用）
                                if tcp_socket:
                                    try:
                                        tcp_socket.send(send_data.encode('utf-8'))
                                    except (BrokenPipeError, ConnectionResetError, socket.error) as e:
                                        print(f"TCP connection lost: {e}")
                                        tcp_socket.close()
                                        tcp_socket = None
                                
                                # 匹配原始客户端的时间间隔
                                await asyncio.sleep(0.02)
                            
                        else:
                            print(f"Received text message: {message}")
                        
                    except Exception as e:
                        print(f"Error processing message: {e}")
                        monitor.on_error()
                        continue  # 继续处理下一条消息
            finally:
                # 取消健康检查任务
                health_check_task.cancel()
                try:
                    await health_check_task
                except asyncio.CancelledError:
                    pass
                        
        except websockets.exceptions.ConnectionClosed as e:
            print(f"WebSocket connection closed: {e}")
        except websockets.exceptions.InvalidURI:
            print(f"Invalid WebSocket URI: {robot_ws_url}")
            break
        except websockets.exceptions.InvalidHandshake as e:
            print(f"WebSocket handshake failed: {e}")
        except asyncio.TimeoutError:
            print("WebSocket connection timeout")
        except OSError as e:
            print(f"Network error: {e}")
        except Exception as e:
            print(f"Unexpected error in WebSocket client: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # 清理WebSocket连接
            if websocket:
                try:
                    await websocket.close()
                except:
                    pass
            
            # 清理TCP连接
            if tcp_socket:
                try:
                    tcp_socket.close()
                except:
                    pass
                tcp_socket = None
            
            # 指数退避重连策略
            print(f"Reconnecting in {reconnect_delay} seconds...")
            await asyncio.sleep(reconnect_delay)
            reconnect_delay = min(reconnect_delay * 1.5, max_reconnect_delay)

async def main():
    """主函数，处理优雅关闭"""
    # 设置信号处理器
    def signal_handler(signum, frame):
        print(f"\nReceived signal {signum}, shutting down gracefully...")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print("Starting WebSocket-based ROV data client...")
    print("Press Ctrl+C to stop")
    
    try:
        await websocket_client()
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Client stopped")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nProgram interrupted")