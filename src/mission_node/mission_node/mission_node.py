#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import math

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        
        # === Параметры ===
        self.declare_parameter('wp_radius', 2.0)  # Радиус достижения точки (метры)
        self.declare_parameter('rtl_altitude', 5.0)  # Высота возврата (метры)
        
        self.wp_radius = self.get_parameter('wp_radius').value
        self.rtl_altitude = self.get_parameter('rtl_altitude').value
        
        # === Подписки ===
        # Текущая позиция от OpenVINS
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Команды от оператора (веб-интерфейс или CLI)
        self.cmd_sub = self.create_subscription(
            String,
            '/mission/command',
            self.command_callback,
            10
        )
        
        # === Публикации ===
        # Целевая позиция для msp_bridge
        self.target_pub = self.create_publisher(
            PoseStamped,
            '/mission/target',
            10
        )
        
        # Режим для msp_bridge (MANUAL/AUTO/RTL)
        self.mode_pub = self.create_publisher(
            String,
            '/mission/mode',
            10
        )
        
        # Статус миссии (для оператора)
        self.status_pub = self.create_publisher(
            String,
            '/mission/status',
            10
        )
        
        # === Таймер ===
        # Проверка миссии 10 Гц
        self.timer = self.create_timer(0.1, self.mission_loop)
        
        # === Состояние ===
        self.mission = []  # Список waypoints: [{'x': 0, 'y': 0, 'z': -5}, ...]
        self.current_wp_index = 0
        self.mode = 'MANUAL'  # MANUAL, AUTO, RTL, LOITER
        self.current_pos = None
        self.home_pos = None  # Точка взлёта (0, 0, 0 в локальных координатах)
        
        self.get_logger().info('✅ Mission Node запущен')
        self.publish_status('READY')
    
    def odom_callback(self, msg):
        """Получение текущей позиции от OpenVINS"""
        self.current_pos = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
        
        # Запоминаем точку взлёта (первая позиция)
        if self.home_pos is None:
            self.home_pos = self.current_pos.copy()
            self.get_logger().info(f'🏠 Home установлен: {self.home_pos}')
    
    def command_callback(self, msg):
        """Обработка команд от оператора"""
        cmd = msg.data.strip().upper()
        
        if cmd == 'LOAD_MISSION':
            # Пример миссии (в реальном проекте загружается из файла или веб-интерфейса)
            self.mission = [
                {'x': 10.0, 'y': 0.0, 'z': -5.0},
                {'x': 10.0, 'y': 10.0, 'z': -5.0},
                {'x': 0.0, 'y': 10.0, 'z': -5.0},
                {'x': 0.0, 'y': 0.0, 'z': -5.0},
            ]
            self.current_wp_index = 0
            self.get_logger().info(f'📍 Миссия загружена: {len(self.mission)} точек')
            self.publish_status(f'MISSION_LOADED:{len(self.mission)}')
        
        elif cmd == 'START_AUTO':
            if self.mission:
                self.mode = 'AUTO'
                self.current_wp_index = 0
                self.publish_mode('AUTO')
                self.get_logger().info('🚀 AUTO режим запущен')
                self.publish_status('AUTO_STARTED')
            else:
                self.get_logger().warn('❌ Миссия не загружена!')
                self.publish_status('ERROR:NO_MISSION')
        
        elif cmd == 'SWITCH_MANUAL':
            self.mode = 'MANUAL'
            self.publish_mode('MANUAL')
            self.get_logger().info('👐 MANUAL режим')
            self.publish_status('MANUAL')
        
        elif cmd == 'RTL':
            self.mode = 'RTL'
            self.publish_mode('AUTO')  # msp_bridge в AUTO для RTL
            self.get_logger().info('🏠 Возврат домой (RTL)')
            self.publish_status('RTL')
        
        elif cmd == 'LOITER':
            self.mode = 'LOITER'
            self.publish_mode('MANUAL')  # Удерживаем текущую позицию
            self.get_logger().info('⏸ LOITER (удержание позиции)')
            self.publish_status('LOITER')
        
        elif cmd.startswith('GOTO:'):
            # Прямая команда к точке: GOTO:x,y,z
            try:
                parts = cmd[5:].split(',')
                target = {
                    'x': float(parts[0]),
                    'y': float(parts[1]),
                    'z': float(parts[2]) if len(parts) > 2 else -5.0
                }
                self.mission = [target]
                self.current_wp_index = 0
                self.mode = 'AUTO'
                self.publish_mode('AUTO')
                self.get_logger().info(f'🎯 GOTO: {target}')
                self.publish_status(f'GOTO:{target["x"]},{target["y"]},{target["z"]}')
            except Exception as e:
                self.get_logger().error(f'❌ Ошибка GOTO: {e}')
                self.publish_status(f'ERROR:GOTO:{e}')
        
        else:
            self.get_logger().warn(f'❓ Неизвестная команда: {cmd}')
    
    def mission_loop(self):
        """Основной цикл миссии (10 Гц)"""
        if self.mode == 'MANUAL':
            return  # В ручном режиме не управляем
        
        if self.current_pos is None:
            return  # Нет данных о позиции
        
        if self.mode == 'RTL':
            # Возврат домой
            target = self.home_pos.copy() if self.home_pos else {'x': 0, 'y': 0, 'z': -5.0}
            target['z'] = -self.rtl_altitude  # Высота RTL
            
            self.publish_target(target)
            
            # Проверка достижения дома
            if self.distance(self.current_pos, target) < self.wp_radius:
                self.get_logger().info('🏠 Дом достигнут! Переключение в LOITER')
                self.mode = 'LOITER'
                self.publish_status('RTL_COMPLETE')
        
        elif self.mode == 'AUTO' and self.mission:
            # Получаем текущую целевую точку
            if self.current_wp_index < len(self.mission):
                target = self.mission[self.current_wp_index]
                self.publish_target(target)
                
                # Проверка достижения точки
                dist = self.distance(self.current_pos, target)
                
                if dist <= self.wp_radius:
                    self.get_logger().info(f'✅ WP {self.current_wp_index + 1} достигнут (расстояние: {dist:.2f}м)')
                    self.current_wp_index += 1
                    
                    # Проверка завершения миссии
                    if self.current_wp_index >= len(self.mission):
                        self.get_logger().info('🎉 Миссия завершена! Возврат домой')
                        self.publish_status('MISSION_COMPLETE')
                        self.mode = 'RTL'
                    else:
                        self.publish_status(f'WP:{self.current_wp_index + 1}/{len(self.mission)}')
                else:
                    self.publish_status(f'AUTO:WP{self.current_wp_index + 1},dist={dist:.1f}m')
    
    def publish_target(self, target):
        """Публикация целевой позиции для msp_bridge"""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = target['x']
        msg.pose.position.y = target['y']
        msg.pose.position.z = target['z']
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)
    
    def publish_mode(self, mode):
        """Публикация режима для msp_bridge"""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
    
    def publish_status(self, status):
        """Публикация статуса для оператора"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def distance(self, p1, p2):
        """Расчёт расстояния между двумя точками (2D)"""
        return math.sqrt(
            (p1['x'] - p2['x'])**2 + 
            (p1['y'] - p2['y'])**2
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
