#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
import time  # ✅ Добавили стандартный time

class OdomEmulator(Node):
    def __init__(self):
        super().__init__('odom_emulator')
        
        # Параметры эмуляции
        self.declare_parameter('speed', 1.0)  # м/с
        self.declare_parameter('wp_radius', 2.0)  # м
        
        self.speed = self.get_parameter('speed').value
        self.wp_radius = self.get_parameter('wp_radius').value
        
        # Публикация одометрии
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Таймер 50 Гц (как настоящий OpenVINS)
        self.timer = self.create_timer(0.02, self.update)
        
        # Состояние
        self.x = 0.0
        self.y = 0.0
        self.z = -5.0  # Высота 5м
        self.yaw = 0.0
        
        # Целевые точки (как в mission_node)
        self.waypoints = [
            {'x': 10.0, 'y': 0.0, 'z': -5.0},
            {'x': 10.0, 'y': 10.0, 'z': -5.0},
            {'x': 0.0, 'y': 10.0, 'z': -5.0},
            {'x': 0.0, 'y': 0.0, 'z': -5.0},
        ]
        self.current_wp = 0
        self.mode = 'WAIT'  # WAIT, MOVING, COMPLETE
        self.start_delayed = False
        
        self.get_logger().info('🎮 Odom Emulator запущен')
        self.get_logger().info(f'📍 Скорость: {self.speed} м/с, Waypoints: {len(self.waypoints)}')
        self.get_logger().info('⏳ Старт через 3 секунды...')
    
    def update(self):
        # Задержка старта на 3 секунды
        if not self.start_delayed:
            time.sleep(3.0)  # ✅ Используем time.sleep()
            self.start_delayed = True
            self.mode = 'MOVING'
            self.current_wp = 0
            self.get_logger().info('🚀 Эмуляция движения запущена!')
            return
        
        if self.mode == 'WAIT':
            return
        
        if self.mode == 'MOVING' and self.current_wp < len(self.waypoints):
            target = self.waypoints[self.current_wp]
            
            # Вектор к цели
            dx = target['x'] - self.x
            dy = target['y'] - self.y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist <= self.wp_radius:
                # Достигли точки
                self.current_wp += 1
                self.get_logger().info(f'✅ WP {self.current_wp} достигнут!')
                
                if self.current_wp >= len(self.waypoints):
                    self.mode = 'COMPLETE'
                    self.get_logger().info('🎉 Миссия завершена!')
                return
            
            # Нормализуем и двигаемся
            vx = (dx / dist) * self.speed
            vy = (dy / dist) * self.speed
            
            # Обновляем позицию (50 Гц → dt=0.02)
            dt = 0.02
            self.x += vx * dt
            self.y += vy * dt
            
            # Вычисляем yaw (направление движения)
            self.yaw = math.atan2(vy, vx)
            
            # Публикуем одометрию
            self.publish_odom()
        
        elif self.mode == 'COMPLETE':
            # Удерживаем последнюю позицию
            self.publish_odom()
    
    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z
        
        # Кватернион из yaw
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        msg.pose.covariance[0] = 0.01  # x
        msg.pose.covariance[7] = 0.01  # y
        msg.pose.covariance[14] = 0.01  # z
        
        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomEmulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
