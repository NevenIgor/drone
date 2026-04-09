#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple Visual Odometry для Raspberry Pi + USB камера + IMU
Использует ORB features + BFMatcher + RANSAC
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from collections import deque

class SimpleVONode(Node):
    def __init__(self):
        super().__init__('simple_vo_node')
        
        # === Параметры ===
        self.declare_parameter('camera_fps', 30.0)
        self.declare_parameter('feature_count', 200)
        self.declare_parameter('min_distance', 30)
        self.declare_parameter('scale', 1.0)
        
        self.fps = self.get_parameter('camera_fps').value
        self.feature_count = self.get_parameter('feature_count').value
        self.min_distance = self.get_parameter('min_distance').value
        self.scale = self.get_parameter('scale').value
        
        # === Подписки ===
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        # === Публикации ===
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # === Состояние VO ===
        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(nfeatures=self.feature_count)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        self.prev_frame = None
        self.prev_kp = None
        self.prev_desc = None
        
        # Позиция
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.yaw = 0.0
        
        # Home point
        self.home_set = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        
        # Буфер для сглаживания
        self.pos_buffer = deque(maxlen=10)
        
        self.get_logger().info('✅ Simple VO запущен')
        self.get_logger().info(f'📍 Feature count: {self.feature_count}, Scale: {self.scale}')
    
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Ошибка конвертации: {e}')
            return
        
        # Детекция фич
        kp, desc = self.orb.detectAndCompute(frame, None)
        
        # ОТЛАДКА: выводим количество фич
        if desc is not None:
            self.get_logger().debug(f'🔍 Фич: {len(kp)}, дескрипторов: {desc.shape[0]}')
        
        if self.prev_frame is None:
            self.prev_frame = frame
            self.prev_kp = kp
            self.prev_desc = desc
            self.get_logger().info('📷 Первый кадр получен')
            return
        
        if desc is None or self.prev_desc is None:
            self.get_logger().warn('⚠️ Нет дескрипторов!')
            self.prev_frame = frame
            self.prev_kp = kp
            self.prev_desc = desc
            return
        
        if len(kp) < 10 or len(self.prev_kp) < 10:
            self.get_logger().warn(f'⚠️ Мало фич: curr={len(kp)}, prev={len(self.prev_kp)}')
            self.prev_frame = frame
            self.prev_kp = kp
            self.prev_desc = desc
            return
        
        # Matching
        matches = self.bf.match(self.prev_desc, desc)
        matches = sorted(matches, key=lambda x: x.distance)
        
        # ОТЛАДКА: выводим количество матчей
        self.get_logger().debug(f'🔗 Матчей: {len(matches)}')
        
        good_matches = matches[:max(10, len(matches) // 2)]
        
        if len(good_matches) < 8:
            self.get_logger().warn(f'⚠️ Мало хороших матчей: {len(good_matches)}')
            self.prev_frame = frame
            self.prev_kp = kp
            self.prev_desc = desc
            return
        
        # Извлечение координат
        prev_pts = np.array([self.prev_kp[m.queryIdx].pt for m in good_matches], dtype=np.float32)
        curr_pts = np.array([kp[m.trainIdx].pt for m in good_matches], dtype=np.float32)
        
        # RANSAC
        H, mask = cv2.findHomography(prev_pts, curr_pts, cv2.RANSAC, 5.0)
        
        if H is None or mask is None:
            self.get_logger().warn('⚠️ Гомография не найдена!')
            return
        
        inliers = np.sum(mask)
        self.get_logger().debug(f'✅ Инлайеры: {inliers}')
        
        if inliers < 6:
            self.get_logger().warn(f'⚠️ Мало инлайеров: {inliers}')
            return
        
        # Извлечение движения
        dx, dy, dtheta = self.extract_motion(H)
        
        # Интегрирование
        self.x += dx * self.scale
        self.y += dy * self.scale
        self.yaw += dtheta
        
        # Сглаживание
        self.pos_buffer.append((self.x, self.y, self.yaw))
        avg_x = sum(p[0] for p in self.pos_buffer) / len(self.pos_buffer)
        avg_y = sum(p[1] for p in self.pos_buffer) / len(self.pos_buffer)
        avg_yaw = sum(p[2] for p in self.pos_buffer) / len(self.pos_buffer)
        
        # Home point
        if not self.home_set:
            self.home_x = avg_x
            self.home_y = avg_y
            self.home_z = self.z
            self.home_set = True
            self.get_logger().info(f'🏠 Home установлен: ({self.home_x:.2f}, {self.home_y:.2f})')
        
        # Локальные координаты
        local_x = avg_x - self.home_x
        local_y = avg_y - self.home_y
        local_z = self.z
        
        # Публикация
        self.publish_odom(local_x, local_y, local_z, avg_yaw, msg.header.stamp)
        
        # Обновление кадра
        self.prev_frame = frame
        self.prev_kp = kp
        self.prev_desc = desc
        
        # Лог каждый кадр
        self.get_logger().info(f'📍 Pos: ({local_x:.2f}, {local_y:.2f}) yaw={np.degrees(avg_yaw):.1f}° inliers={inliers}')
    
    def imu_callback(self, msg):
        # Пока просто сохраняем данные (можно добавить fusion позже)
        pass
    
    def extract_motion(self, H):
        """Извлечение dx, dy, dtheta из гомографии"""
        dx = H[0, 2]
        dy = H[1, 2]
        dtheta = np.arctan2(H[1, 0], H[0, 0])
        return dx, dy, dtheta
    
    def publish_odom(self, x, y, z, yaw, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        
        msg.pose.pose.orientation.z = np.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = np.cos(yaw / 2.0)
        
        msg.pose.covariance[0] = 0.05
        msg.pose.covariance[7] = 0.05
        msg.pose.covariance[14] = 0.1
        msg.pose.covariance[35] = 0.01
        
        self.odom_pub.publish(msg)
        self.publish_tf(x, y, z, yaw, stamp)
    
    def publish_tf(self, x, y, z, yaw, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.z = np.sin(yaw / 2.0)
        t.transform.rotation.w = np.cos(yaw / 2.0)
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleVONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
