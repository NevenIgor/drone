#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple Visual Odometry + IMU Fusion для Raspberry Pi
IMU → Yaw (точнее), VO → X,Y (позиция)
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
        self.declare_parameter('scale', 0.001)
        self.declare_parameter('motion_threshold', 0.1)
        self.declare_parameter('use_imu', True)
        
        self.fps = self.get_parameter('camera_fps').value
        self.feature_count = self.get_parameter('feature_count').value
        self.scale = self.get_parameter('scale').value
        self.motion_threshold = self.get_parameter('motion_threshold').value
        self.use_imu = self.get_parameter('use_imu').value
        
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
        
        # Позиция (от VO)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        # Yaw (от IMU!)
        self.yaw = 0.0
        self.imu_yaw = 0.0
        self.prev_imu_time = None
        
        # IMU данные
        self.imu_data = None
        self.imu_moving = False
        self.prev_accel = None
        self.prev_gyro = None
        self.imu_timer = self.create_timer(0.05, self.check_imu_motion)
        
        # Home point
        self.home_set = False
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        
        # Буфер для сглаживания
        self.pos_buffer = deque(maxlen=20)
        
        # Статистика
        self.frame_count = 0
        self.skipped_frames = 0
        
        self.get_logger().info('✅ Simple VO + IMU (Yaw from IMU) запущен')
        self.get_logger().info(f'📍 Feature count: {self.feature_count}, Scale: {self.scale}')
        self.get_logger().info(f'🧭 Yaw: от IMU (гироскоп), Position: от VO (камера)')
    
    def image_callback(self, msg):
        self.frame_count += 1
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'❌ Ошибка конвертации: {e}')
            return
        
        # Детекция фич
        kp, desc = self.orb.detectAndCompute(frame, None)
        
        if self.prev_frame is None:
            self.prev_frame = frame
            self.prev_kp = kp
            self.prev_desc = desc
            self.get_logger().info('📷 Первый кадр получен')
            return
        
        if desc is None or self.prev_desc is None:
            self.prev_frame = frame
            self.prev_kp = kp
            self.prev_desc = desc
            return
        
        if len(kp) < 10 or len(self.prev_kp) < 10:
            self.skipped_frames += 1
            self.prev_frame = frame
            self.prev_kp = kp
            self.prev_desc = desc
            return
        
        # Проверка IMU
        if self.use_imu and not self.imu_moving:
            self.skipped_frames += 1
            self.prev_frame = frame
            self.prev_kp = kp
            self.prev_desc = desc
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'⏸ IMU не движется — пропущено: {self.skipped_frames}/{self.frame_count}')
            return
        
        # Matching
        matches = self.bf.match(self.prev_desc, desc)
        matches = sorted(matches, key=lambda x: x.distance)
        
        good_matches = matches[:max(10, len(matches) // 2)]
        
        if len(good_matches) < 8:
            self.skipped_frames += 1
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
            return
        
        inliers = np.sum(mask)
        
        if inliers < 6:
            return
        
        # Проверка качества
        condition_number = np.linalg.cond(H)
        if condition_number > 1000:
            return
        
        # Извлечение движения (только X,Y — yaw от IMU!)
        dx, dy, _ = self.extract_motion(H)
        
        # Интегрирование позиции
        self.x += dx
        self.y += dy
        
        # ✅ Yaw от IMU (вместо VO)
        if self.use_imu and self.imu_data is not None:
            self.yaw = self.imu_yaw
        else:
            self.get_logger().warn('⚠️ Нет IMU данных — используем VO yaw')
        
        # Нормализация
        self.yaw = np.mod(self.yaw + np.pi, 2 * np.pi) - np.pi
        
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
        
        # Лог каждые 30 кадров
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'📍 Pos: ({local_x:.2f}, {local_y:.2f}) yaw={np.degrees(avg_yaw):.1f}° | '
                f'Кадров: {self.frame_count}, Пропущено: {self.skipped_frames} | Inliers: {inliers}'
            )
    
    def imu_callback(self, msg):
        self.imu_data = msg
        
        # ✅ Интеграция yaw от гироскопа
        if self.prev_imu_time is not None:
            dt = (msg.header.stamp.sec - self.prev_imu_time.sec + 
                  (msg.header.stamp.nanosec - self.prev_imu_time.nanosec) / 1e9)
            
            yaw_rate = msg.angular_velocity.z
            self.imu_yaw += yaw_rate * dt
            
            # Нормализация
            self.imu_yaw = np.mod(self.imu_yaw + np.pi, 2 * np.pi) - np.pi
        
        self.prev_imu_time = msg.header.stamp
    
    def check_imu_motion(self):
        """Проверка движения по IMU"""
        if self.imu_data is None:
            self.imu_moving = True
            return
        
        accel = self.imu_data.linear_acceleration
        gyro = self.imu_data.angular_velocity
        
        accel_moving = False
        if self.prev_accel is not None:
            delta_accel = np.sqrt(
                (accel.x - self.prev_accel.x)**2 +
                (accel.y - self.prev_accel.y)**2 +
                (accel.z - self.prev_accel.z)**2
            )
            accel_moving = delta_accel > 0.05
        
        gyro_moving = False
        if self.prev_gyro is not None:
            delta_gyro = np.sqrt(
                (gyro.x - self.prev_gyro.x)**2 +
                (gyro.y - self.prev_gyro.y)**2 +
                (gyro.z - self.prev_gyro.z)**2
            )
            gyro_moving = delta_gyro > 0.01
        
        abs_rotation = np.sqrt(gyro.x**2 + gyro.y**2 + gyro.z**2)
        rotation_moving = abs_rotation > 0.02
        
        self.imu_moving = accel_moving or gyro_moving or rotation_moving
        
        self.prev_accel = accel
        self.prev_gyro = gyro
    
    def extract_motion(self, H):
        """Извлечение dx, dy (yaw теперь от IMU)"""
        dx = H[0, 2]
        dy = H[1, 2]
        
        # Порог движения
        if abs(dx) < self.motion_threshold and abs(dy) < self.motion_threshold:
            return 0.0, 0.0, 0.0
        
        # Масштаб
        dx = dx * self.scale
        dy = dy * self.scale
        
        # Ограничения
        dx = np.clip(dx, -0.15, 0.15)
        dy = np.clip(dy, -0.15, 0.15)
        
        return dx, dy, 0.0  # ✅ dtheta = 0 (yaw от IMU)
    
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
        msg.pose.covariance[35] = 0.01  # ✅ Меньше т.к. IMU точнее
        
        if self.imu_data is not None:
            msg.twist.twist.linear.x = self.imu_data.linear_acceleration.x * 0.1
            msg.twist.twist.linear.y = self.imu_data.linear_acceleration.y * 0.1
            msg.twist.twist.angular.z = self.imu_data.angular_velocity.z
        
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
