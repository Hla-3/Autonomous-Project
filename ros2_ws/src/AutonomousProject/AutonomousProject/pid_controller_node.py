#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
import time

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller_node')

        # مشتركين - أضف لنهاية المسار
        self.sub_lane = self.create_subscription(
            Float64, '/lane_error', self.lane_callback, 10)
        self.sub_lane_end = self.create_subscription(  # جديد
            Bool, '/lane_end_detected', self.lane_end_callback, 10)
        
        # يمكنك إضافة obstacle إذا عندك
        # self.sub_obs = self.create_subscription(
        #     Bool, '/obstacle_detected', self.obs_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # معاملات PID
        self.kp = 0.005
        self.ki = 0.0
        self.kd = 0.001

        # متغيرات التحكم
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

        # حالات النظام
        self.lane_end_detected = False
        self.robot_stopped = False
        self.current_error = 0.0
        
        # مؤقت للتحكم
        self.control_timer = self.create_timer(0.05, self.control_loop)

    def lane_end_callback(self, msg):
        """استقبال حالة نهاية المسار"""
        if msg.data and not self.lane_end_detected:
            self.lane_end_detected = True
            self.get_logger().warn("LANE END DETECTED! Will stop soon...")

    def lane_callback(self, msg):
        self.current_error = msg.data

    def control_loop(self):
        """حلقة التحكم الرئيسية"""
        cmd = Twist()
        
        if self.robot_stopped:
            # الروبوت متوقف بالفعل
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return
        
        if self.lane_end_detected and not self.robot_stopped:
            # توقف تدريجي عند نهاية المسار
            # حلقة توقف ذكية
            
            # الخطوة 1: تقليل السرعة تدريجياً
            current_speed = getattr(self, 'current_speed', 0.2)
            new_speed = max(0.0, current_speed - 0.05)
            
            cmd.linear.x = new_speed
            cmd.angular.z = 0.0  # لا دوران أثناء التوقف
            
            self.current_speed = new_speed
            
            # إذا السرعة قريبة من الصفر، أوقف نهائي
            if new_speed <= 0.05:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.robot_stopped = True
                self.get_logger().info("ROBOT COMPLETELY STOPPED")
            
            self.get_logger().info(f"Stopping... Speed: {cmd.linear.x:.2f}")
        
        else:
            # التحكم العادي في المسار
            now = time.time()
            dt = now - self.prev_time if now - self.prev_time > 0 else 0.01

            error = self.current_error
            
            # حساب PID
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
            control = self.kp * error + self.ki * self.integral + self.kd * derivative

            cmd.linear.x = 0.2
            cmd.angular.z = control

            self.prev_error = error
            self.prev_time = now
            
            # تسجيل
            self.get_logger().info(f"Error: {error:.2f}, Control: {control:.3f}")

        # نشر الأمر
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = PIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
