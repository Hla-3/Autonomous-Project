#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool  # أضف Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class LanePerception(Node):
    def __init__(self):
        super().__init__('lane_perception_node')

        self.bridge = CvBridge()

        # مشترك في الكاميرا
        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        # ناشرين - أضف واحد لنهاية المسار
        self.error_pub = self.create_publisher(Float64, '/lane_error', 10)
        self.end_pub = self.create_publisher(Bool, '/lane_end_detected', 10)  # جديد

        # متغيرات لاكتشاف نهاية المسار
        self.consecutive_no_lane_frames = 0
        self.MAX_NO_LANE_FRAMES = 15  # إذا 15 إطار متتالي بدون مسار
        
        # متغيرات التحكم
        self.last_error = 0.0
        self.lane_detected = True

    def image_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w, _ = img.shape

            # 1. معالجة الصورة
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5,5), 0)
            edges = cv2.Canny(blur, 50, 150)

            # 2. تحديد منطقة الاهتمام (ROI)
            mask = np.zeros_like(edges)
            polygon = np.array([[
                (0, h),
                (w, h),
                (w, int(h*0.6)),
                (0, int(h*0.6))
            ]])
            cv2.fillPoly(mask, polygon, 255)
            masked = cv2.bitwise_and(edges, mask)

            # 3. اكتشاف الخطوط
            lines = cv2.HoughLinesP(masked, 1, np.pi/180, 50,
                                    minLineLength=50, maxLineGap=100)

            left_x, right_x = [], []

            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    slope = (y2 - y1) / (x2 - x1 + 1e-6)
                    if slope < -0.5:  # خط يسار
                        left_x.extend([x1, x2])
                    elif slope > 0.5:  # خط يمين
                        right_x.extend([x1, x2])

            # 4. حساب الخطأ وفحص وجود المسار
            if left_x and right_x:
                # المسار موجود
                self.consecutive_no_lane_frames = 0  # إعادة التعيين
                self.lane_detected = True
                
                lane_center = (np.mean(left_x) + np.mean(right_x)) / 2.0
                image_center = w / 2.0
                error = image_center - lane_center
                self.last_error = error
                
                # نشر خطأ المسار
                self.error_pub.publish(Float64(data=float(error)))
                
                # نشر إن المسار لسه موجود
                self.end_pub.publish(Bool(data=False))
                
                # رسالة للتصحيح
                self.get_logger().info(
                    f"Lane detected. Error: {error:.2f}, "
                    f"Left: {len(left_x)//2}, Right: {len(right_x)//2} lines"
                )
                
                # رسم الخطوط على الصورة (اختياري للتصحيح)
                self.draw_lanes(img, left_x, right_x, lines)
                
            else:
                # لا توجد خطوط مسار
                self.consecutive_no_lane_frames += 1
                
                # إذا عدد كبير من الإطارات بدون مسارات
                if self.consecutive_no_lane_frames >= self.MAX_NO_LANE_FRAMES:
                    self.lane_detected = False
                    
                    # نشر إن المسار انتهى
                    self.end_pub.publish(Bool(data=True))
                    
                    # نشر خطأ صفر (أو آخر خطأ معروف)
                    self.error_pub.publish(Float64(data=0.0))
                    
                    self.get_logger().warn(
                        f"LANE END DETECTED! "
                        f"No lanes for {self.consecutive_no_lane_frames} frames"
                    )
                else:
                    # استخدم آخر خطأ معروف
                    self.error_pub.publish(Float64(data=float(self.last_error)))
                    self.end_pub.publish(Bool(data=False))
                    
                    self.get_logger().info(
                        f"Searching for lanes... "
                        f"Frame {self.consecutive_no_lane_frames}/{self.MAX_NO_LANE_FRAMES}"
                    )

            # 5. عرض الصورة للتصحيح
            cv2.putText(img, f"Lanes: {self.lane_detected}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(img, f"Error: {self.last_error:.1f}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Lane Detection", img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in lane detection: {str(e)}")

    def draw_lanes(self, img, left_x, right_x, lines):
        """رسم الخطوط على الصورة للتصحيح"""
        h, w, _ = img.shape
        
        # رسم منطقة الاهتمام
        polygon = np.array([[
            (0, h),
            (w, h),
            (w, int(h*0.6)),
            (0, int(h*0.6))
        ]])
        cv2.polylines(img, [polygon], True, (0, 255, 255), 1)
        
        # رسم الخطوط المكتشفة
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
        # رسم متوسط مركز المسار
        if left_x and right_x:
            lane_center = int((np.mean(left_x) + np.mean(right_x)) / 2.0)
            image_center = w // 2
            
            # خط مركز المسار
            cv2.line(img, (lane_center, h), (lane_center, int(h*0.6)), 
                    (255, 0, 0), 2)
            
            # خط مركز الصورة
            cv2.line(img, (image_center, h), (image_center, int(h*0.6)), 
                    (0, 255, 0), 2)
            
            # رسم المسافة بينهما
            cv2.putText(img, f"Diff: {abs(lane_center - image_center)}", 
                       (w//2 - 50, h//2), cv2.FONT_HERSHEY_SIMPLEX, 
                       0.7, (255, 255, 0), 2)

def main():
    rclpy.init()
    node = LanePerception()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
