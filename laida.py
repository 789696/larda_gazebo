#!/usr/bin/env python3
# coding=utf-8

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarBehavior:
    def __init__(self):
        rospy.init_node("lidar_behavior")
        
        # 配置参数
        self.stop_distance = 1.0   # 停止距离阈值（米）
        self.region_width = 0.6    # 检测区域宽度（米）
        self.forward_speed = 0.2  # 前进速度
        
        # 状态变量
        self.is_stopped = False
        self.obstacle_scan = False  # scan雷达是否检测到障碍物
        self.obstacle_scan2 = False # scan2雷达是否检测到障碍物

        # 订阅两个雷达话题
        self.lidar_sub = rospy.Subscriber("scan", LaserScan, self.lidar_callback)
        self.lidar_sub2 = rospy.Subscriber("scan2", LaserScan, self.lidar_callback2)
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    def _get_min_distance_in_region(self, msg):
        """
        计算前方指定宽度区域内的最小障碍物距离
        返回：区域内最小有效距离，若无有效数据则返回无穷大
        """
        # 计算检测区域对应的角度范围（基于宽度的半角）
        # 当障碍物在stop_distance处时，刚好能检测到0.6米宽的区域
        half_angle = math.atan2(self.region_width/2, self.stop_distance)
        
        min_distance = float('inf')
        for i, distance in enumerate(msg.ranges):
            # 计算当前点的角度
            angle = msg.angle_min + i * msg.angle_increment
            
            # 过滤角度范围外的点
            if abs(angle) > half_angle:
                continue
            
            # 过滤无效数据（超出雷达量程或异常值）
            if distance < msg.range_min or distance > msg.range_max:
                continue
            
            # 更新最小距离
            if distance < min_distance:
                min_distance = distance
        
        return min_distance

    def lidar_callback(self, msg):
        """处理scan雷达数据"""
        min_dist = self._get_min_distance_in_region(msg)
        self.obstacle_scan = (min_dist <= self.stop_distance)
        self._control_movement()

    def lidar_callback2(self, msg):
        """处理scan2雷达数据"""
        min_dist = self._get_min_distance_in_region(msg)
        self.obstacle_scan2 = (min_dist <= self.stop_distance)
        self._control_movement()

    def _stop_robot(self):
        """停止机器人运动"""
        if not self.is_stopped:
            vel_cmd = Twist()
            vel_cmd.linear.x = 0.0
            vel_cmd.angular.z = 0.0
            self.vel_pub.publish(vel_cmd)
            self.is_stopped = True
            rospy.loginfo("检测到障碍物！距离小于%.2f米 → 停止运动", self.stop_distance)

    def _move_forward(self):
        """控制机器人前进"""
        if self.is_stopped:
            rospy.loginfo("前方无障碍物 → 恢复移动")
            self.is_stopped = False
        
        vel_cmd = Twist()
        vel_cmd.linear.x = self.forward_speed
        vel_cmd.angular.z = 0.0
        self.vel_pub.publish(vel_cmd)

    def _control_movement(self):
        """综合两个雷达的检测结果，决定运动状态"""
        # 只要有一个雷达检测到障碍物就停止
        if self.obstacle_scan or self.obstacle_scan2:
            self._stop_robot()
        else:
            self._move_forward()

if __name__ == "__main__":
    try:
        node = LidarBehavior()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# #!/usr/bin/env python3
# # coding=utf-8

# import rospy
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import Twist

# class LidarBehavior:
#     def __init__(self):
#         rospy.init_node("lidar_behavior")
        
#         # 状态变量
#         self.is_stopped = False
#         self.safe_distance = 1.1   # 恢复移动的安全距离
#         self.stop_distance = 1.0   # 停止阈值

#         # 订阅与发布
#         self.lidar_sub = rospy.Subscriber("scan", LaserScan, self.lidar_callback)
#         self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

#     def lidar_callback(self, msg):
#         # 获取正前方测距值（动态计算中点）
#         front_index = len(msg.ranges) // 2
#         raw_distance = msg.ranges[front_index]
        
#         # 状态判断
#         if raw_distance < self.stop_distance:
#             self.is_stopped = True
#             self._stop_robot()
#         elif raw_distance > self.safe_distance and self.is_stopped:
#             self.is_stopped = False
#             self._resume_movement()

#         # 执行控制逻辑
#         self._control_movement()

#     def _stop_robot(self):
#         """停止机器人"""
#         vel_cmd = Twist()
#         vel_cmd.linear.x = 0.0
#         vel_cmd.angular.z = 0.0
#         self.vel_pub.publish(vel_cmd)
#         rospy.loginfo("检测到障碍物！距离：%.2f m → 停止运动", self.stop_distance)

#     def _resume_movement(self):
#         """恢复移动"""
#         rospy.loginfo("障碍物已清除 → 恢复移动")
#         self._move_forward()

#     def _move_forward(self):
#         """直行控制"""
#         vel_cmd = Twist()
#         vel_cmd.linear.x = 0.05
#         vel_cmd.angular.z = 0.0
#         self.vel_pub.publish(vel_cmd)

#     def _control_movement(self):
#         """主控制逻辑"""
#         if not self.is_stopped:
#             self._move_forward()

# if __name__ == "__main__":
#     try:
#         node = LidarBehavior()
#         rospy.spin()
#     except rospy.ROSInterruptException:
#         pass