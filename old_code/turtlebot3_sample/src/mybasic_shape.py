#!/usr/bin/env python
import rospy
import time  # 主要是调用了sleep函数
from visualization_msgs.msg import Marker  # 可以通过rosmsg show visualization_msgs/Marker查看marker的内容
#rate = rospy.Rate(0.1)  # 消息發布的刷新頻率
pub = rospy.Publisher("visualization_marker", Marker, queue_size=1) # topic名稱為visualization_marker(固定的) 消息類型為Marker
shape = Marker.CYLINDER
marker = Marker()
marker.header.frame_id = "my_frame"
marker.header.stamp = rospy.Time.now()
marker.ns = "basic_shapes"
marker.id = 0
marker.type = shape
marker.action = Marker.ADD
# 初始化marker的位置和角度
marker.pose.position.x = 0
marker.pose.position.y = 0
marker.pose.position.z = 0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0
# marker的尺寸大小 1.0對應現實的1m
marker.scale.x = 1.0
marker.scale.y = 1.0
marker.scale.z = 1.0
# marker的顏色 a為透明度0.0無法看到
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.color.a = 1.0
marker.lifetime = rospy.Duration()
while pub.get_num_connections() < 1:
# get_num_connections()是python版本查閱當前topic的訂閱器數目， cpp版本对应函数marker_pub.getNumScribers()
# 用来判斷當前是否有訂閱器，也就是說rviz是否打開，如果沒有訂閱器沒必要發布消息。當然没有訂閱器情况下發布消息也是沒問題的。
if rospy.is_shutdown():
exit(1)
rospy.loginfo("Please create a subscriber to the marker!")
time.sleep(1)  # 休眠等待，不斷查詢有沒訂閱器
pub.publish(marker)  # 發布消息


#rate.sleep()  # 指定頻率刷新
