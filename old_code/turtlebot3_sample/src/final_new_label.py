from visualization_msgs.msg import Marker
import time
import sys
import threading
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan
import os
import math
import numpy as np
import copy
import matplotlib
f = open('ada_demo.txt')
train_data = np.loadtxt(f)
classified_num = np.size(train_data, 1)
f.close
class Segment_xydata:
def __init__(self, xy, num):
self.xydata = xy
self.num_xy = num
self.T1 = num
self.T2 = np.sqrt(np.sum(np.power(np.std(xy, axis=0), 2)))
self.T3 = np.linalg.norm(xy[0, :]-xy[num-1, :])
self.T4 = circularity(xy, num, 0)
self.T5 = linearity(xy, num)
def get_xydata(self):
    return self.xydata.copy()
def display(Seg, S_n):
global tracking_left
global tracking_right
global start_tracking_left
global start_tracking_right
global retrack_left
global retrack_right
global pre_X_left
global pre_X_right
global Pre_P_left
global Pre_P_right
global P_left
global X_left
global P_right
global X_right
global t
t += 1
Z = np.zeros(S_n)
for i in range(S_n):
if isbox(Seg[i].T1, Seg[i].T2, Seg[i].T3, Seg[i].T4, Seg[i].T5) > 0:
Z[i] = 1
SM = Bayesfilter(Z, S_n)
foot_Seg = np.where(Z > 0.2)
foot_num = np.size(foot_Seg, 1)
foot_pos = np.zeros((foot_num, 2))
initial_foot_pos_left = np.zeros((2, 2))
initial_foot_pos_right = np.zeros((2, 2))
initial_foot_count_left = 0
initial_foot_count_right = 0
dis_from_leg_left = np.zeros((foot_num, 1))
dis_from_leg_right = np.zeros((foot_num, 1))
for i in range(0, foot_num):
foot_pos[i, :] = np.mean(Seg[foot_Seg[0][i]].xydata, axis=0)
# print(foot_pos)
# 繼續tracking與否
if (np.sum(tracking_left) < 16):
    start_tracking_left = 0
    retrack_left = 0
    tracking_left = np.ones((1, 24))

if (np.sum(tracking_right) < 16):
    start_tracking_right = 0
    retrack_right = 0
    tracking_right = np.ones((1, 24))

# 左邊初始設定
if (start_tracking_left == 0):
    for i in range(0, foot_num):
        if (foot_pos[i][0] < 0):
            initial_foot_count_left += 1
    initial_foot_pos_left = np.zeros((initial_foot_count_left, 2))
    initial_foot_count_left = 0
    for i in range(0, foot_num):
        if (foot_pos[i][0] < 0):
            initial_foot_count_left += 1
            initial_foot_pos_left[initial_foot_count_left-1,
                                  :] = foot_pos[i, :]
    mean_foot_pos_left = np.mean(initial_foot_pos_left, axis=0)
    # mean_foot_pos_left = [x, y]
else:
    for i in range(0, foot_num):  # 求mahalanobis distance

        foot_pos_transpose = np.transpose(foot_pos)[:, i:i+1]
        # (foot_pos(i,:)'-H*pre_X_left)
        Dm_up_left = foot_pos_transpose-np.matmul(H, pre_X_left)
        Dm_down_left = np.matmul(np.matmul(H, Pre_P_left),
                                 np.transpose(H)) + Q  # (H*Pre_P_left*H' + Q)
        dis_from_leg_left[i] = np.matmul(
            np.matmul(np.transpose(Dm_up_left), np.linalg.inv(Dm_down_left)), Dm_up_left)  # (foot_pos(i,:)'-H*pre_X_left)'/(H*Pre_P_left*H' + Q)*(foot_pos(i,:)'-H*pre_X_left)
    if (dis_from_leg_left.size == 0):
        mean_foot_pos_left = np.array([0, 0])
    else:
        if (dis_from_leg_left.min() < 200):
            idx = np.unravel_index(
                np.argmin(dis_from_leg_left, axis=None), dis_from_leg_left.shape)
            # print(idx)
            mean_foot_pos_left = foot_pos[idx[0]]
        else:
            mean_foot_pos_left = np.array([0, 0])

# 右邊初始設定
if (start_tracking_right == 0):
    for i in range(0, foot_num):
        if (foot_pos[i][0] > 0):
            initial_foot_count_right += 1
    initial_foot_pos_right = np.zeros((initial_foot_count_right, 2))
    initial_foot_count_right = 0
    for i in range(0, foot_num):
        if (foot_pos[i][0] > 0):
            initial_foot_count_right += 1
            initial_foot_pos_right[initial_foot_count_right-1,
                                   :] = foot_pos[i, :]
    mean_foot_pos_right = np.mean(initial_foot_pos_right, axis=0)
    # mean_foot_pos_right = [x, y]
else:
    for i in range(0, foot_num):  # 求mahalanobis distance
        foot_pos_transpose = np.transpose(foot_pos)[:, i:i+1]
        # (foot_pos(i,:)'-H*pre_X_right)
        Dm_up_right = foot_pos_transpose-np.matmul(H, pre_X_right)
        Dm_down_right = np.matmul(np.matmul(H, Pre_P_right),
                                  np.transpose(H)) + Q  # (H*Pre_P_right*H' + Q)
        dis_from_leg_right[i] = np.matmul(
            np.matmul(np.transpose(Dm_up_right), np.linalg.inv(Dm_down_right)), Dm_up_right)  # (foot_pos(i,:)'-H*pre_X_right)'/(H*Pre_P_right*H' + Q)*(foot_pos(i,:)'-H*pre_X_right)
    if (dis_from_leg_right.size == 0):
        mean_foot_pos_right = np.array([0, 0])
    else:
        if (dis_from_leg_right.min() < 200):
            idx = np.unravel_index(
                np.argmin(dis_from_leg_right, axis=None), dis_from_leg_right.shape)
            # print(idx)
            mean_foot_pos_right = foot_pos[idx[0]]
        else:
            mean_foot_pos_right = np.array([0, 0])
# 左tracking
if (start_tracking_left == 0):
    x_left = mean_foot_pos_left[0]
    vx = 0
    y = mean_foot_pos_left[1]
    vy = 0
    X_left = np.array([[x_left], [vx], [y], [vy]])
else:
    X_left = np.matmul(T1, X_left) + np.matmul(T2, np.array([[1], [1]]))
    P_left = np.matmul(np.matmul(T1, P_left), np.transpose(
        T1)) + np.matmul(np.matmul(T2, R), np.transpose(T2))
predict_pos_left = np.matmul(H, X_left)
K_left_denominator = np.matmul(np.matmul(H, P_left),
                               np.transpose(H)) + Q
K_left = np.matmul(np.matmul(P_left, np.transpose(H)), np.linalg.inv(
    K_left_denominator))  # P_left*H'/(H*P_left*H' + Q);

if (mean_foot_pos_left[0] == 0 and mean_foot_pos_left[1] == 0):
    retrack_left += 1
    tracking_left[0][t % 24] = 0
else:
    tmp_tra_left = np.array(
        [[mean_foot_pos_left[0]], [mean_foot_pos_left[1]]])
    pre_X_left = X_left + \
        np.matmul(K_left, (tmp_tra_left - np.matmul(H, X_left)))
    Pre_P_left = np.matmul(np.identity(4) - np.matmul(K_left, H), P_left)
    DM_left_nominator = tmp_tra_left - \
        np.matmul(H, pre_X_left)  # (tmp_tra_left-H*pre_X_left)
    DM_left_denominator = np.matmul(
        np.matmul(H, Pre_P_left), np.transpose(H)) + Q  # (H*Pre_P_left*H' + Q)
    DM_left = np.matmul(np.matmul(np.transpose(DM_left_nominator), np.linalg.inv(
        DM_left_denominator)), DM_left_nominator)  # (tmp_tra_left-H*pre_X_left)'/(H*Pre_P_left*H' + Q)*(tmp_tra_left-H*pre_X_left);
    if (DM_left < 5):
        X_left = pre_X_left
        P_left = Pre_P_left
        tracking_left[0][t % 24] = 1
    else:
        retrack_left += 1
        tracking_left[0][t % 24] = 0
start_tracking_left += 1
filter_pos_left = np.matmul(H, X_left)

# 右tracking
if (start_tracking_right == 0):
    x_right = mean_foot_pos_right[0]
    vx = 0
    y = mean_foot_pos_right[1]
    vy = 0
    X_right = np.array([[x_right], [vx], [y], [vy]])
else:
    X_right = np.matmul(T1, X_right) + np.matmul(T2, np.array([[1], [1]]))
    P_right = np.matmul(np.matmul(T1, P_right), np.transpose(
        T1)) + np.matmul(np.matmul(T2, R), np.transpose(T2))
predict_pos_right = np.matmul(H, X_right)
K_right_denominator = np.matmul(np.matmul(H, P_right),
                                np.transpose(H)) + Q
K_right = np.matmul(np.matmul(P_right, np.transpose(H)), np.linalg.inv(
    K_right_denominator))  # P_right*H'/(H*P_right*H' + Q);
if (mean_foot_pos_right[0] == 0 and mean_foot_pos_right[1] == 0):
    retrack_right += 1
    tracking_right[0][t % 24] = 0
else:
    tmp_tra_right = np.array(
        [[mean_foot_pos_right[0]], [mean_foot_pos_right[1]]])
    pre_X_right = X_right + \
        np.matmul(K_right, (tmp_tra_right - np.matmul(H, X_right)))
    Pre_P_right = np.matmul(np.identity(
        4) - np.matmul(K_right, H), P_right)
    DM_right_nominator = tmp_tra_right - \
        np.matmul(H, pre_X_right)  # (tmp_tra_right-H*pre_X_right)
    DM_right_denominator = np.matmul(
        np.matmul(H, Pre_P_right), np.transpose(H)) + Q  # (H*Pre_P_right*H' + Q)
    DM_right = np.matmul(np.matmul(np.transpose(DM_right_nominator), np.linalg.inv(
        DM_right_denominator)), DM_right_nominator)  # (tmp_tra_right-H*pre_X_right)'/(H*Pre_P_right*H' + Q)*(tmp_tra_right-H*pre_X_right);
    if (DM_right < 5):
        X_right = pre_X_right
        P_right = Pre_P_right
        tracking_right[0][t % 24] = 1
    else:
        retrack_right += 1
        tracking_right[0][t % 24] = 0
start_tracking_right += 1
filter_pos_right = np.matmul(H, X_right)

for j in range(S_n):
    if Z[j] == 1:
        # topic名稱為visualization_marker(固定的) 消息類型為Marker
        pub = rospy.Publisher("visualization_marker", Marker, queue_size=1)
        shape = Marker.CYLINDER

        marker1 = Marker()
        marker1.header.frame_id = "my_frame"
        marker1.header.stamp = rospy.Time.now()
        marker1.ns = "basic_shapes"
        marker1.id = 0

        marker1.type = shape
        marker1.action = Marker.ADD

        # 初始化marker的位置和角度
        marker1.pose.position.x = filter_pos_left[0]
        marker1.pose.position.y = filter_pos_left[1]
        marker1.pose.position.z = 0
        marker1.pose.orientation.x = 0.0
        marker1.pose.orientation.y = 0.0
        marker1.pose.orientation.z = 0.0
        marker1.pose.orientation.w = 1.0
        # marker的尺寸大小 1.0對應現實的1m
        marker1.scale.x = 0.5
        marker1.scale.y = 0.5
        marker1.scale.z = 0.5
        # marker的顏色 a為透明度0.0無法看到
        marker1.color.r = 0.0
        marker1.color.g = 1.0
        marker1.color.b = 0.0
        marker1.color.a = 1.0

        marker1.lifetime = rospy.Duration()
        marker2 = Marker()
        marker2.header.frame_id = "my_frame"
        marker2.header.stamp = rospy.Time.now()
        marker2.ns = "basic_shapes"
        marker2.id = 1

        marker2.type = shape
        marker2.action = Marker.ADD

        # 初始化marker的位置和角度
        marker2.pose.position.x = filter_pos_right[0]
        marker2.pose.position.y = filter_pos_right[1]
        marker2.pose.position.z = 0
        marker2.pose.orientation.x = 0.0

        marker2.pose.orientation.y = 0.0

        marker2.pose.orientation.z = 0.0

        marker2.pose.orientation.w = 1.0

        # marker的尺寸大小 1.0對應現實的1m
        marker2.scale.x = 0.5
        marker2.scale.y = 0.5
        marker2.scale.z = 0.5
        # marker的顏色 a為透明度0.0無法看到
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.color.a = 1.0

        marker2.lifetime = rospy.Duration()

        marker3 = Marker()
        marker3.header.frame_id = "my_frame"
        marker3.header.stamp = rospy.Time.now()
        marker3.ns = "basic_shapes"
        marker3.id = 2

        marker3.type = shape
        marker3.action = Marker.ADD

        # 初始化marker的位置和角度
        marker3.pose.position.x = predict_pos_left[0]
        marker3.pose.position.y = predict_pos_left[1]
        marker3.pose.position.z = 0
        marker3.pose.orientation.x = 0.0
        marker3.pose.orientation.y = 0.0
        marker3.pose.orientation.z = 0.0
        marker3.pose.orientation.w = 1.0
        # marker的尺寸大小 1.0對應現實的1m
        marker3.scale.x = 0.5
        marker3.scale.y = 0.5
        marker3.scale.z = 0.5
        # marker的顏色 a為透明度0.0無法看到
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 1.0
        marker3.color.a = 1.0

        marker3.lifetime = rospy.Duration()

        marker4 = Marker()
        marker4.header.frame_id = "my_frame"
        marker4.header.stamp = rospy.Time.now()
        marker4.ns = "basic_shapes"
        marker4.id = 4

        marker4.type = shape
        marker4.action = Marker.ADD

        # 初始化marker的位置和角度
        marker4.pose.position.x = predict_pos_right[0]
        marker4.pose.position.y = predict_pos_right[1]
        marker4.pose.position.z = 0
        marker4.pose.orientation.x = 0.0
        marker4.pose.orientation.y = 0.0
        marker4.pose.orientation.z = 0.0
        marker4.pose.orientation.w = 1.0
        # marker的尺寸大小 1.0對應現實的1m
        marker4.scale.x = 0.5
        marker4.scale.y = 0.5
        marker4.scale.z = 0.5
        # marker的顏色 a為透明度0.0無法看到
        marker4.color.r = 1.0
        marker4.color.g = 0.5
        marker4.color.b = 1.0
        marker4.color.a = 1.0

        marker4.lifetime = rospy.Duration()

        while pub.get_num_connections() < 1:
            # get_num_connections()是python版本查閱當前topic的訂閱器數目， cpp版本对应函数marker_pub.getNumScribers()
            # 用来判斷當前是否有訂閱器，也就是說rviz是否打開，如果沒有訂閱器沒必要發布消息。當然没有訂閱器情况下發布消息也是沒問題的。
            if rospy.is_shutdown():
                exit(1)
            rospy.loginfo("Please create a subscriber to the marker!")
            time.sleep(0.1)  # 休眠等待，不斷查詢有沒訂閱器

        pub.publish(marker1)  # 發布消息
        pub.publish(marker2)  # 發布消息
        pub.publish(marker3)  # 發布消息
        pub.publish(marker4)  # 發布消息

        # rate.sleep()  # 指定頻率刷新

        # plt.plot(Seg[j].xydata[:, 0], Seg[j].xydata[:, 1], 'r.')
        print("predict_pos_right")
        print(predict_pos_right)
        print("filter_pos_right")
        print(filter_pos_right)
        print("predict_pos_left")
        print(predict_pos_left)
        print("filter_pos_left")
        print(filter_pos_left)
    # else:
        # plt.plot(Seg[j].xydata[:, 0], Seg[j].xydata[:, 1], 'b.')
# plt.plot(predict_pos_left[0], predict_pos_left[1], 'y+')
# plt.plot(filter_pos_left[0], filter_pos_left[1], 'g^')
# plt.plot(predict_pos_right[0], predict_pos_right[1], 'k*')
# plt.plot(filter_pos_right[0], filter_pos_right[1], 'cx')
def Bayesfilter(Z, t):
T = np.array([[0.8, 0.2], [0.0004, 0.9996]])
Z0_filter = np.array([[0.1, 0], [0, 0.7]])
Z1_filter = np.array([[0.9, 0], [0, 0.3]])
X = np.array([0.5, 0.5])
filter_X = np.zeros((2, t))
backward_X = np.zeros((2, t))
smooth = np.zeros((2, t))
backward_X[0, t-1] = 1
backward_X[1, t-1] = 1
for i in range(t):
if Z[i] == 0:
temp = np.matmul(np.matmul(X, T), Z0_filter)
c = 1/(temp[0] + temp[1])
X = c*temp
filter_X[0, i] = X[0]
filter_X[1, i] = X[1]
else:
temp = np.matmul(np.matmul(X, T), Z1_filter)
c = 1/(temp[0] + temp[1])
X = c*temp
filter_X[0, i] = X[0]
filter_X[1, i] = X[1]
for i in range(t-2, -1, -1):
if Z[i+1] == 0:
backward_X[:, i] = np.matmul(
T, np.matmul(Z0_filter, backward_X[:, i+1]))
else:
backward_X[:, i] = np.matmul(
T, np.matmul(Z1_filter, backward_X[:, i+1]))
for i in range(t):
s = np.sum(np.multiply(filter_X[:, i], backward_X[:, i]), axis=0)
c2 = 1/(s)
smooth[:, i] = np.multiply(c2*filter_X[:, i], backward_X[:, i])
return smooth
def isbox(T1, T2, T3, T4, T5):
D = 0
for i in range(classified_num):
if train_data[0][i] == 1:
if train_data[4][i]:
if T1 > train_data[2][i]:
h = 1
else:
h = -1
else:
if T1 < train_data[2][i]:
h = 1
else:
h = -1
ah = train_data[3][i]*h
elif train_data[0][i] == 2:
if train_data[4][i]:
if T2 > train_data[2][i]:
h = 1
else:
h = -1
else:
if T2 < train_data[2][i]:
h = 1
else:
h = -1
ah = train_data[3][i]*h
elif train_data[0][i] == 3:
if train_data[4][i]:
if T3 > train_data[2][i]:
h = 1
else:
h = -1
else:
if T3 < train_data[2][i]:
h = 1
else:
h = -1
ah = train_data[3][i]*h
elif train_data[0][i] == 4:
if train_data[4][i]:
if T4 > train_data[2][i]:
h = 1
else:
h = -1
else:
if T4 < train_data[2][i]:
h = 1
else:
h = -1
ah = train_data[3][i]*h
elif train_data[0][i] == 5:
if train_data[4][i]:
if T5 > train_data[2][i]:
h = 1
else:
h = -1
else:
if T5 < train_data[2][i]:
h = 1
else:
h = -1
ah = train_data[3][i]*h
D = D + ah
return D
def linearity(xy_data, num_xy):
tmp_A = np.ones((num_xy, 2))
tmp_b = np.ones((num_xy, 1))
tmp_A[:, 0] = xy_data[:, 0]
tmp_b = xy_data[:, 1]
a, b = np.linalg.lstsq(tmp_A, tmp_b)[0]
diff_xy = np.linalg.norm(
a*xy_data[:, 0] + b*np.ones(num_xy) - xy_data[:, 1])
return diff_xy
def circularity(xy_data, num_xy, D):
tmp_A = np.ones((num_xy, 3))
tmp_A[:, 0] = xy_data[:, 0]*(-2)
tmp_A[:, 1] = xy_data[:, 1]*(-2)
tmp_b = np.transpose(-np.sum(np.power(xy_data, 2), axis=1))
c_x, c_y, c_r = np.linalg.lstsq(tmp_A, tmp_b)[0]
c_r = np.sqrt(c_x**2 + c_y**2 - c_r)
if D:
return c_r
else:
diff_xy = xy_data.copy()
diff_xy[:, 0] = diff_xy[:, 0] - np.transpose(np.ones((num_xy, 1))*c_x)
diff_xy[:, 1] = diff_xy[:, 1] - np.transpose(np.ones((num_xy, 1))*c_y)
Sc = np.sum(np.power((c_r*np.ones((num_xy, 1)) -
np.sqrt(np.sum(np.power(diff_xy, 2), axis=1))), 2))
return Sc
def Segment(xy_data):
X = xy_data[:, 0]
Y = xy_data[:, 1]
S_i = 1
S_n = 0
threshold = 0.1
tmp_n0ind = np.nonzero(xy_data)
if tmp_n0ind[0].size:
n0ind = np.array([tmp_n0ind[0][0]])
for i in range(1, tmp_n0ind[0].size):
if tmp_n0ind[0][i] > tmp_n0ind[0][i-1]:
n0ind = np.concatenate([n0ind, np.array([tmp_n0ind[0][i]])])
n_0 = n0ind.size
xy = np.array([X[n0ind[0]], Y[n0ind[0]]])
Seg = []
for i in range(1, n_0):
if np.sqrt((X[n0ind[i]] - X[n0ind[i-1]])**2 + (Y[n0ind[i]] - Y[n0ind[i-1]])**2) < threshold:
S_i += 1
new_xy = np.array([X[n0ind[i]], Y[n0ind[i]]])
xy = np.vstack([xy, new_xy])
else:
if xy.size > 4:
S_n = S_n + 1
Seg.append(Segment_xydata(xy, S_i))
xy = np.array([X[n0ind[i]], Y[n0ind[i]]])
S_i = 1
if xy.size > 4:
S_n = S_n + 1
Seg.append(Segment_xydata(xy, S_i))
display(Seg, S_n)
# KF
t = 0
dt = 0.1
timing = 120
x_left = 1
x_right = 1
vx = 0
y = 0
vy = 0
X_left = np.array([[x_left], [vx], [y], [vy]])
X_right = np.array([[x_right], [vx], [y], [vy]])
P_left = np.identity(4)
P_right = np.identity(4)
T1 = np.identity(4)
T1[0][1] = dt
T1[2][3] = dt
T2 = np.array([[dt**2/2, 0], [dt, 0], [0, dt**2/2], [0, dt]])
H = np.zeros((2, 4))
H[0, 0] = 1
H[1, 2] = 1
R = np.identity(2)*5
Q = np.array([[0.001474613368833, 0.000524673241571],
[0.000524673241571, 0.001269948264295]])
filter_pos_left = np.zeros((2, timing))
predict_pos_left = np.zeros((2, timing))
DM_left = np.zeros(timing)
filter_pos_right = np.zeros((2, timing))
predict_pos_right = np.zeros((2, timing))
DM_right = np.zeros(timing)
# initialize
start_tracking_left = 0
retrack_left = 0
start_tracking_right = 0
retrack_right = 0
tracking_left = np.ones((1, 24))
tracking_right = np.ones((1, 24))
'''
plt.ion()
plt.figure(1)
for i in range(0, 120):
plt.clf()
plt.title('sec=%i' % i)
x = XYT[0+360*i:360*(i+1), 0]
y = XYT[0+360*i:360*(i+1), 1]
plt.plot(x, y, ".")
plt.pause(0.1)
plt.ioff()
plt.show()
'''
def callback(data):
tmp = np.array([1000, 1000])  # 改改看
for i in range(360):
    rad = data.angle_min+data.angle_increment*i
    ranges = data.ranges
    xydata = np.array([math.cos(rad)*ranges[i], math.sin(rad)*ranges[i]])
    if i == 0:
        xy_data = xydata
    else:
        xy_data = np.vstack([xy_data, xydata])

Segment(xy_data)
def listener():
rospy.init_node('listener', anonymous=True)
rate = rospy.Rate(0.1)  # 1hz
while not rospy.is_shutdown():
rospy.Subscriber("scan", LaserScan, callback)
rate.sleep()
# rospy.spin()
# f.close
# plt.ion()
# plt.figure(1)
if __name__ == '__main__':
listener()
