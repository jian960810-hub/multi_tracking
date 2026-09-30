#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import os
import math
import numpy as np
import copy
import sys
import time
import random
import rospkg
path_test = os.path.abspath(os.path.dirname('adaboost_trained_data_mess_430.txt'))
path1_test = os.path.abspath('adaboost_trained_data_mess_430.txt')
path2_test = os.path.abspath('stationary_simple_bencnmark.txt')
path1 = os.path.join(rospkg.RosPack().get_path('turtlebot3_sample'), 'src', 'adaboost_trained_data_mess_430.txt')
path2 = os.path.join(rospkg.RosPack().get_path('turtlebot3_sample'), 'src', 'stationary_simple_bencnmark.txt')

f = open(path1)
train_data = np.loadtxt(f)
classified_num = np.size(train_data, 1)
print("classified_num:", classified_num)
f.close
f1 = open(path2)
(a) = np.loadtxt(f1, delimiter=',')
f1.close


class Segment_xydata:
    def __init__(self, xy, num):
        self.xydata = xy
        self.num_xy = num
        self.T1 = num  # number of points
        self.T2 = np.sqrt(np.sum(np.power(np.std(xy, axis=0), 2)))  # std
        self.T3 = boundary_std(xy, num)  # boundary regularity
        self.T4 = circularity(xy, num, 0)  # circularity
        self.T5 = circularity(xy, num, 1)  # radius
        self.T6 = boundary_length(xy, num)  # boundary length
        self.T7 = mean_average_deviation_from_median(
            xy, num)  # mean average deviation from median
        self.T10 = linearity(xy, num)  # linearity
        self.T11 = np.linalg.norm(xy[0, :]-xy[num-1, :])  # width
        self.T12 = mean_angular(xy, num)
        self.T13 = mean_curvature(xy, num)

    def get_xydata(self):
        return self.xydata.copy()


class Trackor(Segment_xydata):  # 這裡的xydata是全部segment的點的值. xy_data則是segment的平均.
    def __init__(self, xy, num):
        super().__init__(xy, num)
        self.xy_data = np.mean(self.xydata, axis=0)


class Tracking_Target_Candidate:  # 兩隻腳距離小於threshold的點,當作trackor的候選人
    def __init__(self, xy_data, distance):
        self.xy_data = xy_data
        self.distance = distance


class Tracking_Target:  # 真正發布的trackor
    def __init__(self, xy_data, distance):
        self.xy_data = xy_data
        self.distance = distance
        self.scan_count = 1  # scan count >= 5才畫出trackor
        self.scan_array = np.ones((1, 20))  # 用來計算retrack
        self.flag = 1  # 用來判斷是否成功data associate
        self.X = np.array([[1], [0], [0], [0]])
        self.P = np.identity(4)
        self.T1 = np.array([[1, 0.1, 0, 0], [0, 1, 0, 0],
                           [0, 0, 1, 0.1], [0, 0, 0, 1]])
        self.T2 = np.array([[dt**2/2, 0], [dt, 0], [0, dt**2/2], [0, dt]])
        self.start_tracking = 0
        self.retrack = 0
        self.candidate_paired_count = 1  # 用來計算與candidate trackor配對了幾次
        self.color = (random.random(), random.random(), random.random())


class To_Be_Delete_Points:  # xy_data是要刪掉的點的座標
    def __init__(self, xy_data):
        self.xy_data = xy_data


class Data_Association_Points:
    def __init__(self, xy_data, candidate_or_not):
        self.xy_data = xy_data
        self.is_candidate = candidate_or_not  # 用來看這個點是不是candidate trackor的.


def display(Seg, S_n):
    pub = rospy.Publisher("visualization_marker", Marker, queue_size=100)
    global published_trackor
    global t
    t += 1
    print("********************************************************************************************************************")
    print("sec:", t)
    Z = np.zeros(S_n)
    for i in range(S_n):
        if (i == 0):
            distance_from_succeeding = distance_from_succeeding_segment(
                Seg[i].xydata, Seg[i+1].xydata, Seg[i].T1)
            distance_from_preceeding = distance_from_preceeding_segment(
                Seg[i].xydata, Seg[S_n-1].xydata, Seg[S_n-1].T1)
        elif (i == S_n-1):
            distance_from_succeeding = distance_from_succeeding_segment(
                Seg[i].xydata, Seg[0].xydata, Seg[i].T1)
            distance_from_preceeding = distance_from_preceeding_segment(
                Seg[i].xydata, Seg[i-1].xydata, Seg[i-1].T1)
        else:
            distance_from_succeeding = distance_from_succeeding_segment(
                Seg[i].xydata, Seg[i+1].xydata, Seg[i].T1)
            distance_from_preceeding = distance_from_preceeding_segment(
                Seg[i].xydata, Seg[i-1].xydata, Seg[i-1].T1)
        if isbox(Seg[i].T1, Seg[i].T2, Seg[i].T3, Seg[i].T4, Seg[i].T5, Seg[i].T6, Seg[i].T7, distance_from_preceeding, distance_from_succeeding, Seg[i].T10, Seg[i].T11, Seg[i].T12, Seg[i].T13) > 0:
            Z[i] = 1

    # Trackor
    trackor = []
    candidate_trackor = []
    detected_points = []
    Detected_Points_For_Candidate_Trackor = []
    Points_For_Data_Associate = []
    # 存candidate trackor用來與published trackor配對完要刪掉的detected points
    Detected_Points_To_Be_Deleted = []
    for i in range(S_n):
        if (Z[i] == 1):
            trackor.append(Trackor(Seg[i].xydata, Seg[i].num_xy))
            detected_points.append(Trackor(Seg[i].xydata, Seg[i].num_xy))

    trackor_length = len(trackor)
    for i in range(trackor_length):
        trackor[i].mean_segment_position = np.mean(trackor[i].xydata, axis=0)
    # Detected_Points_For_Pairing 是紀錄最近兩點的配對,該兩點的座標.
    Detected_Points_For_Pairing = np.zeros(((trackor_length//2)*2, 2))
    distance_between_detected_points = np.ones((trackor_length//2, 1))
    detected_position = np.ones((trackor_length//2, 2))
    detected_idx = np.ones((trackor_length//2, 2))
    for i in range(trackor_length//2):
        [distance, detected_position_1, detected_position_2,
            detected_idx_1, detected_idx_2] = brute_force_pairing(trackor)
        distance_between_detected_points[i][0] = distance
        Detected_Points_For_Pairing[2*i][:] = detected_position_1
        Detected_Points_For_Pairing[2*i+1][:] = detected_position_2
        detected_position[i][:] = [
            (g + h) / 2 for g, h in zip(detected_position_1, detected_position_2)]  # detected_position存配對到的點的中點
        detected_idx[i][0] = detected_idx_1  # detected_idx存配對的點的index
        detected_idx[i][1] = detected_idx_2
        del (trackor[detected_idx_1])  # 刪除配對好的點,不然會重複用到
        del (trackor[detected_idx_2-1])  # 刪除配對好的點,不然會重複用到

    for i in range(len(detected_points)):
        print("detected_points:", detected_points[i].xy_data)

    # Tracking Target Candidate
    tracking_target_count = 0  # tracking_target_count是candidate_trackor的數量
    for i in range(trackor_length//2):  # 小於一定距離的腳發布candidate_trackor,但還沒開始track
        if (distance_between_detected_points[i] < 0.8):  # threshold = 0.8m
            candidate_trackor.append(Tracking_Target_Candidate(
                detected_position[i][:], distance_between_detected_points[i]))
            Detected_Points_For_Candidate_Trackor.append(To_Be_Delete_Points(
                Detected_Points_For_Pairing[2*i][:]))  # 用來記candidate trackor是用哪兩個點合成的,這是合成用的第一個點
            Detected_Points_For_Candidate_Trackor.append(To_Be_Delete_Points(
                Detected_Points_For_Pairing[2*i+1][:]))  # 用來記candidate trackor是用哪兩個點合成的,這是合成用的第二個點
            tracking_target_count += 1
            print("candidate_trackor[i].xy_data:",
                  candidate_trackor[i].xy_data)

    # 刪掉candidate trackor用過的點
    for i in range(len(Detected_Points_For_Candidate_Trackor)):
        for j in range(len(detected_points)):
            if ((detected_points[j].xy_data[0] == Detected_Points_For_Candidate_Trackor[i].xy_data[0]) & (detected_points[j].xy_data[1] == Detected_Points_For_Candidate_Trackor[i].xy_data[1])):
                detected_points.pop(j)
                print("pop detected!")
                break

    for i in range(len(Detected_Points_For_Candidate_Trackor)):
        print("Detected_Points_For_Candidate_Trackor:",
              Detected_Points_For_Candidate_Trackor[i].xy_data)
    for i in range(len(detected_points)):
        print("detected_points:", detected_points[i].xy_data)
    for i in range(len(candidate_trackor)):
        Points_For_Data_Associate.append(
            Data_Association_Points(candidate_trackor[i].xy_data, 1))
    for i in range(len(detected_points)):
        Points_For_Data_Associate.append(
            Data_Association_Points(detected_points[i].xy_data, 0))
    for i in range(len(Points_For_Data_Associate)):
        print("Points_For_Data_Associate:",
              Points_For_Data_Associate[i].xy_data)

    # 每回合重置publihed trackor的flag
    for i in range(len(published_trackor)):
        published_trackor[i].flag = 0

    # Cost Matrix 初始化
    Cost_Matrix = np.zeros(
        (len(published_trackor), len(Points_For_Data_Associate)))

    # Tracking Target
    if (len(published_trackor) == 0):  # 如果目前沒有人在published_trackor，則把candidate_trackor東西丟進來。
        for i in range(tracking_target_count):
            published_trackor.append(Tracking_Target(
                candidate_trackor[i].xy_data, candidate_trackor[i].distance))
    else:  # 已有物件在publish_trackor,則用detected_points與其比較.若兩者最小距離小於threshold,則data associate.

        # 做Cost Matrix
        for i in range(len(published_trackor)):
            distance_from_each_data_association_points = np.zeros(
                (len(Points_For_Data_Associate), 1))
            for j in range(len(Points_For_Data_Associate)):
                distance_from_each_data_association_points[j] = mahalanobis_distance(
                    Points_For_Data_Associate[j].xy_data, published_trackor[i].Predict_X, published_trackor[i].Predict_P)
                Cost_Matrix[i][j] = distance_from_each_data_association_points[j]
        # 配對detected points與published trackor.
        # 匈牙利演算法
        print("Cost_Matrix:")
        print(Cost_Matrix)
        if (np.size(Cost_Matrix, 0) & (np.size(Cost_Matrix, 1) != 0)):  # 把Cost Matrix Cost過大的Row弄掉
            for i in range(np.size(Cost_Matrix, 0)):
                if (Cost_Matrix[i][:].min() > 600):
                    Cost_Matrix[i][:] = np.zeros(np.size(Cost_Matrix, 1))
        Cost_Matrix_Row = np.size(Cost_Matrix, 0)
        Cost_Matrix_Column = np.size(Cost_Matrix, 1)
        if ((Cost_Matrix_Row != 0) & (Cost_Matrix_Column != 0)):
            print("Cost_Matrix_Row:", Cost_Matrix_Row)
            print("Cost_Matrix_Column:", Cost_Matrix_Column)

            if (Cost_Matrix_Row < Cost_Matrix_Column):
                col_minus_row = Cost_Matrix_Column - Cost_Matrix_Row
                for i in range(col_minus_row):
                    Cost_Matrix = np.append(
                        Cost_Matrix, np.zeros((1, Cost_Matrix_Column)), axis=0)
            else:
                row_minus_col = Cost_Matrix_Row - Cost_Matrix_Column
                for i in range(row_minus_col):
                    Cost_Matrix = np.append(
                        Cost_Matrix, np.zeros((Cost_Matrix_Row, 1)), axis=1)
            print("Adjusted_Cost_Matrix:", Cost_Matrix)
            # Get the element position.
            ans_pos = hungarian_algorithm(Cost_Matrix.copy())
            # Get the minimum or maximum value and corresponding matrix.
            ans, ans_mat = ans_calculation(Cost_Matrix, ans_pos)

            # Show the result
            print(f"Linear Assignment problem result: {ans:.0f}\n{ans_mat}")

            for i in range(Cost_Matrix_Row):
                data_association_ith_published_trackor_flag = 0
                for j in range(Cost_Matrix_Column):
                    if ((ans_mat[i][j] != 0) & (ans_mat[i][j] < 250)):
                        print("paired! [ trackor:", i, "detected:", j, "]")
                        published_trackor[i].xy_data = Points_For_Data_Associate[j].xy_data
                        if (Points_For_Data_Associate[j].is_candidate == 1):
                            published_trackor[i].candidate_paired_count += 1
                            for k in range(len(candidate_trackor)):
                                if ((Points_For_Data_Associate[j].xy_data[0] == candidate_trackor[k].xy_data[0]) & (Points_For_Data_Associate[j].xy_data[1] == candidate_trackor[k].xy_data[1])):
                                    candidate_trackor.pop(k)
                                    print("pop candidate!")
                                    break
                        print("break!")
                        published_trackor[i].scan_count += 1
                        published_trackor[i].flag = 1
                        data_association_ith_published_trackor_flag = 1
                if (data_association_ith_published_trackor_flag == 0):
                    published_trackor[i].flag = 0
        else:
            for i in range(len(published_trackor)):
                published_trackor[i].flag = 0

        # 計算每個candidate_trackor與每個published_trackor的馬氏距離.
        for i in range(len(published_trackor)):
            candidate_trackor_pop_count = 0
            for j in range(len(candidate_trackor)):
                distance_from_each_candidate_trackor = mahalanobis_distance(
                    candidate_trackor[j-candidate_trackor_pop_count].xy_data, published_trackor[i].Predict_X, published_trackor[i].Predict_P)
                # candidate_trackor與published_trackor距離小於200馬氏距離的被排除(因為可能被當成data associate的點).
                if (distance_from_each_candidate_trackor < 250):
                    candidate_trackor.pop(j-candidate_trackor_pop_count)
                    candidate_trackor_pop_count += 1

        # candidate_trackor與published_trackor距離小於200馬氏距離的在上面的程式被排除了,其餘的點距離已經在追蹤中的published_trackor有段距離,因此,將其視為新的target.
        for i in range(len(candidate_trackor)):
            print("ADD CANDIDATE!")
            published_trackor.append(Tracking_Target(
                candidate_trackor[i].xy_data, candidate_trackor[i].distance))

    for i in range(len(published_trackor)):
        print("用來update的點:", published_trackor[i].xy_data)

    for i in range(len(published_trackor)):
        Kalman_filter(published_trackor[i],
                      t, published_trackor[i].flag)

    # 刪除掉15scan裡沒有10成功的tracking
    published_trackor_pop_count = 0
    for i in range(len(published_trackor)):
        if (np.sum(published_trackor[i-published_trackor_pop_count].scan_array) < 12):
            published_trackor.pop(i-published_trackor_pop_count)
            published_trackor_pop_count += 1

    print("published_trackor length:", len(published_trackor))
    actual_tracking_number = 0
    '''
    for i in range(len(published_trackor)):
        if (published_trackor[i].candidate_paired_count >= 5):
            plt.plot(published_trackor[i].filter_position[0],
                     published_trackor[i].filter_position[1], c=published_trackor[i].color, marker='o', markersize=10)
            actual_tracking_number += 1
            print("actual tracking position: (", published_trackor[i].filter_position[0][0],
                  published_trackor[i].filter_position[1][0], ")")
        else:
            plt.plot(published_trackor[i].filter_position[0],
                     published_trackor[i].filter_position[1], color='orange', marker='*', markersize=10)
            # 如果published_trackor至少跟candiate trackor配對5次(published_trackor[i].candidate_paired_count >= 5)才會畫出來
    print("actual_tracking_number:", actual_tracking_number)

    # 畫laser scan
    for j in range(S_n):
        if Z[j] == 1:
            plt.plot(Seg[j].xydata[:, 0],
                     Seg[j].xydata[:, 1], 'r.')  # detect到的點
        else:
            plt.plot(Seg[j].xydata[:, 0], Seg[j].xydata[:, 1], 'b.')
    plt.show()
    '''
    for i in range(len(published_trackor)):

        shape = Marker.CYLINDER
        marker = Marker()
        marker.header.frame_id = "base_scan"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic_shapes"
        marker.id = i
        marker.type = shape
        marker.action = Marker.ADD
        # 初始化marker的位置和角度
        marker.pose.position.x = published_trackor[i].filter_position[0]
        marker.pose.position.y = published_trackor[i].filter_position[1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        # marker的尺寸大小 1.0對應現實的1m
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.5
        # marker的顏色 a為透明度0.0無法看到
        marker.color.r = published_trackor[i].color[0]
        marker.color.g = published_trackor[i].color[1]
        marker.color.b = published_trackor[i].color[2]
        marker.color.a = 1.0

        shape1 = Marker.CUBE
        marker1 = Marker()
        marker1.header.frame_id = "base_scan"
        marker1.header.stamp = rospy.Time.now()
        marker1.ns = "basic_shapes"
        marker1.id = i+len(published_trackor)
        marker1.type = shape1
        marker1.action = Marker.ADD
        # 初始化marker的位置和角度
        marker1.pose.position.x = published_trackor[i].filter_position[0]
        marker1.pose.position.y = published_trackor[i].filter_position[1]
        marker1.pose.position.z = 0
        marker1.pose.orientation.x = 0.0
        marker1.pose.orientation.y = 0.0
        marker1.pose.orientation.z = 0.0
        marker1.pose.orientation.w = 1.0
        # marker的尺寸大小 1.0對應現實的1m
        marker1.scale.x = 0.2
        marker1.scale.y = 0.2
        marker1.scale.z = 0.2
        # marker的顏色 a為透明度0.0無法看到
        marker1.color.r = published_trackor[i].color[0]
        marker1.color.g = published_trackor[i].color[1]
        marker1.color.b = published_trackor[i].color[2]
        marker1.color.a = 1.0

        marker.lifetime = rospy.Duration(0.2)
        marker1.lifetime = rospy.Duration(0.2)
        # 如果published_trackor至少被掃到5次(scan count >= 5)才會畫出來
        if (published_trackor[i].candidate_paired_count >= 5):
            pub.publish(marker)
        else:
            pub.publish(marker1)
        


def calculate_distance(x, y):
    return math.sqrt((x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2)


# ret是配對到的點的距離.trackor_list[a],trackor_list[b]是配對的兩個點的座標.a,b是他們的index.
def brute_force_pairing(trackor):
    length = len(trackor)
    tmp_np = trackor[0].mean_segment_position
    for i in range(length-1):
        tmp_np = np.vstack(
            [tmp_np, trackor[i+1].mean_segment_position])
    trackor_list = tmp_np.tolist()
    ret = sys.maxsize
    a, b = None, None
    n = len(trackor_list)
    for i in range(n):
        for j in range(i+1, n):
            dis = calculate_distance(trackor_list[i], trackor_list[j])
            if dis < ret:
                ret = dis
                a, b = i, j
    return ret, trackor_list[a], trackor_list[b], a, b


def mahalanobis_distance(detected_position, Pre_X, Pre_P):
    foot_pos = np.array([detected_position])
    foot_pos_transpose = np.transpose(foot_pos)
    # (foot_pos(i,:)'-H*pre_X_left)
    Dm_up_left = foot_pos_transpose-np.matmul(H, Pre_X)
    Dm_down_left = np.matmul(np.matmul(H, Pre_P),
                             np.transpose(H)) + Q  # (H*Pre_P_left*H' + Q)
    mahalanobis_dis_from_leg = np.matmul(
        np.matmul(np.transpose(Dm_up_left), np.linalg.inv(Dm_down_left)), Dm_up_left)
    return mahalanobis_dis_from_leg


def Kalman_filter(target, t, flag):
    global R
    global Q
    # 繼續tracking與否
    if (np.sum(target.scan_array) < 12):
        target.start_tracking = 0
        target.retrack = 0
        target.scan_array = np.ones((1, 20))
    # tracking
    if (target.scan_count == 1):
        leg_x_coordinate = target.xy_data[0]
        vx = 0
        leg_y_coordinate = target.xy_data[1]
        vy = 0
        target.X = np.array(
            [[leg_x_coordinate], [vx], [leg_y_coordinate], [vy]])
    else:
        target.X = np.matmul(target.T1, target.X) + \
            np.matmul(target.T2, np.array([[1], [1]]))
        target.P = np.matmul(np.matmul(target.T1, target.P), np.transpose(
            target.T1)) + np.matmul(np.matmul(target.T2, R), np.transpose(target.T2))
    target.predict_pos = np.matmul(H, target.X)
    K_denominator = np.matmul(np.matmul(H, target.P),
                              np.transpose(H)) + Q
    K = np.matmul(np.matmul(target.P, np.transpose(H)), np.linalg.inv(
        K_denominator))  # P_left*H'/(H*P_left*H' + Q);
    if (flag == 0):
        target.retrack += 1
        target.scan_array[0][t % 20] = 0
    else:
        tmp_tra = np.array(
            [[target.xy_data[0]], [target.xy_data[1]]])
        target.Predict_X = target.X + \
            np.matmul(K, (tmp_tra - np.matmul(H, target.X)))
        target.Predict_P = np.matmul(
            np.identity(4) - np.matmul(K, H), target.P)
        DM_nominator = tmp_tra - \
            np.matmul(H, target.Predict_X)  # (tmp_tra_left-H*pre_X_left)
        DM_denominator = np.matmul(
            np.matmul(H, target.Predict_P), np.transpose(H)) + Q  # (H*Pre_P_left*H' + Q)
        DM = np.matmul(np.matmul(np.transpose(DM_nominator), np.linalg.inv(
            DM_denominator)), DM_nominator)  # (tmp_tra_left-H*pre_X_left)'/(H*Pre_P_left*H' + Q)*(tmp_tra_left-H*pre_X_left);
        # if (DM < 5):
        target.X = target.Predict_X
        target.P = target.Predict_P
        target.scan_array[0][t % 20] = 1
        # else:
        # target.retrack += 1
        # target.scan_array[0][t % 15] = 0
    target.start_tracking += 1
    target.filter_position = np.matmul(H, target.X)
    print("tracking position: (",
          target.filter_position[0][0], target.filter_position[1][0], ")")
    print("scan_array:", target.scan_array)
    print("scan_array_sum:", np.sum(target.scan_array))


def isbox(T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13):
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
        elif train_data[0][i] == 6:
            if train_data[4][i]:
                if T6 > train_data[2][i]:
                    h = 1
                else:
                    h = -1
            else:
                if T6 < train_data[2][i]:
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        elif train_data[0][i] == 7:
            if train_data[4][i]:
                if T7 > train_data[2][i]:
                    h = 1
                else:
                    h = -1
            else:
                if T7 < train_data[2][i]:
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        elif train_data[0][i] == 8:
            if train_data[4][i]:
                if T8 > train_data[2][i]:
                    h = 1
                else:
                    h = -1
            else:
                if T8 < train_data[2][i]:
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        elif train_data[0][i] == 9:
            if train_data[4][i]:
                if T9 > train_data[2][i]:
                    h = 1
                else:
                    h = -1
            else:
                if T9 < train_data[2][i]:
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        elif train_data[0][i] == 10:
            if train_data[4][i]:
                if T10 > train_data[2][i]:
                    h = 1
                else:
                    h = -1
            else:
                if T10 < train_data[2][i]:
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        elif train_data[0][i] == 11:
            if train_data[4][i]:
                if T11 > train_data[2][i]:
                    h = 1
                else:
                    h = -1
            else:
                if T11 < train_data[2][i]:
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        elif train_data[0][i] == 12:
            if train_data[4][i]:
                if T12 > train_data[2][i]:
                    h = 1
                else:
                    h = -1
            else:
                if T12 < train_data[2][i]:
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        elif train_data[0][i] == 13:
            if train_data[4][i]:
                if T13 > train_data[2][i]:
                    h = 1
                else:
                    h = -1
            else:
                if T13 < train_data[2][i]:
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        D = D + ah
    return D


def boundary_length(xy_data, num_xy):
    distance = 0
    for i in range(num_xy-1):
        distance = np.linalg.norm(xy_data[i, :]-xy_data[i+1, :]) + distance
    return distance


def boundary_std(xy_data, num_xy):
    distance = np.zeros(num_xy-1)
    for i in range(num_xy-1):
        distance[i] = np.linalg.norm(xy_data[i, :]-xy_data[i+1, :])
    std = np.std(distance, ddof=1)
    return std


def mean_curvature(xy_data, num_xy):
    curvature_sum = 0
    for i in range(num_xy-2):
        x1 = xy_data[i, 0]
        x2 = xy_data[i+1, 0]
        x3 = xy_data[i+2, 0]
        y1 = xy_data[i, 1]
        y2 = xy_data[i+1, 1]
        y3 = xy_data[i+2, 1]
        da = math.sqrt((x1-x2)**2+(y1-y2)**2)
        db = math.sqrt((x2-x3)**2+(y2-y3)**2)
        dc = math.sqrt((x3-x1)**2+(y3-y1)**2)
        p = (da + db + dc)/2
        area = math.sqrt(p*(p-da)*(p-db)*(p-dc))
        curvature = (4*area)/(da*db*dc)
        curvature_sum += curvature
    mean = curvature_sum/(num_xy-2)
    return mean


def mean_angular(xy_data, num_xy):
    theta_sum = 0
    for i in range(num_xy-2):
        x1 = xy_data[i, 0]
        x2 = xy_data[i+1, 0]
        x3 = xy_data[i+2, 0]
        y1 = xy_data[i, 1]
        y2 = xy_data[i+1, 1]
        y3 = xy_data[i+2, 1]
        v1 = np.array([x1-x2, y1-y2])
        v2 = np.array([x3-x2, y3-y2])
        length_v1 = np.linalg.norm(v1)
        length_v2 = np.linalg.norm(v2)
        cosine = np.dot(v1, v2)/(length_v1*length_v2)
        theta_sum += math.acos(cosine)
    mean = theta_sum/(num_xy-2)
    return mean


def distance_from_preceeding_segment(current_segment, preceeding_segment, number_of_points_preceeding):
    distance_from_preceeding = np.linalg.norm(
        current_segment[0, :]-preceeding_segment[number_of_points_preceeding-1, :])
    return distance_from_preceeding


def distance_from_succeeding_segment(current_segment, succeeding_segment, number_of_points_current):
    distance_from_succeeding = np.linalg.norm(
        current_segment[number_of_points_current-1, :]-succeeding_segment[0, :])
    return distance_from_succeeding


def mean_average_deviation_from_median(xy_data, num_xy):
    if (num_xy % 2 == 1):
        K = round((num_xy + 1)/2)
        median = xy_data[K-1, :]
    else:
        K = round(num_xy/2)
        median = (xy_data[K, :]+xy_data[K-1, :])/2
    mean_average_deviation = np.linalg.norm(xy_data-median)/num_xy
    return mean_average_deviation


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
            # print("S_i = ", S_i)
            # print("xy_size = ", xy.size)
            Seg.append(Segment_xydata(xy, S_i))
        # print(Seg)
        display(Seg, S_n)


def min_zero_row(zero_mat, mark_zero):
    '''
    The function can be splitted into two steps:
    #1 The function is used to find the row which containing the fewest 0.
    #2 Select the zero number on the row, and then marked the element corresponding row and column as False
    '''

    # Find the row
    min_row = [99999, -1]

    for row_num in range(zero_mat.shape[0]):
        if np.sum(zero_mat[row_num] == True) > 0 and min_row[0] > np.sum(zero_mat[row_num] == True):
            min_row = [np.sum(zero_mat[row_num] == True), row_num]

    # Marked the specific row and column as False
    zero_index = np.where(zero_mat[min_row[1]] == True)[0][0]
    mark_zero.append((min_row[1], zero_index))
    zero_mat[min_row[1], :] = False
    zero_mat[:, zero_index] = False


def mark_matrix(mat):
    '''
    Finding the returning possible solutions for LAP problem.
    '''

    # Transform the matrix to boolean matrix(0 = True, others = False)
    cur_mat = mat
    zero_bool_mat = (cur_mat == 0)
    zero_bool_mat_copy = zero_bool_mat.copy()

    # Recording possible answer positions by marked_zero
    marked_zero = []
    while (True in zero_bool_mat_copy):
        min_zero_row(zero_bool_mat_copy, marked_zero)

    # Recording the row and column positions seperately.
    marked_zero_row = []
    marked_zero_col = []
    for i in range(len(marked_zero)):
        marked_zero_row.append(marked_zero[i][0])
        marked_zero_col.append(marked_zero[i][1])

    # Step 2-2-1
    non_marked_row = list(set(range(cur_mat.shape[0])) - set(marked_zero_row))

    marked_cols = []
    check_switch = True
    while check_switch:
        check_switch = False
        for i in range(len(non_marked_row)):
            row_array = zero_bool_mat[non_marked_row[i], :]
            for j in range(row_array.shape[0]):
                # Step 2-2-2
                if row_array[j] == True and j not in marked_cols:
                    # Step 2-2-3
                    marked_cols.append(j)
                    check_switch = True

        for row_num, col_num in marked_zero:
            # Step 2-2-4
            if row_num not in non_marked_row and col_num in marked_cols:
                # Step 2-2-5
                non_marked_row.append(row_num)
                check_switch = True
    # Step 2-2-6
    marked_rows = list(set(range(mat.shape[0])) - set(non_marked_row))

    return (marked_zero, marked_rows, marked_cols)


def adjust_matrix(mat, cover_rows, cover_cols):
    cur_mat = mat
    non_zero_element = []

    # Step 4-1
    for row in range(len(cur_mat)):
        if row not in cover_rows:
            for i in range(len(cur_mat[row])):
                if i not in cover_cols:
                    non_zero_element.append(cur_mat[row][i])
    min_num = min(non_zero_element)

    # Step 4-2
    for row in range(len(cur_mat)):
        if row not in cover_rows:
            for i in range(len(cur_mat[row])):
                if i not in cover_cols:
                    cur_mat[row, i] = cur_mat[row, i] - min_num
    # Step 4-3
    for row in range(len(cover_rows)):
        for col in range(len(cover_cols)):
            cur_mat[cover_rows[row], cover_cols[col]
                    ] = cur_mat[cover_rows[row], cover_cols[col]] + min_num
    return cur_mat


def hungarian_algorithm(mat):
    dim = mat.shape[0]
    cur_mat = mat

    # Step 1 - Every column and every row subtract its internal minimum
    for row_num in range(mat.shape[0]):
        cur_mat[row_num] = cur_mat[row_num] - np.min(cur_mat[row_num])

    for col_num in range(mat.shape[1]):
        cur_mat[:, col_num] = cur_mat[:, col_num] - np.min(cur_mat[:, col_num])
    zero_count = 0
    while zero_count < dim:
        # Step 2 & 3
        ans_pos, marked_rows, marked_cols = mark_matrix(cur_mat)
        zero_count = len(marked_rows) + len(marked_cols)

        if zero_count < dim:
            cur_mat = adjust_matrix(cur_mat, marked_rows, marked_cols)

    return ans_pos


def ans_calculation(mat, pos):
    total = 0
    ans_mat = np.zeros((mat.shape[0], mat.shape[1]))
    for i in range(len(pos)):
        total += mat[pos[i][0], pos[i][1]]
        ans_mat[pos[i][0], pos[i][1]] = mat[pos[i][0], pos[i][1]]
    return total, ans_mat


# KF
published_trackor = []
t = 0
dt = 0.2

# 可以改,我用hard coding直接輸值而已.原本是
# T1 = np.identity(4)
# T1[0][1] = dt
# T1[2][3] = dt
H = np.zeros((2, 4))
H[0, 0] = 1
H[1, 2] = 1
R = np.identity(2)*5
Q = np.array([[0.001474613368833, 0.000524673241571],
              [0.000524673241571, 0.001269948264295]])*1

# 資料轉換
ang = a[:, 0]
dis = a[:, 1]
X = np.cos(ang)*dis
Y = np.sin(ang)*dis
XY = np.array([X, Y])
XYT = np.transpose(XY)


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
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
