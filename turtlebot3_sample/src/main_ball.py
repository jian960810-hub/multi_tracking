import rospy
from sensor_msgs.msg import LaserScan
import os
import math
import numpy as np
import copy

f = open('New_trainball.txt')
train_data = np.loadtxt(f)
classified_num = np.size(train_data,1)
f.close

class Segment_xydata:
    def __init__(self,xy_data,num):
        self.xydata = xy_data
        self.num_xy = num
        self.STD = np.sqrt(np.sum(np.power(np.std(xy_data,axis=0),2)))
        self.BS = boundary_std(xy_data,num)
        self.CR = circularity(xy_data,num)
        self.RS = radius(xy_data,num)
        self.BL = boundary_length(xy_data,num)

    def get_xydata(self):
        return self.xydata.copy()
        
def display(Seg,S_n):
    Z = np.zeros(S_n)
    for i in range(S_n):
        if isbox(Seg[i].num_xy,Seg[i].STD,Seg[i].BS,Seg[i].CR,Seg[i].RS,Seg[i].BL)>0:
            Z[i] = 1;
    SM = Bayesfilter(Z,S_n)
    for j in range(S_n):
        if Z[j] == 1:
            print(np.mean(Seg[j].xydata,axis = 0),"probability: ",SM[1,j],j,'This is ball')
            
def Bayesfilter(Z,t):
    T = np.array([[0.8,0.2] , [0.0004,0.9996]])
    Z0_filter = np.array([[0.1,0] , [0,0.7]])
    Z1_filter = np.array([[0.9,0] , [0,0.3]])
    X = np.array([0.5,0.5])
    filter_X = np.zeros((2,t))
    backward_X = np.zeros((2,t))
    smooth = np.zeros((2,t))
    backward_X[0,t-1] = 1
    backward_X[1,t-1] = 1
    for i in range(t):
        if Z[i]==0:
            temp = np.matmul(np.matmul(X,T),Z0_filter)
            c = 1/(temp[0] + temp[1])
            X = c*temp;
            filter_X[0,i] = X[0]
            filter_X[1,i] = X[1]
        else:
            temp = np.matmul(np.matmul(X,T),Z1_filter)
            c = 1/(temp[0] + temp[1])
            X = c*temp;
            filter_X[0,i] = X[0]
            filter_X[1,i] = X[1]
    for i in range(t-2,-1,-1):
        if Z[i+1]==0:
            backward_X[:,i] = np.matmul(T,np.matmul(Z0_filter,backward_X[:,i+1]))
        else:
            backward_X[:,i] = np.matmul(T,np.matmul(Z1_filter,backward_X[:,i+1]))
    for i in range(t):
        s = np.sum(np.multiply(filter_X[:,i],backward_X[:,i]),axis=0)
        c2 = 1/(s)
        smooth[:,i] = np.multiply(c2*filter_X[:,i],backward_X[:,i])
    return smooth

def isbox(T1,T2,T3,T4,T5,T6):
    D = 0
    for i in range(classified_num):
        if train_data[0][i] == 1:
            if train_data[4][i]:
                if T1 >  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            else:
                if T1 <  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h           
        elif train_data[0][i] == 2:
            if train_data[4][i]:
                if T2 >  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            else:
                if T2 <  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        elif train_data[0][i] == 3:
            if train_data[4][i]:
                if T3 >  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            else:
                if T3 <  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        elif train_data[0][i] == 4:
            if train_data[4][i]:
                if T4 >  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            else:
                if T4 <  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        elif train_data[0][i] == 5:
            if train_data[4][i]:
                if T5 >  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            else:
                if T5 <  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        elif train_data[0][i] == 6:
            if train_data[4][i]:
                if T6 >  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            else:
                if T6 <  train_data[2][i]: 
                    h = 1
                else:
                    h = -1
            ah = train_data[3][i]*h
        D = D + ah
    return D

def radius(xy_data,num_xy):
    A = np.ones((num_xy,3));   
    A[:,0] = xy_data[:,0]*(-2);
    A[:,1] = xy_data[:,1]*(-2);
    b = np.transpose(-np.sum(np.power(xy_data,2), axis=1))
    c_x,c_y,c_r = np.linalg.lstsq(A,b)[0]
    c_r = np.sqrt(c_x**2 + c_y**2 - c_r);
    return c_r

def circularity(xy_data,num_xy):
    A = np.ones((num_xy,3));   
    A[:,0] = xy_data[:,0]*(-2);
    A[:,1] = xy_data[:,1]*(-2);
    b = np.transpose(-np.sum(np.power(xy_data,2), axis=1))
    c_x,c_y,c_r = np.linalg.lstsq(A,b)[0]
    c_r = np.sqrt(c_x**2 + c_y**2 - c_r);
    diff_xy = xy_data.copy()
    diff_xy[:,0] = diff_xy[:,0] - np.transpose(np.ones((num_xy,1))*c_x)
    diff_xy[:,1] = diff_xy[:,1] - np.transpose(np.ones((num_xy,1))*c_y)
    Sc = np.sum(np.power((c_r*np.ones((num_xy,1)) - np.sqrt(np.sum( np.power(diff_xy,2), axis=1) )),2));
    return Sc

def boundary_std(xy_data,num_xy):
    tmp_X = np.ones((num_xy,1))
    tmp_Y = np.ones((num_xy,1))
    tmp_X[:,0] = xy_data[:,0]
    tmp_Y[:,0] = xy_data[:,1]
    tmp_X_boundary = np.ones((num_xy-1,1))
    tmp_Y_boundary = np.ones((num_xy-1,1))
    tmp_boundary = np.ones((num_xy-1,1))
    for i in range(0,num_xy-1):
        tmp_X_boundary[i,0] = (tmp_X[i+1,0] - tmp_X[i,0])**2
        tmp_Y_boundary[i,0] = (tmp_Y[i+1,0] - tmp_Y[i,0])**2
        tmp_boundary[i,0] = np.sqrt((tmp_X[i+1,0] - tmp_X[i,0])**2 + (tmp_Y[i+1,0] - tmp_X[i,0])**2)    
    BS = np.std(tmp_boundary)
    return(BS)

def boundary_length(xy_data,num_xy):
    tmp_X = np.ones((num_xy,1))
    tmp_Y = np.ones((num_xy,1))
    tmp_X[:,0] = xy_data[:,0]
    tmp_Y[:,0] = xy_data[:,1]
    tmp_X_boundary = np.ones((num_xy-1,1))
    tmp_Y_boundary = np.ones((num_xy-1,1))
    tmp_boundary = np.ones((num_xy-1,1))
    for i in range(0,num_xy-1):
        tmp_X_boundary[i,0] = (tmp_X[i+1,0] - tmp_X[i,0])**2
        tmp_Y_boundary[i,0] = (tmp_Y[i+1,0] - tmp_Y[i,0])**2
        tmp_boundary[i,0] = np.sqrt((tmp_X[i+1,0] - tmp_X[i,0])**2 + (tmp_Y[i+1,0] - tmp_X[i,0])**2)
    BL = np.sum(tmp_boundary)
    return(BL)
    
def Segment(xy_data):
    X = xy_data[:,0]
    Y = xy_data[:,1]
    S_i = 1
    S_n = 0
    threshold = 0.1
    tmp_n0ind = np.nonzero(xy_data)
    if tmp_n0ind[0].size:
        n0ind = np.array([tmp_n0ind[0][0]])
        for i in range(1,tmp_n0ind[0].size):
            if tmp_n0ind[0][i]>tmp_n0ind[0][i-1]:
                n0ind = np.concatenate([n0ind,np.array([tmp_n0ind[0][i]])])      
        n_0 = n0ind.size;
        xy = np.array([X[n0ind[0]] , Y[n0ind[0]]])
        Seg = []
        for i in range(1,n_0):
            if np.sqrt(( X[n0ind[i]] - X[n0ind[i-1]] )**2 + ( Y[n0ind[i]] - Y[n0ind[i-1]] )**2) < threshold:
                S_i += 1
                new_xy = np.array([X[n0ind[i]] , Y[n0ind[i]]])
                xy = np.vstack([xy,new_xy])
            else:
                if xy.size > 4:
                    S_n = S_n + 1;
                    Seg.append(Segment_xydata(xy,S_i))                   
                xy = np.array([X[n0ind[i]],Y[n0ind[i]]])
                S_i = 1;
        if xy.size > 4:
                S_n = S_n + 1;
                Seg.append(Segment_xydata(xy,S_i)) 
        display(Seg,S_n)

        
    
    

def callback(data):
    tmp = np.array([1000,1000])#改改看
    for i in range(360):
        rad = data.angle_min+data.angle_increment*i
        ranges = data.ranges
        xydata = np.array([math.cos(rad)*ranges[i],math.sin(rad)*ranges[i]])
        if i == 0:
            xy_data = xydata
        else:   
            xy_data = np.vstack([xy_data,xydata])
    Segment(xy_data)  
    


def listener():
    rospy.init_node('listener', anonymous=True)  
    rate = rospy.Rate(0.1) # 1hz
    while not rospy.is_shutdown():
        rospy.Subscriber("scan", LaserScan, callback)
        rate.sleep()     
    #rospy.spin()
    #f.close

if __name__ == '__main__':
    listener()
