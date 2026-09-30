import rospy
from sensor_msgs.msg import LaserScan
import os
import math
import numpy as np
import copy

f = open('ball.txt')
DEF = np.loadtxt(f)
classified_num = np.size(DEF,1)
f.close

sensor_m = np.array([[0.9657,0.0343],[0.0354,0.9646]])
motion_m = np.transpose(np.array([[0.9,0.1],[0.3,0.7]]))

class Segment_xydata:
    def __init__(self, xy,num):
        self.xydata = xy
        self.num_xy = num
        self.T1 = num
        self.T2 = np.sqrt(np.sum(np.power(np.std(xy, axis=0),2)))
        self.T3 = np.linalg.norm(np.mean(xy,axis = 0))
        self.T4 = circularity(xy,num,0)
        self.T5 = circularity(xy,num,1)

    def get_xydata(self):
        return self.xydata.copy()


def Basefilter(Z,num):
    x0 = np.array([[0.5],[0.5]])
    fv = np.zeros((num,2));
    b = np.array([1,1])
    smooth = np.zeros((num,2))
    for i in range(num):
        P = motion_m.dot(x0)
        if Z[i]:
            x0 = (sensor_m[:,0]*np.transpose(P))/np.sum((sensor_m[:,0]*np.transpose(P)));
        else:
            x0 = (sensor_m[:,1]*np.transpose(P))/np.sum((sensor_m[:,1]*np.transpose(P)));
        fv[i,:] = x0
        x0 = np.transpose(x0);
        #print(x0)  
    for j in range(num-1,-1,-1):
        smooth[j,:] = fv[j,:]*b/sum(fv[j,:]*b);
        if Z[j]:
            tmp = np.transpose(sensor_m[:,0]*b)
            b = motion_m.dot(tmp)
        else:
            tmp = np.transpose(sensor_m[:,1]*b)
            b = motion_m.dot(tmp)
    return smooth

def display(Seg,S_n):
    Z = np.zeros(S_n)
    for i in range(S_n):
        #print(i)
        if isbox(Seg[i].T1,Seg[i].T2,Seg[i].T3,Seg[i].T4,Seg[i].T5)>0:
            Z[i] = 1;
            #print(i)
            #print(np.mean(Seg[i].xydata,axis = 0))
    SM = Basefilter(Z,S_n)
    for j in range(S_n):
        if Z[j] == 1:
            print(j)
            print(np.mean(Seg[j].xydata,axis = 0),SM[j,0],'ball!')




def isbox(T1,T2,T3,T4,T5):
    D = 0
    for i in range(classified_num):
        if DEF[0][i] == 1:
            if DEF[4][i]:       
                if T1 >  DEF[2][i]: 
                    h = 1
                else:
                    h = -1
            else:
                if T1 <  DEF[2][i]: 
                    h = 1
                else:
                    h = -1
            ah = DEF[3][i]*h            
        elif DEF[0][i] == 2:
            if DEF[4][i]:
                if T2 >  DEF[2][i]: 
                    h = 1
                else:
                    h = -1
            else:
                if T2 <  DEF[2][i]: 
                    h = 1
                else:
                    h = -1
            ah = DEF[3][i]*h
        elif DEF[0][i] == 3:
            if DEF[4][i]:
                if T3 >  DEF[2][i]: 
                    h = 1
                else:
                    h = -1
            else:
                if T3 <  DEF[2][i]: 
                    h = 1
                else:
                    h = -1
            ah = DEF[3][i]*h
        elif DEF[0][i] == 4:
            if DEF[4][i]:
                if T4 >  DEF[2][i]: 
                    h = 1
                else:
                    h = -1
            else:
                if T4 <  DEF[2][i]: 
                    h = 1
                else:
                    h = -1
            ah = DEF[3][i]*h
        elif DEF[0][i] == 5:
            if DEF[4][i]:
                if T5 >  DEF[2][i]: 
                    h = 1
                else:
                    h = -1
            else:
                if T5 <  DEF[2][i]: 
                    h = 1
                else:
                    h = -1
            ah = DEF[3][i]*h
        D = D + ah
    return D

def linearity(xy_data,num_xy):
    tmp_A = np.ones((num_xy,2));
    tmp_b = np.ones((num_xy,1))
    tmp_A[:,0] = xy_data[:,0]
    tmp_b = xy_data[:,1]
    a,b = np.linalg.lstsq(tmp_A,tmp_b)[0]
    diff_xy = np.linalg.norm(a*xy_data[:,0] + b*np.ones(num_xy) - xy_data[:,1])
    return diff_xy

def circularity(xy_data,num_xy,D):
    tmp_A = np.ones((num_xy,3));   
    tmp_A[:,0] = xy_data[:,0]*(-2);
    tmp_A[:,1] = xy_data[:,1]*(-2);
    tmp_b = np.transpose(-np.sum(np.power(xy_data,2), axis=1))
    c_x,c_y,c_r = np.linalg.lstsq(tmp_A,tmp_b)[0]
    c_r = np.sqrt(c_x**2 + c_y**2 - c_r);
    if D:
        return c_r
    else :
        diff_xy = xy_data.copy()
        diff_xy[:,0] = diff_xy[:,0] - np.transpose(np.ones((num_xy,1))*c_x)
        diff_xy[:,1] = diff_xy[:,1] - np.transpose(np.ones((num_xy,1))*c_y)
        Sc = np.sum(np.power((c_r*np.ones((num_xy,1)) - np.sqrt(np.sum( np.power(diff_xy,2), axis=1) )),2));
        return Sc
    


def Segment(xydata):
    x = xydata[:,0]
    y = xydata[:,1]
    threshold= 0.1;
    S_i = 1;#size of segment
    S_n = 0;#number of segment
    i = 0
    tmp_n0ind = np.nonzero(xydata)
    if tmp_n0ind[0].size:
        n0ind = np.array([tmp_n0ind[0][0]])
        for i in range(1,tmp_n0ind[0].size):
            if tmp_n0ind[0][i]>tmp_n0ind[0][i-1]:
                n0ind = np.concatenate([n0ind,np.array([tmp_n0ind[0][i]])])      
        n_0 = n0ind.size;
        tmp = np.array([x[n0ind[0]],y[n0ind[0]]])
        Seg = [];
        for i in range(1,n_0):
            if np.sqrt(( x[n0ind[i]] - x[n0ind[i-1]] )**2 + ( y[n0ind[i]] - y[n0ind[i-1]] )**2) < threshold:
                S_i = S_i + 1;
                tmp_xy = np.array([x[n0ind[i]],y[n0ind[i]]])
                tmp = np.vstack([tmp,tmp_xy])
            else:
                if tmp.size > 4:
                    S_n = S_n + 1;
                    Seg.append(Segment_xydata(tmp,S_i))                   
                tmp = np.array([x[n0ind[i]],y[n0ind[i]]])
                S_i = 1;
        if tmp.size > 4:
                S_n = S_n + 1;
                Seg.append(Segment_xydata(tmp,S_i)) 
        if np.linalg.norm(Seg[0].xydata[0] -  Seg[S_n-1].xydata[Seg[S_n-1].num_xy-1])< threshold:
            S_n = S_n - 1
            combine_xy = np.vstack([Seg[S_n].get_xydata(),Seg[0].get_xydata()])
            #print(Seg[0].get_xydata())
            #print(Seg[S_n].get_xydata())
            #print(combine_xy)
            conbine_num = Seg[0].num_xy + Seg[S_n].num_xy
            Seg[0] = Segment_xydata(combine_xy,conbine_num)
            del Seg[S_n]
        display(Seg,S_n)

    

def callback(data):
    tmp = np.array([1000,1000])
    for i in range(360):
        rad = data.angle_min+data.angle_increment*i
        ranges = data.ranges
        #print(rad,ranges[i])
        lines = [str(rad),'    ',str(ranges[i]),'\n']
        xydata = np.array([math.cos(rad)*ranges[i],math.sin(rad)*ranges[i]])

        if i == 0:
            tmp = xydata
        else:   
            tmp = np.vstack([tmp,xydata])
        #f.writelines(lines)
    #print(tmp[0])
    #print(tmp[359])
        
    Segment(tmp)  
    


def listener():
    rospy.init_node('listener', anonymous=True)  
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        rospy.Subscriber("scan", LaserScan, callback)
        rate.sleep()     
    #rospy.spin()
    #f.close

if __name__ == '__main__':
    listener()
