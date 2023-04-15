from datetime import time
from re import L
import numpy as np
import time
import rospy
import math
import csv
from ackermann_msgs.msg import AckermannDriveStamped
from gazebo_msgs.msg import ModelStates
from eufs_msgs.msg import WheelSpeedsStamped
rospy.init_node("stanley")
rate = rospy.Rate(10)
pub = rospy.Publisher("/cmd_vel_out", AckermannDriveStamped, queue_size=10)



class params:
    Kp= 0.3
    k=0.5
    dt=0.1
    dref=0.5
    L = 1.530 #wheelbase 
    max_steer = 1

class Node(object):
    def callback(self,msg):
        self.x= msg.pose[2].position.x
        self.y = msg.pose[2].position.y
        #print("x",self.x)
        #print("y",self.y)
    def callback2(self,msg):
        self.wheelspeed = msg.lb_speed
        self.steering_angle = msg.steering



    def __init__(self,x,y,v,desired_v,yaw):
        self.x = 0
        self.y=0
        self.v = 0
        self.desired_v = 1
        self.yaw=0
        self.wheelspeed=0
        self.steering_angle = 0
        self.sub = rospy.Subscriber("/gazebo/model_states",ModelStates,self.callback)
        self.sub2 =  rospy.Subscriber("/ros_can/wheel_speeds",WheelSpeedsStamped,self.callback2)

        

    def update(self,delta,a):
        delta = delta
        print(delta)
        self.x += self.v *math.cos(self.yaw)*params.dt
        self.y += self.v *math.sin(self.yaw)*params.dt
        self.yaw += self.v / params.L*math.tan(delta)*params.dt
        self.v = self.wheelspeed*math.pi*0.505*60/63360

class Trajectory:
    def __init__(self, cx, cy, cyaw):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ind_old = 0 
    def calc_theta_e_and_ef(self, node):
        """
        calc theta_e and ef.
        theta_e = theta_car - theta_path
        ef = lateral distance in frenet frame (front wheel)
        :param node: current information of vehicle
        :return: theta_e and ef
        """
        
        fx = node.x + params.L * math.cos(node.yaw)
        fy = node.y + params.L * math.sin(node.yaw)
        #print("fx  ",fx)
        #print("fy  ",fy)

        dx = [fx - x for x in self.cx]
        dy = [fy - y for y in self.cy]
        #print("dx",dx)
        #print("length of dx",len(dx))
        #print(dy)

        target_index = int(np.argmin(np.hypot(dx, dy)))
        target_index = max(self.ind_old, target_index)
        self.ind_old = max(self.ind_old, target_index)
        #print(target_index)

        front_axle_vec_rot_90 = np.array([[math.cos(node.yaw - math.pi / 2.0)],
                                          [math.sin(node.yaw - math.pi / 2.0)]])

        vec_target_2_front = np.array([[dx[target_index]],
                                       [dy[target_index]]])

        ef = np.dot(vec_target_2_front.T, front_axle_vec_rot_90)

        theta = node.steering_angle
        print(theta)
        theta_p = self.cyaw[target_index]
        theta_e = pi_2_pi(theta_p - theta)
        # #print("fx",fx)
        # #print("fy",fy)
        # #print("theta_e",theta_e)
        # #print("theta_p",theta_p)
        # #print("x",node.x)

        return theta_e, ef, target_index

def front_wheel_feedback_control(node, ref_path):
    """
    front wheel feedback controller
    :param node: current information
    :param ref_path: reference path: x, y, yaw
    :return: optimal steering angle
    """

    theta_e, ef, target_index = ref_path.calc_theta_e_and_ef(node)
    #print("velocity is",node.v)
    delta = theta_e + math.atan2(params.k * ef, node.v)
    # #print("delta",delta)
    # #print("angle",math.atan2(params.k * ef, node.v))
    # #print("expected delta",theta_e + math.atan2(params.k * ef, node.v))
    # #print("thetae",theta_e)


    return delta, target_index


def pi_2_pi(angle):
    if angle > math.pi:
        return angle - 2.0 * math.pi
    if angle < -math.pi:
        return angle + 2.0 * math.pi

    return angle

def pid_control(target_v, v):
    #print(v)
    a = params.Kp * (target_v - v)
    return a

def main():
    cx=[]
    cy=[]

    with open("/home/soham/eufs_sim_ws/src/eufs_sim/eufs_gazebo/tracks/acceleration_CT.csv","r") as waypoints:
        file_read = csv.reader(waypoints)
        for row in file_read:
            cx.append(float(row[0]))
            cy.append(float(row[1]))
    #print(cx)
    #print(cy)

    cyaw=cx
    for i in range(len(cx)-1):
        cyaw[i] = np.arctan2(cy[i+1]-cy[i], cx[i+1]-cx[i])
    #print(cyaw)

    t = 0.0
    x0, y0, yaw0 = cx[0], cy[0], cyaw[0]
    node = Node(x=x0, y=y0,v=0.0,desired_v=1,yaw=yaw0)
    ref_path = Trajectory(cx, cy, cyaw)

    rate = rospy.Rate(10)
    n=0
    while not rospy.is_shutdown():
        n+=1
        print("n",n)
        #print("entered while loop")
        speed_ref = 5 #m/s
        di, target_index = front_wheel_feedback_control(node, ref_path)
        if di > params.max_steer:
            di = params.max_steer

        elif di < -params.max_steer:
            di = -params.max_steer
        else:
            di = di
        #print("target index is ",target_index)
        ai = pid_control(speed_ref, node.v)
        print(node.v)
        #print("ai before update",ai)
        node.update(di,ai)
        t += params.dt

        print("ai ",ai)
        #print("di",di)
        output = AckermannDriveStamped()
        output.header.stamp = rospy.Time.now()
        output.drive.steering_angle = di
        if node.x<25.0:
            output.drive.acceleration = ai
        else:
            output.drive.acceleration = -7
        pub.publish(output)
        print(di)
        print("x",node.x)
        print("y",node.y)
        print()
        print("ti",target_index)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
