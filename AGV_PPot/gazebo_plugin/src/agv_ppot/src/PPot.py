#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
import rospy
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
import numpy as np
import math
import sympy as sym
from sympy import *
from std_msgs.msg._Float64MultiArray import Float64MultiArray
#########################################################################################################


#########################################################################################################

#######################################################################
#Initialize ROS Node
rospy.init_node('Path_Planning_APF', anonymous=True) #Identify ROS Node
#######################################################################
APF_Param = [rospy.get_param("~K_att"),rospy.get_param("~K_rep")] #Reinforcement attraction, repulsion
tau = rospy.get_param("~sampling_time") #Sampling Time
rob_mass = rospy.get_param("~rob_mass") #Robot Mass (Turtlebot 3 Waffle_pi)

Rob_rate = 10
#######################################################################
#ROS Publisher Code for Velocity
pub1 = rospy.Publisher('/APF_Des_Pos', Pose, queue_size=10) #Identify the publisher "pub1" to publish on topic "/APF_Des_Pos" to send message of type "Pose"
Des_Pos_msg = Pose() #Identify msg variable of data type Twist
rate = rospy.Rate(Rob_rate) # rate of publishing msg 20hz
Obs_Pos_x = [ -7.5,-5.5,-3.5,-1.5,  5.0,5.0,  -8.5, -7.5, -5.5, -4.5, -3.5, -1.5, -1.5, 2.5, 4.5, 3.5, 4.5, 7.5 ]
Obs_Pos_y = [ -6.0,-6.0,-6.0,-6.0,  -7.5,-4.5,  1.5, 7.5, 4.5, 7.5, 1.5, 2.5, 6.5, 0.5, 6.5, 1.5, 2.5, 4.5 ]
Obs_len_x = [ 0.5,0.5,0.5,0.5,  3.0, 3.0,  0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5 ]
Obs_len_y = [ 2.0,2.0,2.0,2.0,  0.5,0.5,  0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5 ]
dsafe=0.4 # Safe distance

#######################################################################

#######################################################################
def euler_to_quaternion(yaw, pitch, roll):
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [x, y, z, w]
#######################################################################

#######################################################################
def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]
#######################################################################

#######################################################################
#ROS Subscriber Code for Position
flag_cont = 0	#Initialize flag by zero
pos_msg = Pose()	#Identify msg variable of data type Pose
position = np.zeros((1,6))
Velocity_msg = Twist()
velocity = np.zeros((1,6))
#######################################################################
#######################################################################
##Callback function for feedback the vehicle current position
#Callback function which is called when a new message of type Pose is received by the subscriber
def callback(data):
    global pos_msg	#Identify msg variable created as global variable
    global sub2		#Identify a subscriber as global variable
    global flag_cont
    global position 
    global Velocity_msg
    global velocity
    
    msg = data
    pos_msg.position.x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
    pos_msg.position.y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
    pos_msg.position.z = round(msg.pose.pose.position.z, 4)		#Round the value of y to 4 decimal places
    pos_msg.orientation.x = round(msg.pose.pose.orientation.x, 4)	#Round the value of theta to 4 decimal places
    pos_msg.orientation.y = round(msg.pose.pose.orientation.y, 4)	#Round the value of theta to 4 decimal places
    pos_msg.orientation.z = round(msg.pose.pose.orientation.z, 4)	#Round the value of theta to 4 decimal places
    pos_msg.orientation.w = round(msg.pose.pose.orientation.w, 4)	#Round the value of theta to 4 decimal places
    [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)
    position = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]
    Velocity_msg.linear.x = round(msg.twist.twist.linear.x, 4)
    Velocity_msg.linear.y = round(msg.twist.twist.linear.y, 4)
    Velocity_msg.linear.z = round(msg.twist.twist.linear.z, 4)
    Velocity_msg.angular.x = round(msg.twist.twist.angular.x, 4)
    Velocity_msg.angular.y = round(msg.twist.twist.angular.y, 4)
    Velocity_msg.angular.z = round(msg.twist.twist.angular.z, 4)
    velocity = [Velocity_msg.linear.x,Velocity_msg.linear.y,Velocity_msg.linear.z,Velocity_msg.angular.x,Velocity_msg.angular.y,Velocity_msg.angular.z]
    flag_cont = 1

sub2 = rospy.Subscriber('/odom', Odometry, callback) #Identify the subscriber "sub2" to subscribe topic "/odom" of type "Odometry"
#######################################################################

#######################################################################
#ROS Subscriber Code for Initial Position
pos_msg_0 = Pose()	#Identify msg variable of data type Pose
position_0 = np.zeros((1,6))
flag_initial = 0
Velocity_msg_0 = Twist()
velocity_0 = np.zeros((1,6))
#######################################################################
#######################################################################
#Initial callback function for setting the vehicle initial position
#Callback function which is called when a new message of type Pose is received by the subscriber 
def callback_Init(data):
    global pos_msg_0		#Identify msg variable created as global variable
    global sub1			#Identify a subscriber as global variable
    global flag_initial 	#Identify flag created as global variable
    global position_0 
    global Velocity_msg_0
    global velocity_0
    
    msg = data
    pos_msg_0.position.x = round(msg.pose.pose.position.x, 4)		#Round the value of x to 4 decimal places
    pos_msg_0.position.y = round(msg.pose.pose.position.y, 4)		#Round the value of y to 4 decimal places
    pos_msg_0.position.z = round(msg.pose.pose.position.z, 4)		#Round the value of y to 4 decimal places
    pos_msg_0.orientation.x = round(msg.pose.pose.orientation.x, 4)	#Round the value of theta to 4 decimal places
    pos_msg_0.orientation.y = round(msg.pose.pose.orientation.y, 4)	#Round the value of theta to 4 decimal places
    pos_msg_0.orientation.z = round(msg.pose.pose.orientation.z, 4)	#Round the value of theta to 4 decimal places
    pos_msg_0.orientation.w = round(msg.pose.pose.orientation.w, 4)	#Round the value of theta to 4 decimal places
    [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w)
    position_0 = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll]
    Velocity_msg_0.linear.x = round(msg.twist.twist.linear.x, 4)
    Velocity_msg_0.linear.y = round(msg.twist.twist.linear.y, 4)
    Velocity_msg_0.linear.z = round(msg.twist.twist.linear.z, 4)
    Velocity_msg_0.angular.x = round(msg.twist.twist.angular.x, 4)
    Velocity_msg_0.angular.y = round(msg.twist.twist.angular.y, 4)
    Velocity_msg_0.angular.z = round(msg.twist.twist.angular.z, 4)
    velocity_0 = [Velocity_msg_0.linear.x,Velocity_msg_0.linear.y,Velocity_msg_0.linear.z,Velocity_msg_0.angular.x,Velocity_msg_0.angular.y,Velocity_msg_0.angular.z]
    flag_initial = 1
    sub1.unregister()				#Unsubsribe from this topic

sub1 = rospy.Subscriber('/odom', Odometry, callback_Init) #Identify the subscriber "sub1" to subscribe topic "/odom" of type "Odometry"
#######################################################################
#######################################################################
##Stop code here till subscribe the first msg of the vehicle position
while flag_initial == 0:
    pass
#######################################################################

#######################################################################
#Define the initial pose and velocity of the vehicle
Rob_pos_0 = [position_0[0],position_0[1],position_0[3]]
Roc_vel_0 = [velocity_0[0],velocity_0[5]]

x_p = Rob_pos_0[0]
y_p = Rob_pos_0[1]
vel_p_x = Roc_vel_0[0]*cos(Rob_pos_0[2])
vel_p_y = Roc_vel_0[0]*sin(Rob_pos_0[2])
#######################################################################
#########################################################################################################

#########################################################################################################
#######################################################################
#APF Inputs - Goal function
goal_data = [0.0,0.0]
def get_Goal(data):
    global goal_data
    global sub_goal
    goal_data = data.data
    print(data.data)
       
sub_goal=rospy.Subscriber('goal', Float64MultiArray, get_Goal)      #Identify the subscriber "sub1" to subscribe topic "/odom" of type "Odometry"
Goal_Pos = goal_data
#Goal_Pos = [rospy.get_param("~x_Goal"),rospy.get_param("~y_Goal")]
#Obs_Pos = [rospy.get_param("~x_Obs1"),rospy.get_param("~y_Obs1"),rospy.get_param("~x_Obs2"),rospy.get_param("~y_Obs2"),rospy.get_param("~x_Obs3"),rospy.get_param("~y_Obs3"),rospy.get_param("~x_Obs_dash"),rospy.get_param("~y_Obs_dash"),rospy.get_param("~x_Obs4"),rospy.get_param("~y_Obs4"), rospy.get_param("~x_Obs5"),rospy.get_param("~y_Obs5"), rospy.get_param("~x_Obs6"),rospy.get_param("~y_Obs6"), rospy.get_param("~x_Obs7"),rospy.get_param("~y_Obs7"), rospy.get_param("~x_Obs8"),rospy.get_param("~y_Obs8")]
'''Obs_Pos = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
dsafe=0.4
APF_Param = [rospy.get_param("~K_att"),rospy.get_param("~K_rep"),0,0,0,0,0,0,0,0,0]'''
#APF_Param = [rospy.get_param("~K_att"),rospy.get_param("~K_rep"),rospy.get_param("~q_star1") + dsafe, rospy.get_param("~q_star2")+dsafe,rospy.get_param("~q_star3")+dsafe,rospy.get_param("~q_star_dash")+ dsafe,rospy.get_param("~q_star4")+dsafe,rospy.get_param("~q_star5")+dsafe,rospy.get_param("~q_star6")+dsafe,rospy.get_param("~q_star7")+dsafe,rospy.get_param("~q_star8")+ dsafe] #[K_att,K_rep,q_star]
#APF_Param = [rospy.get_param("~K_att"),rospy.get_param("~K_rep"),0,0,0,0,0,0,0,0,0]
#######################################################################
#######################################################################
#APF Equations
x_rob = symbols('x_rob')
y_rob = symbols('y_rob')
x_goal = symbols('x_goal')
y_goal = symbols('y_goal')
x_obs = symbols('x_obs')
y_obs = symbols('y_obs')
obs_len_x = symbols('obs_len_x')
obs_len_y = symbols('obs_len_y')
D_obs = symbols('D_obs')

#Attraction Forces Equations
#Fx_att = -APF_Param[0]*(x_rob-x_goal)
#Fy_att = -APF_Param[0]*(y_rob-y_goal)
#Repulsion Forces Equations
#d_obs = sqrt((x_rob-x_obs)**2 + (y_rob-y_obs)**2)
#d_obs_x = sqrt((x_rob-x_obs)**2)
#d_obs_y = sqrt((y_rob-y_obs)**2)

#Fx_rep = -APF_Param[1]*((1/d_obs)-(1/((1/obs_len_x)+dsafe)))*(-(x_rob-x_obs)/(d_obs**3))
#Fy_rep = -APF_Param[1]*((1/d_obs)-(1/((1/obs_len_y)+dsafe)))*(-(y_rob-y_obs)/(d_obs**3))
#Fx_rep = -APF_Param[1]*(1-(D_obs/(obs_len_x+dsafe)))*(-(x_rob-x_obs)/(D_obs**3))
#Fy_rep = -APF_Param[1]*(1-(D_obs/(obs_len_y+dsafe)))*(-(y_rob-y_obs)/(D_obs**3))
#Fx_rep = -APF_Param[1]*((1/d_obs_x)-(1/obs_len_x))*(-(x_rob-x_obs)/(d_obs_x**3))
#Fy_rep = -APF_Param[1]*((1/d_obs_y)-(1/obs_len_y))*(-(y_rob-y_obs)/(d_obs_y**3))
#######################################################################
def APF_Fn(Rob_pos,Goal_pos,Obs_pos_x,Obs_pos_y,APF_Param,dsafe,Obs_len_x,Obs_len_y):
    #global Fx_att
    #global Fy_att
    #global d_obs
    #global d_obs_x
    #global d_obs_y
    #global Fx_rep
    #global Fy_rep
    #global dsafe
    #global Obs_len_x
    #global Obs_len_y
    
    Fx_att_val = -APF_Param[0]*(Rob_pos[0]-Goal_pos[0])
    Fy_att_val = -APF_Param[0]*(Rob_pos[1]-Goal_pos[1])
    #Fx_att.subs([(x_rob,Rob_pos[0]),(x_goal,Goal_pos[0])])
    #Fy_att_val = Fy_att.subs([(y_rob,Rob_pos[1]),(y_goal,Goal_pos[1])])
    Fx_rep_val=0.0
    Fy_rep_val=0.0
    j=0
    flaga = 0
    while j<18:
        #d_obs_val = d_obs.subs([(x_rob,Rob_pos[0]),(x_obs,Obs_pos_x[j]),(y_rob,Rob_pos[1]),(y_obs,Obs_pos_y[j])])
        d_obs_val = sqrt((Rob_pos[0]-Obs_pos_x[j]-Obs_len_x[j])**2+(Rob_pos[1]-Obs_pos_y[j]-Obs_len_y[j])**2)
        d_obs_val_x = sqrt((Rob_pos[0]-Obs_pos_x[j])**2)
        d_obs_val_y = sqrt((Rob_pos[1]-Obs_pos_y[j])**2)
        
        #obs_r = sqrt(Obs_len_x[j]**2 + Obs_len_y[j]**2)
        
        #if d_obs_val < (obs_r + dsafe):
        if ((d_obs_val_x < (Obs_len_x[j] + dsafe)) and (d_obs_val_y < (Obs_len_y[j] + dsafe))):
            flaga = 1
            Fx_rep_val += -APF_Param[1]*(1-(d_obs_val/dsafe))*((Rob_pos[0]-Obs_pos_x[j]-Obs_len_x[j])/(d_obs_val**3))
            Fy_rep_val += -APF_Param[1]*(1-(d_obs_val/dsafe))*((Rob_pos[1]-Obs_pos_y[j]-Obs_len_y[j])/(d_obs_val**3))
            #Fx_rep_val += -(Obs_len_y[j]/Obs_len_x[j])*APF_Param[1]*(1-(d_obs_val/(Obs_len_x[j]+dsafe)))*((Rob_pos[0]-Obs_pos_x[j])/(d_obs_val**3))
            #Fy_rep_val += -(Obs_len_x[j]/Obs_len_y[j])*APF_Param[1]*(1-(d_obs_val/(Obs_len_y[j]+dsafe)))*((Rob_pos[1]-Obs_pos_y[j])/(d_obs_val**3))
            #Fx_rep_val += -APF_Param[1]*(1-(d_obs_val/(Obs_len_x[j]+dsafe)))*(-(Rob_pos[0]-Obs_pos_x[j])/(d_obs_val**3))
            #Fy_rep_val += -APF_Param[1]*(1-(d_obs_val/(Obs_len_y[j]+dsafe)))*(-(Rob_pos[1]-Obs_pos_y[j])/(d_obs_val**3))
            #Fx_rep_val += Fx_rep.subs([(x_rob,Rob_pos[0]),(x_obs,Obs_pos_x[j]),(y_rob,Rob_pos[1]),(y_obs,Obs_pos_y[j]),(D_obs,d_obs_val),(obs_len_x,Obs_len_x[j])])
            #Fy_rep_val += Fy_rep.subs([(x_rob,Rob_pos[0]),(x_obs,Obs_pos_x[j]),(y_rob,Rob_pos[1]),(y_obs,Obs_pos_y[j]),(D_obs,d_obs_val),(obs_len_y,Obs_len_y[j])])
            #Fx_rep_val += Fx_rep.subs([(x_rob,Rob_pos[0]),(x_obs,Obs_pos_x[j]),(d_obs_x,d_obs_val_x),(obs_len_x,Obs_len_x[j])])
            #Fy_rep_val += Fy_rep.subs([(y_rob,Rob_pos[1]),(y_obs,Obs_pos_y[j]),(d_obs_y,d_obs_val_y),(obs_len_y,Obs_len_y[j])])
        else:
            Fx_rep_val += 0.0
            Fy_rep_val += 0.0
          
        j +=1
    
    if flaga == 1:
        Fx_att_val = Fx_att_val/1.5
        Fy_att_val = Fy_att_val/1.5
        Fx_net_val = (Fx_att_val + Fx_rep_val)/4
        Fy_net_val = (Fy_att_val + Fy_rep_val)/4
    else:
        Fx_net_val = Fx_att_val + Fx_rep_val
        Fy_net_val = Fy_att_val + Fy_rep_val
        
    F_xy_net = [Fx_net_val,Fy_net_val]
    return F_xy_net
#######################################################################
#########################################################################################################

#########################################################################################################
#Simulation While Loop

while 1 and not rospy.is_shutdown():
    if flag_cont == 1:
        
        sub_goal = rospy.Subscriber('goal', Float64MultiArray, get_Goal)      #Identify the subscriber "sub1" to subscribe topic "/odom" of type "Odometry"
        Goal_Pos = goal_data
        #Get Robot Current Position and Velocity
        Rob_pos = [position[0],position[1],position[3]]
        Rob_vel = [velocity[0],velocity[5]]
        
        #Implement Artificial Potential Field
        F_xy_net = APF_Fn(Rob_pos,Goal_Pos,Obs_Pos_x,Obs_Pos_y,APF_Param,dsafe,Obs_len_x,Obs_len_y)
        F_net = float(sqrt(F_xy_net[0]**2 + F_xy_net[1]**2))
        F_net_direct = float(atan2(F_xy_net[1], F_xy_net[0]))
        
        #Calculate the desired robot position from the APF
        vel_c_x = vel_p_x + (F_xy_net[0]/rob_mass)*tau
        vel_c_y = vel_p_y + (F_xy_net[1]/rob_mass)*tau
        x_des = x_p + vel_c_x*tau
        y_des = y_p + vel_c_y*tau
        theta_des = F_net_direct
        Rob_pos_des = [x_des,y_des,theta_des]
        
        #Update the previous robot states for the next iteration
        vel_p_x = Rob_vel[0]*cos(Rob_pos[2])
        vel_p_y = Rob_vel[0]*sin(Rob_pos[2])
        x_p = Rob_pos[0]
        y_p = Rob_pos[1]
        flag_cont = 0

    else:
        Rob_pos_des = Rob_pos
        
    Des_Pos_msg.position.x = Rob_pos_des[0]
    Des_Pos_msg.position.y = Rob_pos_des[1]
    Des_Pos_msg.position.z = 0
    [qx_des, qy_des, qz_des, qw_des] = euler_to_quaternion(Rob_pos_des[2], 0, 0)
    Des_Pos_msg.orientation.x = qx_des
    Des_Pos_msg.orientation.y = qy_des
    Des_Pos_msg.orientation.z = qz_des
    Des_Pos_msg.orientation.w = qw_des

    pub1.publish(Des_Pos_msg)	#Publish msg
    rate.sleep()		#Sleep with rate
#########################################################################################################
