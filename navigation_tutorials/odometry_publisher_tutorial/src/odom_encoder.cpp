#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <math.h>

float x ;
float y ;
float th ;

float vx = 0.0;
float vy = 0.0;
float vth = 0.0;
  
float encoder1vel;
float encoder2vel;
float encoder3vel;
float encoder_velx;
float encoder_vely;
float encoder_th;

float a_vel;
float b_vel;
float c_vel;

int a,b,c; //encoder轉向
ros::Time current_time, last_time;

class mini_position{
public:

     ros::NodeHandle n;
     ros::Publisher odom_pub;
     ros::Subscriber pose_msg;
     ros::Subscriber encoder1;
     ros::Subscriber encoder2;
     ros::Subscriber encoder3;
     ros::Subscriber vel_1;
     ros::Subscriber vel_2;
     ros::Subscriber vel_3;
     tf::TransformBroadcaster odom_broadcaster;

     void PosCB(const geometry_msgs::Pose2D::ConstPtr& msg);
     void vel_A(const std_msgs::Float64::ConstPtr& msg);
     void vel_B(const std_msgs::Float64::ConstPtr& msg);
     void vel_C(const std_msgs::Float64::ConstPtr& msg);
     void encoder1_vel(const std_msgs::Float64::ConstPtr& msg);
     void encoder2_vel(const std_msgs::Float64::ConstPtr& msg);
     void encoder3_vel(const std_msgs::Float64::ConstPtr& msg);


};

void mini_position::PosCB(const geometry_msgs::Pose2D::ConstPtr& msg){

   mini_position pose;
   current_time = ros::Time::now();
   double dt = (current_time - last_time).toSec();
   tf::TransformBroadcaster odom_broadcaster;

   //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link"; // URDF 則須改成base_footprint
    
    encoder_velx=-1*encoder1vel*cos(60) + encoder2vel * cos(60)/2;
    encoder_vely= encoder1vel*sin(60)+encoder2vel*sin(60)-encoder3vel/3;
    encoder_th=encoder3vel+encoder2vel+encoder1vel/0.1545;
    vx=(encoder_velx*cos(th)-encoder_vely*sin(th))*dt;
    vy=(encoder_velx*sin(th)+encoder_vely*cos(th))*dt;
    vth=encoder_th*dt;

   
    x += vx;
    y += vy;
    th += vth;


    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link"; // URDF 則須改成base_footprint

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;

}

void mini_position::vel_A(const std_msgs::Float64::ConstPtr& msg){
   a_vel=msg->data;

   if(a_vel<0){
      a=-1;
    }
   else {
      a=1;
    }
}

void mini_position::vel_B(const std_msgs::Float64::ConstPtr& msg){

   b_vel=msg->data;

   if(b_vel<0){
      b=-1;
    }
   else {
      b=1;
    }
}

void mini_position::vel_C(const std_msgs::Float64::ConstPtr& msg){

   c_vel=msg->data;

   if(c_vel<0){
      c=-1;
    }
   else {
      c=1;
    }
}

void mini_position::encoder1_vel(const std_msgs::Float64::ConstPtr& msg){
    
     encoder1vel=msg->data*0.01*a;

}

void mini_position::encoder2_vel(const std_msgs::Float64::ConstPtr& msg){
    
     encoder2vel=msg->data*0.01*b;

}
void mini_position::encoder3_vel(const std_msgs::Float64::ConstPtr& msg){
    
     encoder3vel=msg->data*0.01*c;

}

int main(int argc, char** argv){
   
   ros::init(argc,argv,"odom_vel");
   mini_position pose;
   
   current_time = ros::Time::now();
   pose.odom_pub = pose.n.advertise<nav_msgs::Odometry>("odom",50);
   pose.vel_1=pose.n.subscribe("vel_A", 1000, &mini_position::vel_A, &pose);
   pose.vel_2=pose.n.subscribe("vel_B", 1000, &mini_position::vel_B, &pose);
   pose.vel_3=pose.n.subscribe("vel_C", 1000, &mini_position::vel_C, &pose);
   pose.encoder1=pose.n.subscribe("encoder1_value", 1000, &mini_position::encoder1_vel, &pose);
   pose.encoder2=pose.n.subscribe("encoder2_value", 1000, &mini_position::encoder2_vel, &pose);
   pose.encoder3=pose.n.subscribe("encoder3_value", 1000, &mini_position::encoder3_vel, &pose);
   
   pose.pose_msg = pose.n.subscribe("pose2D", 1000, &mini_position::PosCB, &pose);
   ros::spin();
   
}
