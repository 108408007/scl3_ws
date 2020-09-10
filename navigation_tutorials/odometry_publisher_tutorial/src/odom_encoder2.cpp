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

int main(int argc,char** argv){

  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

   ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(1.0); // 1HZ 發布里程計數值

  while(n.ok()){
  

  current_time = ros::Time::now();
  encoder_velx=-1*encoder1vel*cos(60) + encoder2vel * cos(60)/2;
    encoder_vely= encoder1vel*sin(60)+encoder2vel*sin(60)-encoder3vel/3;
    encoder_th=encoder3vel+encoder2vel+encoder1vel/0.1545;
    vx=(encoder_velx*cos(th)-encoder_vely*sin(th))*dt;
    vy=(encoder_velx*sin(th)+encoder_vely*cos(th))*dt;
    vth=encoder_th*dt;

   
    x += vx;
    y += vy;
    th += vth;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

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
    odom.child_frame_id = "base_link";

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

    ROS_INFO("x = %f", x);
    ROS_INFO("y = %f", y);
    ROS_INFO("th = %f", th);

    last_time = current_time;
    r.sleep();

}
}
