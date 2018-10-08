#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;//define tf broadcaster object
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double angle= 0;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;//创建一个tf发布需要使用的TransformStamped类型消息，然后根据消息结构填充当前的时间戳、参考系id、子参考系id
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    sensor_msgs::JointState joint_state;

    while (ros::ok()) 
    {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(4);
        joint_state.position.resize(4);
	    joint_state.name[0] ="R_F_Wheel_joint";
        joint_state.position[0] = 0;
	    joint_state.name[1] ="L_F_Wheel_joint";
        joint_state.position[1] = 0;
	    joint_state.name[2] ="R_B_Wheel_joint";
        joint_state.position[2] = 0;
	    joint_state.name[3] ="L_B_Wheel_joint";
        joint_state.position[3] = 0;

        // update transform
        // (moving in a circle with radius)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = cos(angle)*1;
        odom_trans.transform.translation.y = sin(angle)*1;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);
        //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(M_PI/2);//transform rotation angle to quaternion

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);
		
		angle += degree/4;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}

