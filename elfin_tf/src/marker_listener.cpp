#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "std_msgs/String.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "tf2_msgs/TFMessage.h"
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>



#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include "Eigen/Geometry"



//Just the Initialization
void markerCallback( const geometry_msgs::PoseStamped& msg );


ros::Publisher pub_cam_chicken;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_listener");
    ROS_INFO("Node Initialized");
    ros::NodeHandle nh;
    ros::Subscriber sub_x_= nh.subscribe("/elfin_camera/aruco_tracker/pose", 1000, &markerCallback);
    pub_cam_chicken = nh.advertise<geometry_msgs::PoseStamped>("/elfin/chicken_cam_pos", 1000);
    ros::spin();
    return 0;
}

void markerCallback( const geometry_msgs::PoseStamped& msg ) 
{
    ROS_INFO("Subscribed to Marker Pose");
    //Conversions to Right-handed System
    float x = msg.pose.position.x;
    float y = msg.pose.position.y;
    float z = msg.pose.position.z;

   //q = msg.pose.orientation;
    tf::Transform markerTF;
    tf::Quaternion q;
    
    static tf::TransformBroadcaster br_marker;
    static tf::TransformBroadcaster br_cam;
    markerTF.setOrigin(tf::Vector3(x, y, z));
    quaternionMsgToTF( msg.pose.orientation  , q);
   //q = q.inverse();         //Conversion to Right-handed System //This didn't work


    double roll, pitch, yaw;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    q.setRPY(roll-1.57, pitch, yaw);        // Conversion to Right-handed System
    markerTF.setRotation(q);


   // Camera Pose Calculation

   tf::Transform marker2cam_TF = markerTF; //.inverse();   //Camera in Marker Frame
   //Marker  in world frame
    tf::Transform world2marker_TF;

    // This just a random pose. We always fix this in gazebo
    world2marker_TF.setOrigin(tf::Vector3(0.7, 0.0, 0.12));
    q.setRPY(0, -1.05, 0);
    world2marker_TF.setRotation(q);

    ROS_INFO("Calculating transforms");

   //Eigens for Transform Multiplication
    Eigen::Affine3d marker2cam_EIG ;
    tf::transformTFToEigen( marker2cam_TF ,marker2cam_EIG ) ;
    Eigen::Affine3d world2marker_EIG;
    tf::transformTFToEigen(world2marker_TF ,world2marker_EIG ) ;
    Eigen::Affine3d world2cam_EIG;
    tf::Transform world2cam_TF ;
    // worldToCamera =  worldToMarker * MarkerToCamera
    world2cam_EIG = world2marker_EIG * marker2cam_EIG;
    tf::transformEigenToTF(world2cam_EIG, world2cam_TF);



    ROS_INFO("Broadcasting transforms");
    // BroadCast World to Camera Frame - We use a new  camera_link_visual frame to keep the original and estimates separate
    br_cam.sendTransform(tf::StampedTransform(world2cam_TF, ros::Time::now(), "world", "camera_link_visual_chicken"));
    tf::TransformListener tflistener;
    tf::StampedTransform stf;

    try{
      tflistener.lookupTransform("camera_link_visual_chicken", "world",  
                                ros::Time(0), stf);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

    
    // geometry_msgs::PoseStamped msgfuck = geometry_msgs::PoseStamped();
    // msgfuck.header.stamp = ros::Time::now();
    // msgfuck.header.frame_id = "world";
    // msgfuck.pose.position.x = stf.getOrigin().x();
    // msgfuck.pose.position.y = stf.getOrigin().y();
    // msgfuck.pose.position.z = stf.getOrigin().z();
    // msgfuck.pose.orientation.x = stf.getRotation().x();
    // msgfuck.pose.orientation.y = stf.getRotation().y();
    // msgfuck.pose.orientation.z = stf.getRotation().z();
    // msgfuck.pose.orientation.w = stf.getRotation().w();

    // pub_cam_chicken.publish(msgfuck);

}

