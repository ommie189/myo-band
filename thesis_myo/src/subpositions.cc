#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h" //mgs for common geometric primatives such as points, vectors, and poses
#include "geometry_msgs/Pose.h"
#include "ros_myo/MyoPose.h" //with pose/gesture ID
#include "ros_myo/MyoArm.h"
#include "qbGradient.hpp"
#include <iostream>
#include <functional>

//using namespace std;

//getting data and save it into variable where current orientation is saved
geometry_msgs::Vector3 orientation_msg;
std::vector<geometry_msgs::Vector3> arm_orientation;
ros_myo::MyoArm arm_msg;
std::vector<ros_myo::MyoArm> arm_positions;

//
void orientationCallback(const geometry_msgs::Vector3::ConstPtr& pose_msg)
{
  ROS_INFO("I heard: [%f]", pose_msg->y);
  orientation_msg=*pose_msg;//global variable where current pose is saved.
}

//global vector where you push the current pose into a vector. the longer the vector the more precise it is.

void gestureCallback(const ros_myo::MyoPose::ConstPtr& pose_msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  switch(pose_msg->pose){
    /*case 0:
    break;

    case 1:*/
    case 2:
      arm_orientation.push_back(orientation_msg);
      /*if(marker_orientation.size()){

      }*/
    break;
    
  }
  ROS_INFO("I heard: [%ld]", arm_orientation.size());//if you move the fist the vector grows longer 
}

/*record the arm position and orientation data at each of the 
three random positions chosen by the person wearing the Myo band*/

void recordPositions(int num_positions){
  ros::NodeHandle n;
  ros::Subscriber sub= n.subscribe("myo_raw/arm", 1000, orientationCallback);

  //std::vector<geometry_msgs::Pose.position> arm_positions;
  //std::vector<ros_myo::MyoArm> arm_positions;
  std::vector<geometry_msgs::Vector3> arm_orientations;
  
//TO DO:if positions are enough, it should calibrate->calculate distance to reference point,
//the new calibration
  while(arm_positions.size() < num_positions){
    ros::spinOnce(); //process any incoming messages
    //If both arm_msg and orientation_msg are present, it 
    //appends the arm position and orientation data to the corresponding vectors
/*
    if(!(arm_msg)||!(orientation_msg)){

      ROS_WARN("Missing Myo arm or orientation data");
      continue;
    }
*/

    arm_positions.push_back(arm_msg);
    arm_orientations.push_back(orientation_msg);
  }

  //process recorded positions and orientations
  //TO DO:calibrate here
  //implement the block
  //qbGradient::qbGradient b=new qbGradient::qbGradient();
  qbGradient b;
  std::function<double(std::vector<double>*)> p_ObjectFcn{ calculateDistanceToReferencePosition };
  b.SetObjectFcn(p_ObjectFcn);
  std::vector<double> startPoint = {1.0};
	b.SetStartPoint(startPoint);
  b.SetMaxIterations(100);
  b.SetGradientThresh(1.0);
  
//Call optimise
  std::vector<double> funcLoc;//reference point
  double funcVal;//error
  b.Optimize(&funcLoc, &funcVal);;

}

void crossProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B, geometry_msgs::Vector3 c_P) {

   c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
   c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
   c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;

}

double calculateDistanceToReferencePosition( std::vector<double> *referencePoint){
  //the 3 known points
  //TO DO: try different points and see how error is behaving
  geometry_msgs::Vector3 p1 = arm_orientation[0];
  geometry_msgs::Vector3 p2 = arm_orientation[1];
  geometry_msgs::Vector3 p3 = arm_orientation[2];


//find difference
  geometry_msgs::Vector3 c;

  c.x = arm_positions[0].position.x - (*referencePoint)[0];
  c.y = arm_positions[1].y - (*referencePoint)[1];
  c.z = arm_positions[2].z - (*referencePoint)[2];

//implement cross product
  geometry_msgs::Vector3 c_P;
  crossProduct(p1,c,c_P);

//magnitude of the cross product and the Orientation
  double magCP = sqrt(pow(c_P.x,2) + pow(c_P.y,2)+ pow(c_P.z,2));
  double magOri1 = sqrt(pow(p1.x,2) + pow(p1.y,2)+pow(p1.z,2));
  double magOri2 = sqrt(pow(p2.x,2) + pow(p2.y,2)+pow(p2.z,2));
  double magOri3 = sqrt(pow(p3.x,2) + pow(p3.y,2)+pow(p3.z,2));

  //geometry_msgs::Vector3 referencePosition;
  double distancep1 = magCP/magOri1;
  double distancep2 = magCP/magOri2;
  double distancep3 = magCP/magOri3;

  double sum = pow(distancep1,2) + pow(distancep2,2) + pow(distancep3,2);

  //sum up distances use Numerical way to minimise the distance
  //look for other libraries as well

  return sum;
}

/*The function returns a single, double-precision floating
	point representing the value of the function at the location
	specified by the input variables. */
  /*
double ObjectFcn(std::vector<double> *funcLoc){
	// y = x^2
	double currentsum = funcLoc->at(0);
  //ComputeGradient b;
	return currentsum;
}
*/

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;

  ros::Subscriber gesturesub = n.subscribe("/myo_raw/myo_gest", 1, gestureCallback);
  ros::Subscriber orientationsub = n.subscribe("/myo_raw/myo_ori", 1, orientationCallback);
 

  ros::spin();


  return 0;
}
