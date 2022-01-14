#pragma once

#include<vector>
#include <ros/ros.h>
#include <algorithm>
#include <ros/ros.h>


#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <stdarg.h>


#include <string>
#include <initializer_list>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <typeinfo>
#include <fstream>










#include <opencv2/core/fast_math.hpp>







#include <opencv2/aruco.hpp>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"

//#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Pose.h"
#include <algorithm>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/Pose2D.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <typeinfo>
#include <fstream>
#include <tf/transform_listener.h>

#include "../../src/tf_tools.cpp"


class TagDetection{

private:





	//////////Visualization

public:

    static const  int logging_level=2;               //-1 no logs
    // 0 -only main statements
    //1- some imp
    //2- all
    static const bool log_to_file=false;


    static void log(std::string str, int log_level);


};



class TextData{
	private:


	std::string fileName="/home/pranav/ur5_ws/src/robot_control/example.md";


	public:

		TextData(std::string str){
			fileName=str;

		}

		TextData(){

		}

		std::string read(){
		    std::string data;
		    std::ifstream myfileread (fileName);
		    if (myfileread.is_open())
		    {
		        std::string line;
		        while ( std::getline (myfileread,line) )
		        {
		          //std::cout << line << '\n';
		          data=data+line+'\n';
		        }

		    }

		    else {ROS_ERROR_STREAM("NOT OPENED");}

		    myfileread.close();

		    return data;


		}

		void write(std::string data){
		    auto prev_data=read();

		    std::ofstream myfile(fileName);

		    if (myfile.is_open()) { /* ok, proceed with output */
		        //ROS_ERROR_STREAM("OPENED");
		        myfile<<prev_data;
		        myfile<<data<<std::endl;


		     }
		    else{ ROS_ERROR_STREAM("NOT OPENED");}

		    myfile.close();


		}


		void clear(){
		    //auto prev_data=ReadDataFromFile();

		    std::ofstream myfile("/home/pranav/ur5_ws/src/robot_control/example.md");

		    if (myfile.is_open()) {
		        myfile<<"";
		     }
		    else{ ROS_ERROR_STREAM("NOT OPENED");}

		    myfile.close();


		}
};



