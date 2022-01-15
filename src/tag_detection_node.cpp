
#include <opencv2/calib3d.hpp>
#include <unordered_map>
#include"tag_detection.h"

// TagDetection::~TagDetection(){

// }

// TagDetection::TagDetection():


// {



// }





cv_bridge::CvImagePtr cv_ptr;
cv::Size_<int> image_size;
double distance_btw_markers=0;
double l=8.5;
double real_distance_btw_markers = 13*l;
double pixel_to_mm=1;
double pi = 3.141596;
TF *TF_;



void StartTF(tf::TransformListener &transformListener, unordered_map<int,std::pair<string,geometry_msgs::Pose>> idToFrameMap)
{
    geometry_msgs::PoseStamped p;
    p.pose=TF::MakeGeometryMsgsPose(0,0,0,0,0,0,1);
    TF_->PublishStaticTransform("base","origin",p.pose);

    for(auto itr = idToFrameMap.begin(); itr!=idToFrameMap.end();itr++)
    {
        std::cout<<"publishing static transform for frame "<<itr->second.first<<std::endl;
        TF_->PublishStaticTransform(itr->second.first,"/origin",itr->second.second);
        ros::spinOnce();
    }
    /*geometry_msgs::Pose tracker1firstPointPoseWRTOrigin=TF::MakeGeometryMsgsPose(1.0,0.465+0.1,0,0,0,0,1);
    TF_->PublishStaticTransform("/tracker1firstPointPoseWRTOrigin","/origin",tracker1firstPointPoseWRTOrigin);
    ros::spinOnce();
//    ros::Duration(0.5).sleep();
    geometry_msgs::Pose tracker1PoseWRTOrigin = TF_->getInFrame(transformListener,TF::MakeGeometryMsgsPose(4.0*l*0.001,4.0*l*0.001,0,0,0,0,1),"/origin","/tracker1firstPointPoseWRTOrigin");
    TF_->PublishStaticTransform("/tracker1PoseWRTOrigin","/tracker1firstPointPoseWRTOrigin",TF::MakeGeometryMsgsPose(4.0*l*0.001,4.0*l*0.001,0,0,0,0,1));
    ros::spinOnce();
//    ros::Duration(0.5).sleep();


    geometry_msgs::Pose tracker2firstPointPoseWRTOrigin=TF::MakeGeometryMsgsPose(1.0 + (8*l*0.001) + (5*l*0.001),0.465+0.1  ,0,0,0,0,1);
    TF_->PublishStaticTransform("/tracker2firstPointPoseWRTOrigin","/origin",tracker2firstPointPoseWRTOrigin);
    geometry_msgs::Pose tracker2PoseWRTOrigin = TF_->getInFrame(transformListener,TF::MakeGeometryMsgsPose(4*l*0.001,4*l*0.001,0,0,0,0,1),"/origin","/tracker2firstPointPoseWRTOrigin");
    TF_->PublishStaticTransform("/tracker2PoseWRTOrigin","/tracker2firstPointPoseWRTOrigin",TF::MakeGeometryMsgsPose(4.0*l*0.001,4.0*l*0.001,0,0,0,0,1));
//    ros::Duration(0.5).sleep();*/

}





void EdgeDetection(cv::Mat src){

    cv::Mat dst,cdst,cdstP;
    cv::Canny(src, dst, 50, 200, 3);
    // Copy edges to the images that will display the results in BGR
    cv::cvtColor(dst, cdst, cv::COLOR_GRAY2BGR);
    cdstP = cdst.clone();
    // Standard Hough Line Transform
    std::vector<cv::Vec2f> lines; // will hold the results of the detection
    cv::HoughLines(dst, lines, 1, 3.141596/180, 150, 0, 0 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 + 10*(-b));
        pt2.y = cvRound(y0 + 10*(a));

        std::cout<<"Line is"<<std::endl;
        std::cout<<pt1.x<<" "<<pt1.y<<" "<<pt2.x<<" "<<pt2.y<<std::endl;
        cv::circle(cdst,pt1,25,cvScalar(0,0,0),1);
        cv::line( cdst, pt1, pt2, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }
    // Probabilistic Line Transform
    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
    cv::HoughLinesP(dst, linesP, 1,  3.1415926/180, 50, 50, 10 ); // runs the actual detection
    // Draw the lines
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        cv::Vec4i l = linesP[i];
        cv::line( cdstP, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, cv::LINE_AA);
    }
    // Show results
    cv::imshow("Source", src);
    cv::imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
    cv::imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP);
    // Wait and Exit
    // cv::waitKey();
}


double distance( cv::Point2f p1 , cv::Point2f p2 ){
    return ( sqrt(((p1.y-p2.y)*(p1.y-p2.y)) + ((p1.x-p2.x)*(p1.x-p2.x)) ));
}

double ConvertFromMinusPitoPi(double a)
{
    while(a<-pi || a>pi)
    {
        if(a<-pi)
            a = 2 * pi + a;
        if(a>pi)
            a = a - 2*pi;
    }
    return a;
}



double getAngle(cv::Point2f p1, cv::Point2f p2)   //in rad           //p1 to p2
{
    double a=atan2((p2.y-p1.y), (p2.x-p1.x));

    return ConvertFromMinusPitoPi( a);
}

struct Marker{
    cv::Point2f first_point;                //is always topleft when marker kept straight
    cv::Point2f second_point;
    cv::Point2f third_point;
    cv::Point2f forth_point;

    cv::Point2f center;
    int id;

    double d;



    Marker(std::vector<cv::Point2f> Points, int id){
        first_point=Points[0];second_point=Points[1];third_point=Points[2];forth_point=Points[3];

        center.x = (first_point.x + third_point .x)/2.0;
        center.y = (first_point.y + third_point .y)/2.0;
        d= (distance(first_point,second_point)+ distance(third_point,forth_point))/2.0;
        this->id = id;

    }


    static double Markerdistance(Marker m1, Marker m2){
        return distance(m1.center,m2.center);
    }

    double getMarkerSizeinPixels(){
        return d;
    }

    geometry_msgs::Pose2D getCameraPose()  //returns values in px
    {
        geometry_msgs::Pose2D camera_pose_wrt_m1;
        camera_pose_wrt_m1.theta = ConvertFromMinusPitoPi(getAngle(first_point,forth_point));
        auto dx1 = -(center.x-(image_size.width/2.0));               //minus sign bcs things moe in reverse in img
        auto dy1 = -(center.y-(image_size.height/2.0));
        camera_pose_wrt_m1.x =( (dx1*cos(camera_pose_wrt_m1.theta)) + (dy1* sin(camera_pose_wrt_m1.theta)) );    // if does not work put minus sign on next line
        camera_pose_wrt_m1.y =-( -(dx1*sin(camera_pose_wrt_m1.theta)) + (dy1*cos(camera_pose_wrt_m1.theta))  );   // converting from camera pose to tracker pose
        return camera_pose_wrt_m1;
    }

};


void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
    // std::cout<<"Subscriber Called"<<std::endl;
    std_msgs::Header msg_header = msg->header;
    std::string frame_id = msg_header.frame_id.c_str();
    //ROS_INFO_STREAM("New Image from " << frame_id);


    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


}


void TagDetection::log(std::string str, int log_level)
{
    std::cout<<str<<std::endl;
//    if(logging_level==-1)
//        return;
//
//
//
//
//
//    if(TagDetection::logging_level>=0   &&   log_level>=0)
//    {
//        std::cout<<"[MAIN]:"<<std::to_string(str)<<std::endl;
//    }
//    if(TagDetection::logging_level>=1  &&   log_level>=1)
//    {
//        std::cout<<"[IMP]:"<<std::to_string(str)<<std::endl;
//    }
//    if(TagDetection::logging_level>=2    &&   log_level>=2)
//    {
//        std::cout<<"[EXTRA]:"<<std::to_string(str)<<std::endl;
//    }

}

std::pair<Marker,Marker> GetOrderedMarkers(Marker m1, Marker m2)   //first in pair closer to marker
{

    //main vect angle
    auto main_angle= getAngle(m1.first_point,m2.first_point);



    //right line anfle
    auto right_line_angle = getAngle(m1.first_point, m1.second_point);
    auto angle_diff=main_angle-right_line_angle;
    angle_diff= ConvertFromMinusPitoPi(angle_diff);


    if(abs(angle_diff-(pi/2.0))<0.2)
        return std::make_pair(m1,m2);
    else
        return std::make_pair(m2,m1);
}




void DistanceByInterpolation2Markers(vector<vector<cv::Point_<float>>> &markerCorners, vector<int> &markerIds, cv::Mat &outputImage, double &pixel_to_mm_d, tf::TransformListener &transformListener)
{
    Marker m1dash(markerCorners[0],markerIds[0]),m2dash(markerCorners[1],markerIds[1]);

    Marker *m1;
    Marker *m2;


    if(m1dash.id==25)
    {
        m1= &m1dash;
        m2= &m2dash;
    }
    if(m1dash.id==33)
    {
        m1= &m2dash;
        m2= &m1dash;
    }
    else
    {
        m1= &m1dash;
        m2= &m2dash;
    }

    std::pair<Marker,Marker> temp= GetOrderedMarkers(*m1,*m2);  //m1 is the one closer to origin
    m1 = &temp.first;
    m2 = &temp.second;


    distance_btw_markers = Marker::Markerdistance(*m1,*m2);
    pixel_to_mm = distance_btw_markers/ real_distance_btw_markers;
    pixel_to_mm_d = (m1->d + m2->d)/2/8/l;
    //TagDetection::log(std::to_string(pixel_to_mm),0);




    auto camera_pose_wrt_m1= m1->getCameraPose();
    camera_pose_wrt_m1.x *= (pixel_to_mm_d*0.001);    //and convert to m for rviz viz
    camera_pose_wrt_m1.y *= (pixel_to_mm_d*0.001);

    auto camera1_quaternion=TF::EulerToQuaternion(0,0,camera_pose_wrt_m1.theta);
    auto camera_pose_wrt_m1_3d=TF::MakeGeometryMsgsPose(camera_pose_wrt_m1.x,camera_pose_wrt_m1.y,0,camera1_quaternion.x,camera1_quaternion.y,camera1_quaternion.z,camera1_quaternion.w);
    auto camera_pose = TF_->getInFrame(transformListener,camera_pose_wrt_m1_3d,"/tracker1PoseWRTOrigin","/origin");
    TF_->publishFrame(camera_pose,"/camera_1","/origin");




    auto camera_pose_wrt_m2= m2->getCameraPose();
    camera_pose_wrt_m2.x *= (pixel_to_mm_d*0.001);    //and convert to m for rviz viz
    camera_pose_wrt_m2.y *= (pixel_to_mm_d*0.001);
    auto camera2_quaternion=TF::EulerToQuaternion(0,0,camera_pose_wrt_m2.theta);
    auto camera_pose_wrt_m2_3d=TF::MakeGeometryMsgsPose(camera_pose_wrt_m2.x,camera_pose_wrt_m2.y,0,camera2_quaternion.x,camera2_quaternion.y,camera2_quaternion.z,camera2_quaternion.w);
    camera_pose = TF_->getInFrame(transformListener,camera_pose_wrt_m2_3d,"/tracker2PoseWRTOrigin","/origin");


    TF_->publishFrame(camera_pose,"/camera_2","/origin");


    cv::arrowedLine(outputImage,m1->first_point,m2->first_point, cvScalar(0,0,255),5);

    cv::arrowedLine(outputImage,m1->first_point,m1->second_point, cvScalar(255,0,0),5);



    cv::circle(outputImage,m1->first_point,10,cvScalar(0,255,255),5);
    cv::circle(outputImage,m2->first_point,10,cvScalar(0,255,255),5);


    cv::circle(outputImage,m1->center,10,cvScalar(255,0,0),5);
    cv::circle(outputImage,m2->center,10,cvScalar(0,0,255),5);
    cv::putText(outputImage,std::to_string((m1->center.x-(image_size.width/2.0))) + " " + std::to_string(m1->center.y-(image_size.height/2.0)),cv::Point2f(55,55),cv::FONT_HERSHEY_DUPLEX,1.0,
                cvScalar(0,0,255));

    cv::putText(outputImage, std::to_string(pixel_to_mm_d) + " " + std::to_string(pixel_to_mm)  + " " +  std::to_string (markerIds[1]) ,cv::Point2f(55,155),cv::FONT_HERSHEY_DUPLEX,1.0,
                cvScalar(0,0,255));
};

void DistanceByInterpolation1Marker(vector<vector<cv::Point_<float>>> &markerCorners, vector<int> &markerIds, cv::Mat &outputImage, double &pixel_to_mm_d, tf::TransformListener &transformListener)
{
    Marker m1(markerCorners[0],markerIds[0]);
    //TagDetection::log(std::to_string(pixel_to_mm),0);



    auto camera_pose_wrt_m1= m1.getCameraPose();
    camera_pose_wrt_m1.x *= (pixel_to_mm_d*0.001);    //and convert to m for rviz viz
    camera_pose_wrt_m1.y *= (pixel_to_mm_d*0.001);
    auto camera1_quaternion=TF::EulerToQuaternion(0,0,camera_pose_wrt_m1.theta);
    auto camera_pose_wrt_m1_3d=TF::MakeGeometryMsgsPose(camera_pose_wrt_m1.x,camera_pose_wrt_m1.y,0,camera1_quaternion.x,camera1_quaternion.y,camera1_quaternion.z,camera1_quaternion.w);
    std::string pose_frame_id="/tracker1PoseWRTOrigin",camera_id="/camera_1";
    if(m1.id==25) {
        pose_frame_id = "/tracker1PoseWRTOrigin";
        camera_id= "/camera_1";
    }
    if(m1.id==33) {
        pose_frame_id = "/tracker2PoseWRTOrigin";
        camera_id= "/camera_2";
    }
    auto camera_pose = TF_->getInFrame(transformListener,camera_pose_wrt_m1_3d,pose_frame_id,"/origin");


    TF_->publishFrame(camera_pose,camera_id,"/origin");

    cv::circle(outputImage,m1.first_point,10,cvScalar(0,255,255),5);


    cv::circle(outputImage,m1.center,10,cvScalar(255,0,0),5);


    cv::putText(outputImage,std::to_string((m1.center.x-(image_size.width/2.0))) + " " + std::to_string(m1.center.y-(image_size.height/2.0)),cv::Point2f(55,55),cv::FONT_HERSHEY_DUPLEX,1.0,
                cvScalar(0,0,255));

    cv::putText(outputImage, std::to_string(m1.center.x) + " " + std::to_string(m1.center.y) + " " + std::to_string(camera_pose_wrt_m1.theta) + " " + std::to_string (markerIds[0]) ,cv::Point2f(55,155),cv::FONT_HERSHEY_DUPLEX,1.0,
                cvScalar(0,0,255));
}



void HouseHolderQR(cv::Mat &A, cv::Mat &Q, cv::Mat &R)
{
    assert ( A.channels() == 1 );
    assert ( A.rows >= A.cols );
    auto sign = [](float value) { return value >= 0 ? 1: -1; };
    const auto totalRows = A.rows;
    const auto totalCols = A.cols;
    R = A.clone();
    Q = cv::Mat::eye ( totalRows, totalRows, A.type() );
    for ( int col = 0; col < A.cols; ++ col )
    {
        cv::Mat matAROI = cv::Mat ( R, cv::Range ( col, totalRows ), cv::Range ( col, totalCols ) );
        cv::Mat y = matAROI.col ( 0 );
        auto yNorm = norm ( y );
        cv::Mat e1 = cv::Mat::eye ( y.rows, 1, A.type() );
        cv::Mat w = y + sign(y.at<float>(0,0)) *  yNorm * e1;
        cv::Mat v = w / norm( w );
        cv::Mat vT; cv::transpose(v, vT );
        cv::Mat I = cv::Mat::eye( matAROI.rows, matAROI.rows, A.type() );
        cv::Mat I_2VVT = I - 2 * v * vT;
        cv::Mat matH = cv::Mat::eye ( totalRows, totalRows, A.type() );
        cv::Mat matHROI = cv::Mat(matH, cv::Range ( col, totalRows ), cv::Range ( col, totalRows ) );
        I_2VVT.copyTo ( matHROI );
        R = matH * R;
        Q = Q * matH;
    }
}

bool isRotationMatrix(cv::Mat &R)

{

    cv::Mat Rt;
    cv::transpose(R, Rt);
    cv::Mat shouldBeIdentity = Rt * R;
    cv::Mat I = cv::Mat::eye(3,3, shouldBeIdentity.type());
    return  cv::norm(I, shouldBeIdentity) < 1e-6;

}



// Calculates rotation matrix to euler angles

// The result is the same as MATLAB except the order

// of the euler angles ( x and z are swapped ).

cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R)

{
//    if(!isRotationMatrix(R))
//    {
//        cv::Mat A; cv::Mat Q; cv::Mat r;
//
////        HouseHolderQR(R,Q,r);
////
////        R = Q.clone();
//
//
//    }

    cv::Mat Q,r;
    HouseHolderQR(R,Q,r);

    R = Q.clone();


    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }

    else

    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }

    return cv::Vec3f(x, y, z);



}



void UsingHomography(Marker *m1, Marker *m2, cv::Mat calibration_matrix, cv::Mat distortion)
{
    //m1
    std::vector<cv::Point3f> objectPoints;
    objectPoints.push_back(cv::Point3f(1.00,0.572,0));  //p1
    objectPoints.push_back(cv::Point3f(1.00,0.572 + (8*l*0.001) ,0));  //p2
    objectPoints.push_back(cv::Point3f(1.00 + (8*l*0.001),0.572 + (8*l*0.001),0));  //p3
    objectPoints.push_back(cv::Point3f(1.00 + (8*l*0.001),0.572,0));  //p4
//
//
//
//            //m2
//            objectPoints.push_back(cv::Point3f(1.00 + (13*l*0.001),0.572,0));  //p1
//            objectPoints.push_back(cv::Point3f(1.00 +  (13*l*0.001),0.572 + (8*l*0.001) ,0));  //p1
//            objectPoints.push_back(cv::Point3f(1.00 +  (13*l*0.001) +(8*l*0.001),0.572 + (8*l*0.001),0));  //p1
//            objectPoints.push_back(cv::Point3f(1.00 + (13*l*0.001) + (8*l*0.001),0.572,0));  //p1


//
//





//            calcChessboardCorners(patternSize, squareSize, objectPoints);
    std::vector<cv::Point2f> objectPointsPlanar;
    for (size_t i = 0; i < objectPoints.size(); i++)
    {
        objectPointsPlanar.push_back(cv::Point2f(objectPoints[i].x, objectPoints[i].y));
    }
    std::vector<cv::Point2f> corners;
    corners.push_back(m1->first_point);
    corners.push_back(m1->second_point);
    corners.push_back(m1->third_point);
    corners.push_back(m1->forth_point);


//            corners.push_back(m2->first_point);
//            corners.push_back(m2->second_point);
//            corners.push_back(m2->third_point);
//            corners.push_back(m2->forth_point);




    std::vector<cv::Point2f> imagePoints;
    cv::undistortPoints(corners, imagePoints, calibration_matrix, distortion);
//            imagePoints=corners;



//            The homography can then be estimated with:
    cv::Mat H = cv::findHomography(objectPointsPlanar, imagePoints);
//            std::cout << "H:\n" << H << std::endl;

    double norm = sqrt(H.at<double>(0,0)*H.at<double>(0,0) +
                       H.at<double>(1,0)*H.at<double>(1,0) +
                       H.at<double>(2,0)*H.at<double>(2,0));
    H /= norm;
    cv::Mat c1  = H.col(0);
    cv::Mat c2  = H.col(1);
    cv::Mat c3 = c1.cross(c2);
    cv::Mat tvec = H.col(2);
    cv::Mat R(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
    {
        R.at<double>(i,0) = c1.at<double>(i,0);
        R.at<double>(i,1) = c2.at<double>(i,0);
        R.at<double>(i,2) = c3.at<double>(i,0);
    }


    cv::Mat W, U, Vt;
    cv::SVDecomp(R, W, U, Vt);
    R = U*Vt;


    cv::Vec3f euler_angles=rotationMatrixToEulerAngles(R);

//            std::cout<<euler_angles<<std::endl;
    double angle = ConvertFromMinusPitoPi(euler_angles[2]);

    double angle1 = ConvertFromMinusPitoPi(getAngle(m1->first_point,m2->second_point));











    geometry_msgs::Pose2D camera_pose_wrt_m1;

    auto dx1 =  tvec.at<double>(0);            //minus sign bcs things moe in reverse in img
    auto dy1 =  tvec.at<double>(1);
    camera_pose_wrt_m1.x =( (dx1*cos(angle1)) + (dy1* sin(angle1)) );   // converting from camera pose to tracker pose
    camera_pose_wrt_m1.y =( (-dx1*sin(angle1)) + (dy1*cos(angle1))  );


    std::cout<<"First\n"<< camera_pose_wrt_m1.x  <<" "<<  camera_pose_wrt_m1.y  <<" "<<angle<<std::endl;
}




int main(int argc, char** argv) {
  

    // cv::imwrite("marker50.png", markerImage);

    TagDetection::log("Start Program",0);
    // cv::Mat inputImage= cv::imread("marker50.png");


    ros::init(argc,argv,"tag_detection");
    ros::NodeHandle nh_;
    image_transport::Subscriber image_sub_;
    TF_=new TF();
    tf::TransformListener transformListener;

    std::unordered_map<int,std::pair<string,geometry_msgs::Pose>> idToFrameMap; /// has name of frame and Pose of midpoint of marker wrt origin



    idToFrameMap[25] = std::make_pair("tracker1Origin",TF::MakeGeometryMsgsPose(1.0 + (4.0*l*0.001),0.575 + (4*l*0.001),0,0,0,0,1));
    idToFrameMap[33] = std::make_pair("tracker2Origin",TF::MakeGeometryMsgsPose(1.0 + (17.0*l*0.001),0.575 + (4*l*0.001),0,0,0,0,1));

    StartTF(transformListener, idToFrameMap);









    // OpenCV Window Name
    static const std::string OPENCV_WINDOW = "Image window";

    


    image_transport::ImageTransport it_(nh_);
    image_sub_ = it_.subscribe("/camera/image_raw", 1,      &image_cb);

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);




    while(1)
    {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ros::spinOnce();
        if(!cv_ptr->image.empty())
        {
            image_size = cv_ptr->image.size();
            TagDetection::log("Image size wh is "+std::to_string(image_size.width) + " " + std::to_string(image_size.height),0);
            break;
        }
    }


    ///We get all these vals by running Calibration.py
    cv::Mat  calibration_matrix, new_camera_matrix, distortion;
    double data[10] = {1340.28016,   0,   967.660953,  0,   1343.37760,   497.492186,  0,   0,   1.0 };
    calibration_matrix = cv::Mat(3, 3, CV_64F, data);     //cv32f means float numbers

    double data1[10] = {1260.12976,   0.000,   987.517863 , 0.00,   1259.19385,   479.093936,  0.00,   0.0000,   1.00};
    new_camera_matrix =  cv::Mat(3, 3, CV_64F, data1);

    double data2[5] = {0.06204154, -0.37845927, -0.01174156,  0.00749854,  0.28207879};
    distortion =  cv::Mat(1, 5, CV_64F, data2);
    cv::Rect roi(27, 21, 1867, 1013);



    std::cout<<"while loop started"<<std::endl;
    double pixel_to_mm_d=1;
    while (1) 
    {

        int text_len =1;
        cv::Mat undistorted_image = cv_ptr->image.clone();

        //cv::undistort(cv_ptr->image, undistorted_image, calibration_matrix, distortion, new_camera_matrix );
//
//
//        undistorted_image = undistorted_image(roi);
//        image_size = undistorted_image.size();

        cv::Mat image, imageCopy;
        //inputVideo.retrieve(image);
        undistorted_image.copyTo(image);
        undistorted_image.copyTo(imageCopy);
        cv::Mat outputImage = imageCopy.clone();

        ros::spinOnce();


        
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::generateCustomDictionary(250, 6);
        cv::aruco::detectMarkers(imageCopy, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        std::unordered_map<int,Marker*> markers;

        double denominator=0;
        for(int i=0;i<markerIds.size();i++)
        {
            Marker *m = new Marker(markerCorners[i],markerIds[i]);
            markers[m->id]=(m);
            denominator += m->getMarkerSizeinPixels();

        }
        if (denominator!=0)
            pixel_to_mm_d =  (markerIds.size())*8*l /(denominator);



        for(auto itr = markers.begin(); itr!=markers.end(); itr++)
        {
            auto m = itr->second;
            auto camera_pose_wrt_marker= m->getCameraPose();


            camera_pose_wrt_marker.x *= (pixel_to_mm_d*0.001);    //and convert to m for rviz viz
            camera_pose_wrt_marker.y *= (pixel_to_mm_d*0.001);
            auto camera_quaternion=TF::EulerToQuaternion(0,0,camera_pose_wrt_marker.theta);

            auto camera_pose_wrt_marker_3d=TF::MakeGeometryMsgsPose(camera_pose_wrt_marker.x,camera_pose_wrt_marker.y,0,camera_quaternion.x,camera_quaternion.y,camera_quaternion.z,camera_quaternion.w);

            std::string pose_frame_id="/tracker1Origin",camera_id="/camera_wrt_";  // placeholders

            pose_frame_id = idToFrameMap[m->id].first;
            camera_id += idToFrameMap[m->id].first;

            auto camera_pose = TF_->getInFrame(transformListener,camera_pose_wrt_marker_3d, pose_frame_id , "/origin");


            TF_->publishFrame(camera_pose,camera_id,"/origin");




            cv::arrowedLine(outputImage,m->first_point,m->forth_point, cvScalar(255,0,0),5);



            cv::circle(outputImage,m->first_point,10,cvScalar(0,0,255),5);
            cv::circle(outputImage,m->second_point,10,cvScalar(0,255,0),5);
            cv::circle(outputImage,m->third_point,10,cvScalar(255,0,0),5);
            cv::circle(outputImage,m->forth_point,10,cvScalar(254,255,255),5);


            cv::circle(outputImage,m->center,10,cvScalar(255,0,0),5);

            cv::putText(outputImage,std::to_string((m->center.x-(image_size.width/2.0))) + " " + std::to_string(m->center.y-(image_size.height/2.0)),cv::Point2f(55,text_len*55),cv::FONT_HERSHEY_DUPLEX,1.0,
                        cvScalar(0,0,255));
            text_len++;
            cv::putText(outputImage, std::to_string(pixel_to_mm_d) + " " + std::to_string(pixel_to_mm)  + " id:" +  std::to_string (m->id) ,cv::Point2f(55,text_len*55),cv::FONT_HERSHEY_DUPLEX,1.0,
                        cvScalar(0,0,255));
            text_len++;



        }
        /*//2 markers detected
        if(markerCorners.size()==2)
        {

            Marker m1dash(markerCorners[0],markerIds[0]),m2dash(markerCorners[1],markerIds[1]);

            Marker *m1;
            Marker *m2;


            if(m1dash.id==25)
            {
                m1= &m1dash;
                m2= &m2dash;
            }
            if(m1dash.id==33)
            {
                m1= &m2dash;
                m2= &m1dash;
            }
            else
            {
                m1= &m1dash;
                m2= &m2dash;
            }

            std::pair<Marker,Marker> temp= GetOrderedMarkers(*m1,*m2);  //m1 is the one closer to origin
            m1 = &temp.first;
            m2 = &temp.second;

            distance_btw_markers = Marker::Markerdistance(*m1,*m2);
            pixel_to_mm =  real_distance_btw_markers/distance_btw_markers;
            pixel_to_mm_d = 2*8*l /(m1->d + m2->d);



            //TagDetection::log(std::to_string(pixel_to_mm),0);



            auto camera_pose_wrt_m1= m1->getCameraPose();
            camera_pose_wrt_m1.x *= (pixel_to_mm*0.001);    //and convert to m for rviz viz
            camera_pose_wrt_m1.y *= (pixel_to_mm*0.001);
            auto camera1_quaternion=TF::EulerToQuaternion(0,0,camera_pose_wrt_m1.theta);
            auto camera_pose_wrt_m1_3d=TF::MakeGeometryMsgsPose(camera_pose_wrt_m1.x,camera_pose_wrt_m1.y,0,camera1_quaternion.x,camera1_quaternion.y,camera1_quaternion.z,camera1_quaternion.w);
            std::string pose_frame_id="/tracker1PoseWRTOrigin",camera_id="/camera_1";
            if(m1->id==25) {
                pose_frame_id = "/tracker1PoseWRTOrigin";
                camera_id= "/camera_1";
            }
            if(m1->id==33) {
                pose_frame_id = "/tracker2PoseWRTOrigin";
                camera_id= "/camera_2";
            }
            auto camera_pose = TF_->getInFrame(transformListener,camera_pose_wrt_m1_3d, pose_frame_id , "/origin");


            TF_->publishFrame(camera_pose,camera_id,"/origin");











            cv::arrowedLine(outputImage,m1->first_point,m2->first_point, cvScalar(0,0,255),5);

            cv::arrowedLine(outputImage,m1->first_point,m1->second_point, cvScalar(255,0,0),5);



            cv::circle(outputImage,m1->first_point,10,cvScalar(0,0,255),5);
            cv::circle(outputImage,m1->second_point,10,cvScalar(0,255,0),5);
            cv::circle(outputImage,m1->third_point,10,cvScalar(255,0,0),5);
            cv::circle(outputImage,m1->forth_point,10,cvScalar(254,255,255),5);


            cv::circle(outputImage,m1->center,10,cvScalar(255,0,0),5);
            cv::circle(outputImage,m2->center,10,cvScalar(0,0,255),5);
            cv::putText(outputImage,std::to_string((m1->center.x-(image_size.width/2.0))) + " " + std::to_string(m1->center.y-(image_size.height/2.0)),cv::Point2f(55,55),cv::FONT_HERSHEY_DUPLEX,1.0,
                        cvScalar(0,0,255));

            cv::putText(outputImage, std::to_string(pixel_to_mm_d) + " " + std::to_string(pixel_to_mm)  + " " +  std::to_string (markerIds[1]) ,cv::Point2f(55,155),cv::FONT_HERSHEY_DUPLEX,1.0,
                        cvScalar(0,0,255));






        }


        //1 markers detected
        if(markerCorners.size()==1)
        {
            //how to know which one got detected

            DistanceByInterpolation1Marker(markerCorners,markerIds,outputImage,pixel_to_mm_d,transformListener);

        }



        // no markers detected
        if(markerCorners.size()==0)
        {
//            std::cout<<"No markers detected"<<std::endl;
        }
*/



        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

        cv::imshow("out", outputImage);
//        cv::imshow("undistrot", undistorted_image);
        char key = (char) cv::waitKey(1);
        if (key == 27)
            break;
    }




    return 0;
}



