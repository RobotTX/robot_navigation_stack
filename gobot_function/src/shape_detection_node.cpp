#include <gobot_function/shape_detection.hpp>
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include "cv_bridge/cv_bridge.h"
#include <std_msgs/Int16.h>
#include <gobot_msg_srv/robot_msgs.h>

using namespace std;
using namespace cv;

const int RGB_THRESHOLD = 30;
int ALIGNMENT_THRESHOLD = 15; 
double AREA_THRESHOLD = 7200, AREA_DIFF_THRESHOLD = 10, STOP_Y_THRESHOLD = 5500;
bool y_adjustment = false, right_edge = false;
int alignment_data_ = 0; 

ros::Time lostSignal;
ros::Publisher alignment_pub;

bool shapeFilter(int area, string shape, rgbcolor color){
    if(area < 500)
        return false;
    
    if(shape != "triangle")
        return false;
    
    if(color.R<1.5*color.G || color.R<1.5*color.B || abs(color.G-color.B)>RGB_THRESHOLD){
        return false;
    }

    return true;
}

bool getAlignmentCb(gobot_msg_srv::GetInt::Request &req, gobot_msg_srv::GetInt::Response &res){
    res.data = alignment_data_;
    return true;
}

/*
alignment:  VALUE   COMMENT                 MOTION
            100     not detected            turn spot
            99      end alignment           ---
            0       aligned                 backward
            1       center in right         turn spot right slowest
            2       center in right         turn spot right slower
            5       center in right         turn spot right
            10      left area>right area    backward right wheel to turn right
            -1      center in left          turn spot left slowest
            -2      center in left          turn spot left slower
            -5      center in left          turn spot left
            -10     right area>left area    backward left wheel to turn left
*/
void shapeDetection(Mat image){
    Mat gray, blurred, thresh;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, blurred, Size(5, 5), 0.0);
    threshold(blurred, thresh, 100, 255, THRESH_BINARY);
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    int index = 1;
    vector<Point> p_cen;
    vector<double> p_area;
    ShapeDetector sd;
    std_msgs::Int16 alignment;
    alignment.data = 100;

    for (size_t i = 0; i < contours.size(); i++)
    {
        // compute the center of the contour, then detect the name of the
        // shape using only the contour
        sd.detect(Mat(image),Mat(contours[i]));

        //filter by size
        if(shapeFilter(sd.get_area(),sd.get_shape_type(),sd.get_rgb())){
            drawContours(image, contours, i, Scalar(0, 255, 0), 1.5);
            
            string image_text = to_string(sd.get_area());
            cv::putText(image, image_text, sd.get_center(), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 255, 255), 1);

            index++;
            p_cen.push_back(sd.get_center());
            p_area.push_back(sd.get_area());
            cv::circle(image, sd.get_center(),5, CV_RGB(255,0,0),-1);
        }
    }

    //draw center line of image
    cv::Point image_cen1(image.cols/2,0), image_cen2(image.cols/2,image.rows);

    if(p_cen.size() == 2){
        int left_x, right_x;
        double left_area, right_area;
        if(p_cen[0].x<p_cen[1].x){
            left_x = p_cen[0].x;
            right_x = p_cen[1].x;
            left_area = p_area[0];
            right_area = p_area[1];
        }
        else{
            left_x = p_cen[1].x;
            right_x = p_cen[0].x;
            left_area = p_area[1];
            right_area = p_area[0];
        }
        
        std::cout<< "left triangle: center@"<<left_x<<"; area@"<<left_area<<endl;
        std::cout<< "right triangle: center@"<<right_x<<"; area@"<<right_area<<endl;

        double area_diff = left_area<right_area ? (left_area - right_area)/left_area : (left_area-right_area)/right_area;
        area_diff = area_diff*100.0;
        std::cout<< "Analyze area: difference@" <<  left_area-right_area <<"; Percentage@"<<area_diff<<"%"<<std::endl;
        cv::Point p_cen_line((p_cen[0].x + p_cen[1].x)/2, (p_cen[0].y + p_cen[1].y)/2);

        alignment_data_ = p_cen_line.x-image_cen1.x;

        //check if robot is close to the target
        if(left_area>AREA_THRESHOLD || right_area>AREA_THRESHOLD){
            alignment.data = 0;
            std::cout<< "Near to target"<<std::endl;
        }
        //check if robot align with center of image
        else {
            if(y_adjustment){
                alignment.data = right_edge ? 10 : -10;
                std::cout<< "Y-axis alignment"<<std::endl;
                cv::circle(image, p_cen_line,10, CV_RGB(255,0,0),2);
                cv::line(image,image_cen1,image_cen2,CV_RGB(0,0,255),2);
            }
            else{
                if(abs(p_cen_line.x-image_cen1.x)< ALIGNMENT_THRESHOLD){
                    cv::circle(image, p_cen_line,10, CV_RGB(0,255,0),-1);
                    cv::line(image,image_cen1,image_cen2,CV_RGB(0,255,0),2);
                    
                    //if y-axis alignment needed
                    if(fabs(area_diff)>AREA_DIFF_THRESHOLD && left_area<STOP_Y_THRESHOLD && right_area<STOP_Y_THRESHOLD){
                        //if left area > right area
                        if(area_diff > 0){
                            alignment.data = 10;
                            right_edge = true;
                        }
                        //if right area > left area
                        else{
                            alignment.data = -10;
                            right_edge = false;
                        }
                        y_adjustment = true;
                        std::cout<< "Start y-axis alignment"<<std::endl;
                    }
                    //check if robot align with y-axis
                    else{
                        alignment.data = 0;
                        std::cout<< "Perfect alignment"<<std::endl;
                    }
                }
                else if(!y_adjustment){
                    cv::circle(image, p_cen_line,10, CV_RGB(255,0,0),2);
                    cv::line(image,image_cen1,image_cen2,CV_RGB(0,0,255),2);
                    if(abs(p_cen_line.x-image_cen1.x) < 30){
                        alignment.data = p_cen_line.x>image_cen1.x ? 1 : -1;
                    }
                    else if(abs(p_cen_line.x-image_cen1.x) < 60){
                        alignment.data = p_cen_line.x>image_cen1.x ? 2 : -2;
                    }
                    else{
                        alignment.data = p_cen_line.x>image_cen1.x ? 5 : -5;
                    }
                    std::cout<< "Miss alignment"<<std::endl;
                }
            }
        }
        lostSignal = ros::Time::now();

        string image_text = std::to_string(p_cen_line.x-image_cen1.x);
        cv::Point text_pos(p_cen_line.x-20, p_cen_line.y+40);
        cv::putText(image,image_text,text_pos,cv::FONT_HERSHEY_SIMPLEX,0.6,CV_RGB(255,255,255),1);
        cout<<endl;
    }
    //can not see both objects
    else if(y_adjustment){
        if(ros::Time::now()-lostSignal>ros::Duration(1.0)){
            y_adjustment = false;
            alignment.data = 99;
            std::cout<< "End Y-axis alignment"<<std::endl;
        }
        else{
            alignment.data = right_edge ? 10 : -10;
            std::cout<< "Y-axis alignment"<<std::endl;
        }
        alignment_data_ = 999;
    }
    else{
        std::cout<< "Can not find object"<<std::endl;
        alignment_data_ = 999;
    }

    alignment_pub.publish(alignment);
    imshow("Result", image);
    waitKey(3);
}

void imageCb(const sensor_msgs::ImageConstPtr &image){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    shapeDetection(cv_ptr->image);
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "shape_detection_node");
    ros::NodeHandle nh;

    nh.getParam("ALIGNMENT_THRESHOLD", ALIGNMENT_THRESHOLD);
    nh.getParam("AREA_THRESHOLD", AREA_THRESHOLD);
    nh.getParam("AREA_DIFF_THRESHOLD", AREA_DIFF_THRESHOLD);
    nh.getParam("STOP_Y_THRESHOLD", STOP_Y_THRESHOLD);

    ros::Subscriber camera_sub = nh.subscribe("/usb_cam/image_raw", 10, imageCb);
    alignment_pub = nh.advertise<std_msgs::Int16>("/gobot_function/object_alignment", 10);

    ros::ServiceServer getAlignmentSrv = nh.advertiseService("/gobot_function/get_object_alignment", getAlignmentCb);


    ros::spin();
    return 0;
}