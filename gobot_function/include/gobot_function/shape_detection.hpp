#include <iostream>
#include <vector>
#include <string>

using namespace std;

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

using namespace cv;

struct rgbcolor{
    int R;
    int G;
    int B;  
};

class ShapeDetector
{
public:
    ShapeDetector(){

    }
    ~ShapeDetector(){

    }

    void detect(const Mat &orig, const Mat &curve)
    {
        string shape = "unidentified";
        double peri = arcLength(curve, true);
        Mat approx;
        approxPolyDP(curve, approx, 0.04 * peri, true); // 0.01~0.05
        const int num_of_vertices = approx.rows;

        // if the shape is a triangle, it will have 3 vertices
        if (num_of_vertices == 3)
        {
            shape = "triangle";
        }
        else if (num_of_vertices == 4)
        {// if the shape has 4 vertices, it is either a square or a rectangle
            Rect rec = boundingRect(approx);
            double ar = 1.0 * rec.width / rec.height;

            // A square will have an aspect ratio that is approximately
            // equal to one, otherwise, the shape is a rectangle
            if (ar >= 0.9 && ar <= 1.1)
            {
                shape = "square";
            }
            else
            {
                shape = "rectangle";
            }
        }
        else if (num_of_vertices == 5)
        {
            shape = "pentagon";
        }
        else if (num_of_vertices > 10)
        {// otherwise, we assume the shape is a circle
            shape = "circle";
        }
        else 
        {// if the shape is a pentagon, it will have 5 vertices
            shape = "polygons";
        }

        //get shape
        m_shape = shape;

        //get center
        Moments M = moments(curve);
        int cX = static_cast<int>(M.m10 / M.m00);
        int cY = static_cast<int>(M.m01 / M.m00);
        m_center = Point(cX, cY);
        if(cX<0 || cY<0){
            shape = "unknown";
            cX = 0;
            cY = 0;
        }

        //get color
        m_color = "(" + to_string(orig.at<Vec3b>(cY,cX)[0]) + ", " + to_string(orig.at<Vec3b>(cY,cX)[1]) + ", " + to_string(orig.at<Vec3b>(cY,cX)[2]) + ")";
        m_rgb.R = orig.at<Vec3b>(cY,cX)[2], m_rgb.G = orig.at<Vec3b>(cY,cX)[1], m_rgb.B = orig.at<Vec3b>(cY,cX)[0];
        //get size
        m_area = contourArea(curve);

    }

    string get_shape_type(){
        return m_shape;
    }

    string get_color(){
        return m_color;
    }

    double get_area(){
        return m_area;
    }

    Point get_center(){
        return m_center;
    }

    rgbcolor get_rgb(){
        return m_rgb;
    }

private:
    string m_shape, m_color;
    double m_area;
    Point m_center;
    rgbcolor m_rgb;
};
