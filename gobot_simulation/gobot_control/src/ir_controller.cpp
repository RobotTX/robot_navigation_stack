#include <gobot_control/ir_controller.hpp>

/// IR sensors
int16_t rearSignal_1 = 0;
int16_t rearSignal_2 = 0;
int16_t leftSignal_1 = 0;
int16_t leftSignal_2 = 0;
int16_t rightSignal_1 = 0;
int16_t rightSignal_2 = 0;

ros::Subscriber sub_rearSignal_1;
ros::Subscriber sub_rearSignal_2;
ros::Subscriber sub_leftSignal_1;
ros::Subscriber sub_leftSignal_2;
ros::Subscriber sub_rightSignal_1;
ros::Subscriber sub_rightSignal_2;


void newRearSignal_1(const sensor_msgs::Image::ConstPtr& msg){
    //std::cout << "(ir_controller) newRearSignal called" << std::endl;
    if(msg->width > 0){
        uint8_t red = msg->data[0];
        uint8_t green = msg->data[1];
        uint8_t blue = msg->data[2];
        //std::cout << "(ir_controller) newRearSignal " << (int) red << " " << (int) green << " " << (int) blue << std::endl;

        rearSignal_1 = convertSignal(red, green, blue);
    } else {
        std::cout << "(ir_controller) newRearSignal_1 empty data" << std::endl;
    }
}

void newRearSignal_2(const sensor_msgs::Image::ConstPtr& msg){
    //std::cout << "(ir_controller) newRearSignal called" << std::endl;
    if(msg->width > 0){
        uint8_t red = msg->data[0];
        uint8_t green = msg->data[1];
        uint8_t blue = msg->data[2];
        //std::cout << "(ir_controller) newRearSignal " << (int) red << " " << (int) green << " " << (int) blue << std::endl;

        rearSignal_2 = convertSignal(red, green, blue);
    } else {
        std::cout << "(ir_controller) newRearSignal_2 empty data" << std::endl;
    }
}

void newLeftSignal_1(const sensor_msgs::Image::ConstPtr& msg){
    //std::cout << "(ir_controller) newLeftSignal called" << std::endl;
    if(msg->width > 0){
        uint8_t red = msg->data[0];
        uint8_t green = msg->data[1];
        uint8_t blue = msg->data[2];
        //std::cout << "(ir_controller) newLeftSignal " << (int) red << " " << (int) green << " " << (int) blue << std::endl;

        leftSignal_1 = convertSignal(red, green, blue);
    } else {
        std::cout << "(ir_controller) newLeftSignal_1 empty data" << std::endl;
    }
}

void newLeftSignal_2(const sensor_msgs::Image::ConstPtr& msg){
    //std::cout << "(ir_controller) newLeftSignal called" << std::endl;
    if(msg->width > 0){
        uint8_t red = msg->data[0];
        uint8_t green = msg->data[1];
        uint8_t blue = msg->data[2];
        //std::cout << "(ir_controller) newLeftSignal " << (int) red << " " << (int) green << " " << (int) blue << std::endl;

        leftSignal_2 = convertSignal(red, green, blue);
    } else {
        std::cout << "(ir_controller) newLeftSignal_2 empty data" << std::endl;
    }
}

void newRightSignal_1(const sensor_msgs::Image::ConstPtr& msg){
    //std::cout << "(ir_controller) newRightSignal called" << std::endl;
    if(msg->width > 0){
        uint8_t red = msg->data[0];
        uint8_t green = msg->data[1];
        uint8_t blue = msg->data[2];
        //std::cout << "(ir_controller) newRightSignal " << (int) red << " " << (int) green << " " << (int) blue << std::endl;

        rightSignal_1 = convertSignal(red, green, blue);
    } else {
        std::cout << "(ir_controller) newRightSignal_1 empty data" << std::endl;
    }
}

void newRightSignal_2(const sensor_msgs::Image::ConstPtr& msg){
    //std::cout << "(ir_controller) newRightSignal called" << std::endl;
    if(msg->width > 0){
        uint8_t red = msg->data[0];
        uint8_t green = msg->data[1];
        uint8_t blue = msg->data[2];
        //std::cout << "(ir_controller) newRightSignal " << (int) red << " " << (int) green << " " << (int) blue << std::endl;

        rightSignal_2 = convertSignal(red, green, blue);
    } else {
        std::cout << "(ir_controller) newRightSignal_2 empty data" << std::endl;
    }
}

int16_t multiSignalToSolo(const int16_t sig1, const int16_t sig2){
    if(sig1 == sig2)
        return sig1;
    else{
        if(sig1 == 0 || sig2 == 0)
            return 0;
        else if(sig1 == 3 || sig2 == 3)
            return std::min(sig1, sig2);
        else
            return 3;
    }
}

int16_t convertSignal(const uint8_t red, const uint8_t green, const uint8_t blue){
    if(red == 255 && green != 255 && blue == 255)
        // both signals
        return 3;
    else if(red == 255)
        // right signal
        return 2;
    else if(blue == 255)
        // left signal
        return 1;
    else
        // no signal
        return 0;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "ir_controller");
    std::cout << "(ir_controller) Ready to be launched." << std::endl;

    ros::NodeHandle n;
    
    sub_rearSignal_1 = n.subscribe("/rear_1_camera/rgb/image_raw", 1, newRearSignal_1);
    sub_rearSignal_2 = n.subscribe("/rear_2_camera/rgb/image_raw", 1, newRearSignal_2);
    sub_leftSignal_1 = n.subscribe("/left_1_camera/rgb/image_raw", 1, newLeftSignal_1);
    sub_leftSignal_2 = n.subscribe("/left_2_camera/rgb/image_raw", 1, newLeftSignal_2);
    sub_rightSignal_1 = n.subscribe("/right_1_camera/rgb/image_raw", 1, newRightSignal_1);
    sub_rightSignal_2 = n.subscribe("/right_2_camera/rgb/image_raw", 1, newRightSignal_2);

    ros::Publisher irPublisher = n.advertise<gobot_base::IrMsg>("ir_topic", 50);
    gobot_base::IrMsg msg;

    std::cout << "(ir_controller) launched." << std::endl;

    ros::Rate loop_rate(10);

    while(ros::ok()){
        msg.rearSignal = multiSignalToSolo(rearSignal_1, rearSignal_2);
        msg.leftSignal = multiSignalToSolo(leftSignal_1, leftSignal_2);
        msg.rightSignal = multiSignalToSolo(rightSignal_1, rightSignal_2);

        irPublisher.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
