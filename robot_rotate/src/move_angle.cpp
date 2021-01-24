#include <pid/pid.h>
#include "iostream"
#include "geometry_msgs/Vector3Stamped.h"
#include <robot_rotate/Rotate.h>
#include "ros/ros.h"
#include <robot_rotate/move_motors.h>

ros::Publisher speed_pub; 
ros::Subscriber imu_sub;
float currentAngle ;

void sendOutput(float output)

    {
        bool inRt1 = true, inRt2 = false, inLt1 = true, inLt2 = false;
        float enLt = 255, enRt = 255;

        if (output < 0) //the car needs to move to lt 
        {
            inRt1 = false;
            inRt2 = true;
            enRt = output * -1; 
            inLt1 = true;
            inLt2 = false;
            enLt = output * -1; 
        }
        else // the car needs to move to right
        {   
            inRt1 = true;
            inRt2 = false;
            enRt = output; 
            inLt1 = false;
            inLt2 = true;
            enLt = output;
        }
        robot_rotate::move_motors msg;
        msg.in_rt_1 = inRt1; 
        msg.in_rt_2 = inRt2; 
        msg.en_rt = enRt;
        msg.in_lt_1 = inLt1; 
        msg.in_lt_2 = inLt2; 
        msg.en_lt = enLt;
        speed_pub.publish(msg);
        ros::spinOnce();

    }
void IMUCallback(const geometry_msgs::Vector3Stamped msg)
{
    currentAngle = msg.vector.z;
    if (currentAngle < 0){
        currentAngle = currentAngle*180.0*7/22 + 360.0;
    }
    else {
        currentAngle = currentAngle*180.0*7/22;
    }
    //ROS_INFO("I heard: [%s]", currentAngle.c_str());
}
float IMUReading()
    {
        // insert the code reading from IMU topic
        //currentAngle = data.vector.z
        ros::spinOnce();
        return currentAngle;
    }
bool PIDHandle(float goalAngle){ //ch

    double kp = 10, ki =0, kd=0;

    // P, I, and D represent constants in the user's program
    PIDController<float> myPIDController(kp, ki, kd, IMUReading, sendOutput); //the PID of the two motors
    ROS_INFO("PID Start.");

    bool continueFlag = true;
    myPIDController.setTarget(goalAngle); //sets the target angle 
    //myPIDController.setInputBounded(true); traget bounds
    //myPIDController.setInputBounds(0,360);
    myPIDController.setOutputBounded(true); //Enables using output bounds
    myPIDController.setOutputBounds(-255,255); // Set the limits of the output that will be read to the h-brigde
    // the following two functions makes the controller understand that the line is actually a circle, and that 0=360, and 10 = 370
    myPIDController.setFeedbackWrapped(true); 
    myPIDController.setFeedbackWrapBounds(0,360);

    while (continueFlag)
    {  
        myPIDController.tick();
        int output = int(myPIDController.getOutput());
        std::cout << "pwm reading is " + output;
        ROS_INFO("inside the while %d.",output);
        ROS_INFO("ca %f.",currentAngle);

        if (output==0)
        {
            ROS_INFO("end.");

            continueFlag= false;
        }
       
    }

}
bool handle_rotate(robot_rotate::Rotate::Request  &req,
         robot_rotate::Rotate::Response &res)
{
    PIDHandle(req.angle);
    res.rotated = true;
    return true;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "rotate_robot_server");
  ros::NodeHandle n;
  speed_pub = n.advertise<robot_rotate::move_motors>("motors_speed", 1000);
  imu_sub = n.subscribe("imu/rpy/filtered", 1000, IMUCallback);
  ros::ServiceServer service = n.advertiseService("rotate_robot", handle_rotate);
  ROS_INFO("Ready to rotate.");
  ros::spin();

  return 0;
}
