#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>

#define NEOPIXEL_COUNT 56

uint16_t neopixel_array[NEOPIXEL_COUNT];
float neopixel_float_array[NEOPIXEL_COUNT];

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser){
    std::vector <float> const &scan = laser->ranges;
    int range_size = scan.size();
    int average_step = 6;

    for(int i = 0; i < NEOPIXEL_COUNT; i++){
        neopixel_float_array[i] = 0;
        neopixel_array[i] = 0;
        int count = 0; // track number of values added
        int index = 0; // index of range array

        if(i < NEOPIXEL_COUNT / 2){
            index = average_step + (average_step * i);
            if(index > range_size/2){
                index = range_size/2;
            }
        }
        else if(i >= NEOPIXEL_COUNT / 2){
            index = range_size - (average_step * (NEOPIXEL_COUNT - i));
            if(index < range_size/2){
                index = range_size/2;
            }
        }

        if(index != 0){
            // loop over subset of value to average them together
            for(int j = 0; j < average_step; j++){
                if(!isinf(scan[index - j])){
                    neopixel_float_array[i] += scan[index - j];
                    count++;
                }
            }
            if(neopixel_float_array[i] != 0){
                neopixel_float_array[i] = neopixel_float_array[i] / count;
            }
            
        }

    }

    for(int i=0; i < NEOPIXEL_COUNT; i++)
    neopixel_array[i] = neopixel_float_array[i]*1000;

}

int main(int argc, char **argv){
    ros::init(argc, argv, "neopixel_publisher");
    ros::NodeHandle nh;
    ros::Publisher neopixel_publisher = nh.advertise<std_msgs::UInt16MultiArray>("neopixel_laserscan", 1000);
    ros::Subscriber laser_subscriber = nh.subscribe<sensor_msgs::LaserScan>("scan", 1000, laserCallback);
    ros::Rate loop_rate(10);
    ROS_INFO("Neopixel Laser Publisher");

    while (ros::ok()){
        std_msgs::UInt16MultiArray neopixel_array_msg;
        neopixel_array_msg.data = std::vector<uint16_t>(neopixel_array, 
                                                        neopixel_array + (sizeof(neopixel_array)/sizeof(neopixel_array[0]))
                                                        );
        neopixel_publisher.publish(neopixel_array_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}