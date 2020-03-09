#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sound_play/sound_play.h>
#include <ros/node_handle.h>
#include <sound_play/SoundRequest.h>
#include <string>
#include <iostream>

std::string response;

void sayback1(const std_msgs::String& message){
    response = message.data;
}


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "saying");
    ros::NodeHandle n1;
    

    ros::Subscriber saySubscriber1 = n1.subscribe("/recognizer/output", 100, sayback1);

    ros::NodeHandle nh("~");
    sound_play::SoundClient sc;

    while(nh.ok()){
        sc.say(response);
        sleep(2);
        break;
    }
   


    // ros::Rate loop_rate(10);
    ros::spin();
}