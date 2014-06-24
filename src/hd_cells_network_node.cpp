#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "hd_cells_network/hd_cells_network.h"

using namespace hd_cells;


HDCellsNetwork network(100);


void imuCallback(const sensor_msgs::ImuConstPtr &message)
{

}

void timerCallback(const ros::TimerEvent& event)
{
    network.excite();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "milford_model");


    network.initWeights(10*M_PI/180,GAUSSIAN);
    network.setGlobalInhibition(0.001);

    //! ROS Node Handle
    ros::NodeHandle node_handle_;

    //! ROS Topics
    ros::Subscriber imu_subscriber_;

    imu_subscriber_ = node_handle_.subscribe("/imu",1,&imuCallback);

    //timer = node_handle_.createTimer(ros::Duration(1), &timerCallback);


    ros::spin();


}
