#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "hd_cells_network/hd_cells_network.h"
#include "visualization_msgs/Marker.h"

using namespace hd_cells;


HDCellsNetwork network(100);


void publishNetworkActivity()
{
    std::vector<float> activity;

    float scale = 0.25;

    visualization_msgs::Marker message;

    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "hd_cells";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::LINE_STRIP;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "hd_cells";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = -activity.size()/2;
    message.pose.position.y = 0;
    message.pose.position.z = 0;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x = scale;
    message.scale.y = scale;
    message.scale.z = scale;

    //! configura a cor dos marcadores
    message.color.r = 0.0;
    message.color.g = 0.0;
    message.color.b = 1.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);


    float max = *std::max_element(activity.begin(),activity.end());
    //! para cada neuronio da matriz
    //cout << "Activity = " << endl;
    message.points.resize(activity.size());
    message.colors.resize(activity.size());
    for(int i=0;i<activity.size();i++)
    {
        //! set point position
        message.points[i].x = i;
        message.points[i].y = 0;
        message.points[i].z = activity[i];
    }
}

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
