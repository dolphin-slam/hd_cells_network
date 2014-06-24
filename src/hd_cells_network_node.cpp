#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "hd_cells_network/hd_cells_network.h"
#include "visualization_msgs/Marker.h"
#include <tf/tf.h>
#include <angles/angles.h>

using namespace hd_cells;

const double MARKER_SCALE = 0.25;
const int NUMBER_OF_NEURONS = 100;
const double STD_IMU = angles::from_degrees(10);

HDCellsNetwork network(NUMBER_OF_NEURONS);


void publishNetworkActivity()
{
    std::vector<double> activity;

    visualization_msgs::Marker message;

    network.getActivity(activity);

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

    message.scale.x = message.scale.y = message.scale.z = MARKER_SCALE;

    //! configura a cor dos marcadores
    message.color.r = 0.0;
    message.color.g = 0.0;
    message.color.b = 1.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    //! para cada neuronio da matriz
    message.points.resize(activity.size());
    for(int i=0;i<activity.size();i++)
    {
        //! set point position
        message.points[i].x = i;
        message.points[i].y = 0;
        message.points[i].z = activity[i];
    }
}

void publishNetworkInput(std::vector<double> input)
{

    visualization_msgs::Marker message;

    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "hd_cells";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::LINE_STRIP;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "input";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = -input.size()/2;
    message.pose.position.y = 1;
    message.pose.position.z = 0;

    message.pose.orientation.x = 0.0;
    message.pose.orientation.y = 0.0;
    message.pose.orientation.z = 0.0;
    message.pose.orientation.w = 1.0;

    message.scale.x = message.scale.y = message.scale.z = MARKER_SCALE;

    //! configura a cor dos marcadores
    message.color.r = 1.0;
    message.color.g = 0.0;
    message.color.b = 0.0;
    message.color.a = 1.0;

    //! configura os marcadores para serem permanentes
    message.lifetime = ros::Duration(0.0);

    //! para cada neuronio da matriz
    message.points.resize(input.size());
    for(int i=0;i<input.size();i++)
    {
        //! set point position
        message.points[i].x = i;
        message.points[i].y = 0;
        message.points[i].z = input[i];
    }
}



void imuCallback(const sensor_msgs::ImuConstPtr &message)
{
    double yaw = tf::getYaw(message->orientation);


    network.applyExternalInput(yaw,STD_IMU);

    std::vector <double> input;
    network.getLastInput(input);
    publishNetworkInput(input);

}

void timerCallback(const ros::TimerEvent& event)
{

    float yaw = 100; //! Test input


    network.excite();
    network.applyExternalInput(yaw,STD_IMU);
    network.inhibit();

    publishNetworkActivity();

    std::vector <double> input;
    network.getLastInput(input);
    publishNetworkInput(input);;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "hd_cells_network");

    network.initWeights(10*M_PI/180,GAUSSIAN);
    network.setGlobalInhibition(0.001);

    //! ROS Node Handle
    ros::NodeHandle node_handle_;

    //! ROS Topics
    //ros::Subscriber imu_subscriber_ = node_handle_.subscribe("/imu",1,&imuCallback);

    ros::Timer timer = node_handle_.createTimer(ros::Duration(1), &timerCallback);


    ros::spin();


}
