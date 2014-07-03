#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "hd_cells_network/hd_cells_network.h"
#include "visualization_msgs/Marker.h"
#include <tf/tf.h>
#include <angles/angles.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <time_monitor/time_monitor.h>
#include <fstream>


#include <opencv/cv.h>


using std::cout;
using std::endl;
using std::ofstream;

using namespace hd_cells;

const double MARKER_SCALE = 0.1;
const int NUMBER_OF_NEURONS = 100;
const double STD_IMU = angles::from_degrees(10);
std::ofstream snapshot;
cv::RNG rng(0xFFFFFFFF);




HDCellsNetwork network(NUMBER_OF_NEURONS);

ros::Publisher marker_publisher;


void publishNetworkActivity()
{
    std::vector<double> activity;

    visualization_msgs::Marker message;

    network.getActivity(activity);

    snapshot << ros::Time::now() << " ";
    BOOST_FOREACH(double act, activity)
            snapshot << act << " ";
    snapshot << endl;

    //! completa o cabeçalho da mensagem
    message.header.stamp = ros::Time::now();
    message.header.frame_id = "hd_cells";

    //! configura o tipo e a ação tomada pela mensagem
    message.type = visualization_msgs::Marker::LINE_STRIP;
    message.action = visualization_msgs::Marker::ADD;
    message.ns = "hd_cells";
    message.id = 0;

    //! configura a pose dos marcadores
    message.pose.position.x = -5; //-activity.size()*MARKER_SCALE/2;
    message.pose.position.y = 0;
    message.pose.position.z = 0;

    //cout << activity.size() << " " << MARKER_SCALE << " " << -activity.size()*MARKER_SCALE/2 << endl ;

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
        message.points[i].x = i*MARKER_SCALE;
        message.points[i].y = activity[i];
        message.points[i].z = 0;
    }


    marker_publisher.publish(message);
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
    message.pose.position.x = -5;
    message.pose.position.y = 3;
    message.pose.position.z = 1;

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
        message.points[i].x = i*MARKER_SCALE;
        message.points[i].y = input[i];
        message.points[i].z = 0;
    }

    marker_publisher.publish(message);
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

    static int count = 0;


    float yaw;
    network.excite();

    if(count < 10)
    {
        yaw = angles::from_degrees(100);
        network.applyExternalInput(yaw,STD_IMU);
    }
    else if (count == 30)
    {
        ros::shutdown();
        return;
    }


    network.normalizeNeurons();

    publishNetworkActivity();

    std::vector <double> input;
    network.getLastInput(input);
    publishNetworkInput(input);


    count ++;


}

void testa_execution_time()
{
    TimeMonitor time_monitor;

    HDCellsNetwork *network;

    std::vector<int> sizes(20);

    sizes[0] = 10;
    for(int i=1;i<20;i++)
    {
        if (i <= 10)
        {
            sizes[i] = (i)*50;
        }
        else
        {
            sizes[i] = (i-10)*1000;
        }
    }

    ofstream execution_time("execution_time.txt");
    execution_time << "nneurons excite input normalize" << endl;

    double time_excite,time_input, time_normalize;
    for(int i=0;i<20;i++)
    {
        cout << "Testando tempo de execução com numero de neuronios = " << sizes[i] << endl;
        network = new HDCellsNetwork(sizes[i]);

        network->initWeights(angles::from_degrees(10),MEXICAN_HAT);

        time_excite = time_input = time_normalize = 0;
        for(int j=0;j<100;j++)
        {
            time_monitor.start();
            network->excite();
            time_monitor.finish();
            time_excite += time_monitor.getDuration();

            time_monitor.start();
            network->applyExternalInput(0,angles::from_degrees(10));
            time_monitor.finish();
            time_input += time_monitor.getDuration();

            time_monitor.start();
            network->normalizeNeurons();
            time_monitor.finish();
            time_normalize += time_monitor.getDuration();

        }
        execution_time << sizes[i] << " " << time_excite/100 << " " << time_input/100 << " " << time_normalize/100 << endl;

        delete network;
    }

    execution_time.close();
}




int main(int argc, char **argv)
{

    ros::init(argc, argv, "hd_cells_network");

    network.initWeights(angles::from_degrees(10),MEXICAN_HAT);

    //! ROS Node Handle
    ros::NodeHandle node_handle_;

    //! ROS Topics
    //ros::Subscriber imu_subscriber_ = node_handle_.subscribe("/imu",1,&imuCallback);
    marker_publisher = node_handle_.advertise<visualization_msgs::Marker>("network",1);

    ros::Timer timer = node_handle_.createTimer(ros::Duration(1), &timerCallback);

    snapshot.open("network_snapshots.txt");

    std::vector<double> activity;
    network.getActivity(activity);
    snapshot << ros::Time::now() << " ";
    BOOST_FOREACH(double act, activity)
            snapshot << act << " ";
    snapshot << endl;


    ros::spin();
    //testa_execution_time();

    snapshot.close();


}
