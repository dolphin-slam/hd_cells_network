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

using namespace hd_cells;


ros::Publisher marker_publisher;




//void testa_execution_time()
//{
//    TimeMonitor time_monitor;

//    HDCellsNetwork *network;

//    std::vector<int> sizes(20);

//    sizes[0] = 10;
//    for(int i=1;i<20;i++)
//    {
//        if (i <= 10)
//        {
//            sizes[i] = (i)*50;
//        }
//        else
//        {
//            sizes[i] = (i-10)*1000;
//        }
//    }

//    ofstream execution_time("execution_time.txt");
//    execution_time << "nneurons excite input normalize" << endl;

//    double time_excite,time_input, time_normalize;
//    for(int i=0;i<20;i++)
//    {
//        cout << "Testando tempo de execução com numero de neuronios = " << sizes[i] << endl;
//        network = new HDCellsNetwork(sizes[i]);

//        network->initWeights(angles::from_degrees(10),MEXICAN_HAT);

//        time_excite = time_input = time_normalize = 0;
//        for(int j=0;j<100;j++)
//        {
//            time_monitor.start();
//            network->excite();
//            time_monitor.finish();
//            time_excite += time_monitor.getDuration();

//            time_monitor.start();
//            network->applyExternalInput(0,angles::from_degrees(10));
//            time_monitor.finish();
//            time_input += time_monitor.getDuration();

//            time_monitor.start();
//            network->normalizeNeurons();
//            time_monitor.finish();
//            time_normalize += time_monitor.getDuration();

//        }
//        execution_time << sizes[i] << " " << time_excite/100 << " " << time_input/100 << " " << time_normalize/100 << endl;

//        delete network;
//    }

//    execution_time.close();
//}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "hd_cells_network");

    HDCellsNetwork network;

    network.createROSPublishers();

    network.createTimer();

    ros::spin();

    //testa_execution_time();


}
