#ifndef HDCELLS_NETWORK_H
#define HDCELLS_NETWORK_H

#include <ros/ros.h>
#include <opencv/cv.h>
#include <angles/angles.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <iostream>

namespace hd_cells
{

/**
 * @brief Excitation Type
 * GAUSSIAN Gaussian Excitation
 * MEXICAN_HAT Mexican Hat Excitation
 */
enum ExcitationType
{
    GAUSSIAN,
    MEXICAN_HAT
};

enum NormalizationType
{
    NONE,
    TOTAL,
    UTMOST
};

struct HDParameters
{
    int number_of_neurons_;
    ExcitationType excitation_type_;
    bool use_normalized_weigths_;
    double std_dev_excitation_;
    double std_dev_input_;
    double time_between_updates_;
    double step_;
    bool use_global_inhibition_;
    double global_inhibition_;
    NormalizationType normalization_type_;
    std::string imu_topic_;
    std::string activity_filename_;

    friend std::ostream &operator<< (std::ostream &out, HDParameters &parameters);
};

/**
 * @brief Class to model a network of Head Direction Cells.
 *
 */
class HDCellsNetwork
{
public:
    /**
     * @brief Constructor
     */
    HDCellsNetwork();

    /**
     * @brief Destructor
     */
    ~HDCellsNetwork();


    bool loadParameters();

    /**
     * @brief Function to initialize the network recurrent weights.
     * @param sigma Standard Deviation of the weighs
     * @param type Excitation Function Type: GAUSSIAN or MEXICAN_HAT
     * @param normalize Flag to indicate if the network will use normalized weights
     * @return Return true if the initialization goes well
     */
    bool initWeights();

    /**
     * @brief Function to perform network excitation
     * @return Return true if the excitation goes well
     */
    bool excite();


    /**
     * @brief Function to perform network inhibition
     *
     * @return Return true if the inhibition  goes well
     */
    bool inhibit();


    /**
     * @brief Function to perform the external input to the network
     *
     * @param input Matrix of the same size of the network.
     * Contains the amount of input on each neuron.     *
     * @return Return true if the external input goes well
     */
    bool applyExternalInput(cv::Mat_<double> input);

    /**
     * @brief Update network.
     * @todo Documentar os passos realizados nessa função
     * @param input_angle Angle of external input
     */
    void update(double input_angle);

    /**
     * @brief Interface to compute the external input with on angle and a stardard deviation.
     * This function will call @fn applyExternalInput(cv::Mat_ <double> input) after compute
     * the amount of energy will be applied on each neuron.     *
     * @param angle The mean angle of input
     * @param std_dev The standard deviation of the input
     * @return Return true if the external input goes well.
     */
    bool applyExternalInput(double angle, double std_dev);

    /**
     * @brief Interface to compute the external input with only one angle.
     * This function will call @fn applyExternalInput(cv::Mat_ <double> input) after compute
     * the amount of energy will be applied on each neuron.     *
     * @param angle The angle of input
     * @return Return true if the external input goes well.
     */
    bool applyExternalInput(double angle);

    /**
     * @brief Function to perform the path integration process
     *
     * @todo This function was not implemented yet.
     * @param delta_angle The amount of rotation was made by the agent.
     * @return Return true if the path integration goes well.
     */
    bool pathIntegrate(double delta_angle);

    /**
     * @brief Function to normalize the neurons.
     * The normalization was made by divide the neuron activity by the maximun(??) activity on the network.
     * The greatest neuron activity will became one.
     * @todo Colocar a formula aqui
     *
     * @return Return true if the normalization goes well.
     */
    bool normalizeNeurons();

    /**
     * @brief Function to normalize the neurons.
     * The normalization was made by divide the neuron activity by the total activity on the network.
     * The total network activity will became one.
     * @todo Colocar a formula aqui
     *
     * @return Return true if the normalization goes well.
     */bool normalizeTotalNeurons();

    /**
     * @brief Function to return the neuron's activity.
     *
     * @param [out] act The neuron's activity
     */
    void getActivity(std::vector<double> &act);

    /**
     * @brief Function to return the last input applyied on the network.
     *
     * @param [out] input The last external input
     */
    void getLastInput(std::vector<double> &input);

    /**
     * @brief ROS Timer Callback
     * Function to be called every time a timer event occurs.
     *
     * @param event Information about time of the callback.
     */
    void timerCallback(const ros::TimerEvent& event);


    /**
     * @brief Configure the timer callback to update the network
     * The time between updates will be configured using the parameter server.
     */
    void createTimer();

    /**
     * @brief Create ros subscribers
     */
    void createROSSubscribers();

    /**
     * @brief Create ros publishers
     */
    void createROSPublishers();

    /**
     * @brief ROS IMU Subscriber Calback
     * @param message IMU messsage
     */
    void imuCallback(const sensor_msgs::ImuConstPtr &message);

    void publishActivity();

    void publishInput();

    void storeNetwork();


private:

    cv::Mat_<double> neurons_; /**< Matrix to store the neuron's activity.  */
    cv::Mat_<double> last_input_; /**< Matrix to store the last input applyied on the network. */
    cv::Mat_<double> recurrent_weights; /**< Matrix to store the recurrent weights. */

    HDParameters parameters_;

    std::ofstream activity_file_;

    //! ROS related code
    ros::NodeHandle node_handle_;
    ros::Subscriber imu_subscriber_;
    ros::Publisher marker_publisher_;
    ros::Timer timer_;

    //! Constants
    const double MARKER_SCALE;
};



}
#endif // HDCELLS_NETWORK_H
