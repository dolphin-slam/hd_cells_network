#ifndef HDCELLS_NETWORK_H
#define HDCELLS_NETWORK_H


#include <opencv/cv.h>

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

/**
 * @brief Class to model a network of Head Direction Cells.
 *
 */
class HDCellsNetwork
{
public:
    /**
     * @brief Constructor
     * @param number_of_neurons Number of neurons on the network
     */
    HDCellsNetwork(int number_of_neurons);


    /**
     * @brief Function to initialize the network recurrent weights.
     * @param sigma Standard Deviation of the weighs
     * @param type Excitation Function Type: GAUSSIAN or MEXICAN_HAT
     * @param normalize Flag to indicate if the network will use normalized weights
     * @return Return true if the initialization goes well
     */
    bool initWeights(double sigma, ExcitationType type, bool normalize = false);

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
     * @brief Set the global inhibition.
     *
     * This was not used in the best network configuration
     *
     * @param inhi The amount of global inhibition has to be applied on each neuron
     * @return Return true if the configuration goes well.
     */
    bool setGlobalInhibition(double inhi);

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

private:

    cv::Mat_<double> neurons_; /**< Matrix to store the neuron's activity.  */
    cv::Mat_<double> last_input_; /**< Matrix to store the last input applyied on the network. */
    cv::Mat_<double> recurrent_weights; /**< Matrix to store the recurrent weights. */
    int number_of_neurons_; /**< Number of neurons on the network. */
    double step_; /**< Step angle between neurons.*/
    double global_inhibition_; /**< Global inhibition. */
};



}
#endif // HDCELLS_NETWORK_H
