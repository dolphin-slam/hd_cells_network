#ifndef HDCELLS_NETWORK_H
#define HDCELLS_NETWORK_H


#include <opencv/cv.h>

namespace hd_cells
{

enum ExcitationType
{
    GAUSSIAN,
    MEXICAN_HAT
};

class HDCellsNetwork
{
public:
    HDCellsNetwork(int number_of_neurons);


    bool initWeights(double sigma, ExcitationType type, bool normalize = false);
    bool excite();
    bool inhibit();
    bool applyExternalInput(cv::Mat_<double> input);
    bool applyExternalInput(double angle, double std_dev);
    bool pathIntegrate(double delta_angle);
    bool setGlobalInhibition(double inhi);
    void getActivity(std::vector<double> &act);
    void getLastInput(std::vector<double> &input);

    cv::Mat_<double> neurons_;
    cv::Mat_<double> last_input_;
    cv::Mat_<double> recurrent_weights;
    int number_of_neurons_;
    double step_;
    double global_inhibition_;
};



}
#endif // HDCELLS_NETWORK_H
