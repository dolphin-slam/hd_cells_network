#ifndef MILFORD_HDCELLS_MODEL_H
#define MILFORD_HDCELLS_MODEL_H


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
    bool applyExternalInput(cv::Mat_<double> input);
    bool pathIntegrate(double delta_angle);
    bool setGlobalInhibition(double inhi);

    cv::Mat_<double> neurons;
    cv::Mat_<double> recurrent_weights;
    int number_of_neurons_;
    double step_;
    double global_inhibition_;
};



}
#endif // MILFORD_HDCELLS_MODEL_H
