#include "hd_cells_network/hd_cells_network.h"

namespace hd_cells
{

HDCellsNetwork::HDCellsNetwork(int number_of_neurons)   {

    number_of_neurons_ = number_of_neurons;
    step_ = 2*M_PI/number_of_neurons_;

    neurons.create(number_of_neurons_,1);
}

bool HDCellsNetwork::initWeights(double sigma, ExcitationType type, bool normalize)
{
    double distance;

    recurrent_weights.create(number_of_neurons_,number_of_neurons_);

    if(type == GAUSSIAN)
    {
        for(int i=0;i<number_of_neurons_;i++)
        {
            for(int j=i;j<number_of_neurons_;j++)
            {
                distance = (i-j)*step_;
                recurrent_weights[i][j] = recurrent_weights[j][i] = exp( -pow(distance,2)/( 2*pow(sigma,2) ) );
            }
        }

        if(normalize)
        {
            recurrent_weights /= ( 1 / ( sigma*sqrt(2*M_PI) ) );
        }
    }
    else if (type == MEXICAN_HAT)
    {
        for(int i=0;i<number_of_neurons_;i++)
        {
            for(int j=0;j<number_of_neurons_;j++)
            {
                distance = (i-j)*step_;
                recurrent_weights = ( 1 - pow(distance,2)/pow(sigma,2) )*exp( -pow(distance,2)/( 2*pow(sigma,2) ) );
            }
        }

        if(normalize)
        {
            recurrent_weights /= ( 2 / ( sqrt(3*sigma*sqrt(M_PI)) ) );
        }
    }

}

bool HDCellsNetwork::excite()
{
    cv::Mat_<double> new_neurons(number_of_neurons_,1);
    std::fill(new_neurons.begin(),new_neurons.end(),0.0);

    //! Neuron i excite every neuron with the associated weight
    for(int i=0;i<number_of_neurons_;i++)
    {
        for(int j=0;j<number_of_neurons_;j++)
        {
            new_neurons[j][0] = new_neurons[i][0]*recurrent_weights[i][j];
        }
    }

    neurons = new_neurons;
}

bool HDCellsNetwork::applyExternalInput(cv::Mat_<double> input)
{

    neurons += input;

}

bool HDCellsNetwork::pathIntegrate(double delta_angle)
{
    cv::Mat_<double> new_neurons(number_of_neurons_,1);
    std::fill(new_neurons.begin(),new_neurons.end(),0.0);

    //! \todo Implementar a funçao de integraçao do caminho

    //neurons = new_neurons;
}

bool HDCellsNetwork::setGlobalInhibition(double inhi)
{
    global_inhibition_ = inhi;
}

}
