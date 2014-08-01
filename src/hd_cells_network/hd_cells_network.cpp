#include "hd_cells_network/hd_cells_network.h"
#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <algorithm>
#include <fstream>
#include <tf/tf.h>


namespace hd_cells
{


/**
 * @brief HDCellsNetwork::HDCellsNetwork
 * @param number_of_neurons
 */
HDCellsNetwork::HDCellsNetwork() : MARKER_SCALE(0.1){

    loadParameters();

    neurons_.create(now_parameters_.now_number_of_neurons_,1);
    std::fill(neurons_.begin(),neurons_.end(),0.0);

    //Initializing variables
    now_parameters_.now_iterations_to_merge_ = 0;
    now_parameters_.now_iterations_to_split_ = 0;
    now_parameters_.splitted_ = 0;
    now_parameters_.merged_ = 0;

    initWeights();

    activity_file_.open(parameters_.activity_filename_.c_str());

    ROS_DEBUG_STREAM(parameters_);

}

HDCellsNetwork::~HDCellsNetwork()
{
    activity_file_.close();
}

double HDCellsNetwork::StdDevExcitation()
{
    return angles::from_degrees(now_parameters_.step_)*parameters_.std_dev_excitation_multiplier_;
}


bool HDCellsNetwork::loadParameters()
{
    ros::NodeHandle private_nh_("~");
    std::string type;

    //! ExcitationType excitation_type_;
    private_nh_.param<std::string>("excitation_type",type,"MEXICAN_HAT");
    if(type == "MEXICAN_HAT")
    {
        parameters_.excitation_type_ = MEXICAN_HAT;
    }
    else if (type == "GAUSSIAN")
    {
        parameters_.excitation_type_ = GAUSSIAN;
    }
    else
    {
        ROS_WARN("Invalid excitation type. Using default -> MEXICAN_HAT");
        parameters_.excitation_type_ = MEXICAN_HAT;
    }

    //! bool use_normalized_weigths_;
    private_nh_.param<bool>("use_normalized_weigths",parameters_.use_normalized_weigths_,true);

    //! double std_dev_excitation_multiplier_;
    private_nh_.param<double>("std_dev_excitation_multiplier",parameters_.std_dev_excitation_multiplier_,angles::from_degrees(5.0));

    //! double std_dev_input_;
    private_nh_.param<double>("std_dev_input",parameters_.std_dev_input_,angles::from_degrees(10.0));

    //! double time_between_updates_;
    private_nh_.param<double>("time_between_updates",parameters_.time_between_updates_,1.0);

    //! bool use_global_inhibition;
    private_nh_.param<bool>("use_global_inhibition",parameters_.use_global_inhibition_,true);

    //! double global_inhibition_;
    private_nh_.param<double>("global_inhibition",parameters_.global_inhibition_,0.25);

    //! NormalizationType normalization_type;
    private_nh_.param<std::string>("normalization_type",type,"NONE");
    if(type == "NONE")
    {
        parameters_.normalization_type_ = NONE;
    }
    else if (type == "TOTAL")
    {
        parameters_.normalization_type_ = TOTAL;
    }
    else if (type == "UTMOST")
    {
        parameters_.normalization_type_ = UTMOST;
    }
    else
    {
        ROS_WARN("Invalid normalization type. Using default -> NONE");
        parameters_.normalization_type_ = NONE;
    }

    //! std::string imu_topic_
    private_nh_.param<std::string>("imu_topic",parameters_.imu_topic_,"imu");

    //! std::string activity_filename_
    private_nh_.param<std::string>("activity_filename",parameters_.activity_filename_,"network_snapshots.txt");

    //! int starting_layer_
    private_nh_.param<int>("starting_layer",parameters_.starting_layer_,4);

    //! int iterations_to_split_
    private_nh_.param<int>("iterations_to_split",parameters_.iterations_to_split_,25);

    //! int iterations_to_merge_
    private_nh_.param<int>("iterations_to_merge",parameters_.iterations_to_merge_,25);

    //! int min_number_of_neurons_
    private_nh_.param<int>("min_number_of_neurons",parameters_.min_number_of_neurons_,16);

    //! int max_number_of_neurons_
    private_nh_.param<int>("max_number_of_neurons",parameters_.max_number_of_neurons_,128);


    now_parameters_.actual_layer_ = parameters_.starting_layer_;
    now_parameters_.now_number_of_neurons_ = pow(2,parameters_.starting_layer_);
    now_parameters_.step_ = (2*M_PI)/now_parameters_.now_number_of_neurons_;
}

double HDCellsNetwork::variation()
{
    return pow(StdDevExcitation(),2);
}

double HDCellsNetwork::distance(int i, int j)
{

    return angles::shortest_angular_distance(i*now_parameters_.step_,j*now_parameters_.step_);
}


bool HDCellsNetwork::applyExternalInput(double angle, double std_dev)
{
    cv::Mat_< double> input(neurons_.size());

    double variation_x2 = 2*variation();

    for(int i=0;i < now_parameters_.now_number_of_neurons_ ;i++)
    {
        input[i][0] = exp(-pow(angles::shortest_angular_distance(i*now_parameters_.step_,angle),2)/ variation_x2);
    }

    applyExternalInput(input);
}

bool HDCellsNetwork::applyExternalInput(double angle)
{
    cv::Mat_< double> input(neurons_.size());

    std::fill(input.begin(), input.end(),0.0);

    input[static_cast<int>(angle/now_parameters_.step_)][0] = 1.0;

    applyExternalInput(input);
}


bool HDCellsNetwork::applyExternalInput(cv::Mat_<double> input)
{
    ROS_DEBUG("Apply external input on network");

    last_input_ = input;

    neurons_ += input;
}



bool HDCellsNetwork::initWeights()
{
    std::ofstream out("recurrent_weights.txt");

    recurrent_weights.create(now_parameters_.now_number_of_neurons_,now_parameters_.now_number_of_neurons_);
    now_parameters_.splitted_++;


    if(parameters_.excitation_type_ == GAUSSIAN)
    {
        for(int i=0;i<now_parameters_.now_number_of_neurons_;i++)
        {
            for(int j=i;j<now_parameters_.now_number_of_neurons_;j++)
            {
                recurrent_weights[i][j] = recurrent_weights[j][i] = exp( -pow(distance(i, j),2)/( 2*variation() ) );
            }
        }

        if(parameters_.use_normalized_weigths_)
        {
            recurrent_weights /= ( 1 / ( StdDevExcitation()*sqrt(2*M_PI) ) );
        }
    }
    else if (parameters_.excitation_type_  == MEXICAN_HAT)
    {
        for(int i=0;i<now_parameters_.now_number_of_neurons_;i++)
        {
            for(int j=i;j<now_parameters_.now_number_of_neurons_;j++)
            {
                recurrent_weights[i][j] = recurrent_weights[j][i] = ( 1 - pow(distance(i,j),2)/variation() )*exp( -pow(distance(i,j),2)/(2*variation()) );
            }
        }

        if(parameters_.use_normalized_weigths_)
        {
            recurrent_weights /= ( 2 / ( sqrt(3*StdDevExcitation()*sqrt(M_PI)) ) );
        }
    }
    else
    {
        ROS_ERROR_STREAM("Invalid excitation type = " << parameters_.excitation_type_ );
    }

    for(int i=0;i<now_parameters_.now_number_of_neurons_;i++)
    {
        for(int j=0;j<now_parameters_.now_number_of_neurons_;j++)
        {
            out << recurrent_weights[i][j] << " ";
        }
        out << std::endl;
    }

    out.close();

}

bool HDCellsNetwork::excite()
{
    ROS_DEBUG("Network excitation");

    cv::Mat_<double> new_neurons(now_parameters_.now_number_of_neurons_,1);
    std::fill(new_neurons.begin(),new_neurons.end(),0.0);

    //! Neuron i receive energy from  every neuron with the associated weight
    for(int i=0;i<now_parameters_.now_number_of_neurons_;i++)
    {
        for(int j=0;j<now_parameters_.now_number_of_neurons_;j++)
        {
            new_neurons[i][0] += neurons_[j][0]*recurrent_weights[j][i];
        }
    }

    for(int i=0;i<now_parameters_.now_number_of_neurons_;i++)
    {
        neurons_[i][0] = std::max(new_neurons[i][0] - neurons_[i][0],0.0);
    }

}

bool HDCellsNetwork::inhibit()
{

    ROS_DEBUG("Network inhibition");

    BOOST_FOREACH(double &neuron, neurons_)
    {
        neuron = std::max(neuron - parameters_.global_inhibition_,0.0);
    }

}



bool HDCellsNetwork::pathIntegrate(double delta_angle)
{
    cv::Mat_<double> new_neurons(now_parameters_.now_number_of_neurons_,1);
    std::fill(new_neurons.begin(),new_neurons.end(),0.0);

    //! \todo Implementar a funçao de integraçao do caminho

    //neurons = new_neurons;
}


bool HDCellsNetwork::normalizeNeurons()
{
    double max =0, total =0;

    if(parameters_.normalization_type_ == UTMOST)
    {
        max = *std::max_element(neurons_.begin(),neurons_.end());
        neurons_ /= max;
    }
    else if(parameters_.normalization_type_ == TOTAL)
    {
        BOOST_FOREACH(double neuron,neurons_)
        {
            total += neuron;
        }
        neurons_ /= total;
    }


}

//void HDCellsNetwork::merge(double input_angle)
//{
//    //Neurons around the input
//    int neuron_ceil, neuron_floor;
//    int find_limits = input_angle/angles::to_degrees(now_parameters_.step_);
//    neuron_ceil = ceil(find_limits);
//    neuron_floor = floor(find_limits);

//    if(neuron_ceil - input_angle < 2*now_parameters_.step_/3    &&  input_angle - neuron_floor > now_parameters_.step_/3    &&  now_parameters_.now_iterations_to_merge_ > parameters_.min_number_of_neurons_)     //! \todo
//    {
//        now_parameters_.now_iterations_to_merge_++;
//        if(now_parameters_.now_iterations_to_merge_ >= parameters_.iterations_to_merge_)
//        {
//            now_parameters_.now_iterations_to_merge_ = 0;
//            merge();
//        }
//    }
//}

//void HDCellsNetwork::merge()
//{
//    cv::Mat_<double> aux_neurons_;
//    cv::Mat_<double> new_neurons_;
//    aux_neurons_ = neurons_.clone();
//    int aux_number_of_neurons;

//    aux_number_of_neurons = now_parameters_.now_number_of_neurons_;
//    now_parameters_.now_number_of_neurons_ /= 2;
//    now_parameters_.step_ = 2*M_PI/now_parameters_.now_number_of_neurons_;
//    neurons_.resize(now_parameters_.now_number_of_neurons_);

//    new_neurons_ = neurons_.clone();

//    initWeights();

//    for (int i=0;i<now_parameters_.now_number_of_neurons_;i++)
//    {
//        new_neurons_[i/2][0] = aux_neurons_[i][0];
//        if (i == now_parameters_.now_number_of_neurons_-1)
//        {
//            new_neurons_[(i/2)+1][0] = (aux_neurons_[i][0] + aux_neurons_[0][0])/2.0;
//        }
//        else
//        {
//            new_neurons_[(i/2)+1][0] = (aux_neurons_[i][0] + aux_neurons_[i+1][0])/2.0;
//        }

//    }

//    neurons_ = new_neurons_.clone();

//}


void HDCellsNetwork::split(double input_angle)
{
    //Neurons around the input
    int neuron_ceil, neuron_floor;
    int find_limits = input_angle/angles::to_degrees(now_parameters_.step_);
    neuron_ceil = ceil(find_limits);
    neuron_floor = floor(find_limits);

    //Input must be in the center of two neurons, dividing the step between them by 3
    if (neuron_ceil - input_angle < 2*now_parameters_.step_/3  && input_angle - neuron_floor > now_parameters_.step_/3   &&  now_parameters_.now_number_of_neurons_ < parameters_.max_number_of_neurons_)
    //! \todo Verificar se esta correta a equacao
    {
        now_parameters_.now_iterations_to_split_++;
        if (now_parameters_.now_iterations_to_split_ >= parameters_.iterations_to_split_)
        {
            now_parameters_.now_iterations_to_split_ = 0;
            split();
        }
    }
}

void HDCellsNetwork::split()
{
    cv::Mat_<double> aux_neurons_;
    cv::Mat_<double> new_neurons_;
    aux_neurons_.create(now_parameters_.now_number_of_neurons_,1);

//    std::fill(aux_neurons_.begin(),aux_neurons_.end(),0.0);

    aux_neurons_ = neurons_.clone();
    int aux_number_of_neurons;

    aux_number_of_neurons = now_parameters_.now_number_of_neurons_;
    now_parameters_.now_number_of_neurons_ *= 2;
    now_parameters_.step_ = 2*M_PI/now_parameters_.now_number_of_neurons_;
    neurons_.resize(now_parameters_.now_number_of_neurons_);

    new_neurons_.create(now_parameters_.now_number_of_neurons_,1);
    std::fill(new_neurons_.begin(),new_neurons_.end(),0.0);
    new_neurons_ = neurons_.clone();

    initWeights();


    for (int i=0;i<aux_number_of_neurons;i++)
    {

        new_neurons_[i*2][0] = aux_neurons_[i][0];
        if (i == aux_number_of_neurons-1)
        {
            new_neurons_[(i*2)+1][0] = (aux_neurons_[i][0] + aux_neurons_[0][0])/2.0;
        }
        else
        {
            new_neurons_[(i*2)+1][0] = (aux_neurons_[i][0] + aux_neurons_[i+1][0])/2.0;
        }

    }

    neurons_ = new_neurons_.clone();

}

void HDCellsNetwork::getActivity(std::vector<double> &act)
{
    neurons_.copyTo(act);
}

void HDCellsNetwork::getLastInput(std::vector<double> &input)
{
    last_input_.copyTo(input);  //! analisar a possibilidade de retornar apenas um ponteiro constante para a matriz.
    //! LER    void convertTo( OutputArray m, int rtype, double alpha=1, double beta=0 ) const;

}

void HDCellsNetwork::update(double input_angle)
{

    //Verifying if the network needs to split
    HDCellsNetwork::split(input_angle);

    std::cout << "After Split: " << format(neurons_.t() ,"csv") << std::endl;

    excite();

    std::cout << "After Excite: " << format(neurons_.t() ,"csv") << std::endl;

    applyExternalInput(input_angle);

    std::cout << "After Input: " << format(neurons_.t() ,"csv") << std::endl;


    if(parameters_.use_global_inhibition_)
    {
        inhibit();
        std::cout << "Global inhibition: " << format(neurons_.t() ,"csv") << std::endl;
    }

    if(parameters_.normalization_type_ != NONE)
    {
        normalizeNeurons();
        std::cout << "Normalized: " << format(neurons_.t() ,"csv") << std::endl;
    }
}

void HDCellsNetwork::timerCallback(const ros::TimerEvent &event)
{

    static ros::Time first_time = event.current_real;
    double ellapsed_time = (event.current_real - first_time).toSec();

    if (ellapsed_time < 5)
    {
        update(angles::from_degrees(70));
    }
    else if (ellapsed_time < 10)
    {
        update(angles::from_degrees(71));
    }
    else if(ellapsed_time < 15)
    {
        update(angles::from_degrees(72));
    }
    else if(ellapsed_time < 20)
    {
        update(angles::from_degrees(73));
    }

    else
    {
        update(angles::from_degrees(359));
    }



    publishActivity();

    publishInput();

}

void HDCellsNetwork::createTimer()
{
    timer_ = node_handle_.createTimer(ros::Duration(parameters_.time_between_updates_),  &HDCellsNetwork::timerCallback,this);

}

void HDCellsNetwork::createROSSubscribers()
{
    imu_subscriber_ = node_handle_.subscribe(parameters_.imu_topic_,1,&HDCellsNetwork::imuCallback,this);
}

void HDCellsNetwork::createROSPublishers()
{
    marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("network",1);
}


void HDCellsNetwork::imuCallback(const sensor_msgs::ImuConstPtr &message)
{
    double yaw = tf::getYaw(message->orientation);

    update(yaw);

    publishActivity();

    publishInput();

}

void HDCellsNetwork::publishActivity()
{
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
    message.pose.position.x = -5; //-activity.size()*MARKER_SCALE/2;
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
    message.points.resize(now_parameters_.now_number_of_neurons_);
    for(int i=0;i<now_parameters_.now_number_of_neurons_;i++)
    {
        //! set point position
        message.points[i].x = i*MARKER_SCALE;
        message.points[i].y = neurons_[i][0];
        message.points[i].z = 0;
    }

    marker_publisher_.publish(message);
}


void HDCellsNetwork::publishInput()
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
    message.points.resize(now_parameters_.now_number_of_neurons_);
    for(int i=0;i<now_parameters_.now_number_of_neurons_;i++)
    {
        //! set point position
        message.points[i].x = i*MARKER_SCALE;
        message.points[i].y = last_input_[i][0];
        message.points[i].z = 0;
    }

    marker_publisher_.publish(message);
}

void HDCellsNetwork::storeNetwork()
{
    activity_file_ << ros::Time::now() << " ";
    BOOST_FOREACH(double act, neurons_)
            activity_file_<< act << " ";
    activity_file_ << std::endl;
}

std::ostream &operator<<(std::ostream &out, HDParameters &parameters)
{
    out << "Network Parameters" << std::endl;
    out << "excitation_type: " << parameters.excitation_type_ << std::endl;
    out << "use_normalized_weigths: " << parameters.use_normalized_weigths_ << std::endl;
    out << "std_dev_excitation_multiplier: " << parameters.std_dev_excitation_multiplier_ << std::endl;
    out << "std_dev_input: " << parameters.std_dev_input_ << std::endl;
    out << "time_between_updates: " << parameters.time_between_updates_ << std::endl;
    out << "use_global_inhibition: " << parameters.use_global_inhibition_ << std::endl;
    out << "global_inhibition: " << parameters.global_inhibition_ << std::endl;
    out << "normalization_type: " << parameters.normalization_type_ << std::endl;
    out << "imu_topic: " << parameters.imu_topic_ << std::endl;
    out << "activity_filename: " << parameters.activity_filename_ << std::endl;
    out << "starting_layer: " << parameters.starting_layer_ << std::endl;
    out << "iterations_to_split: " << parameters.iterations_to_split_ << std::endl;
    out << "iterations_to_merge: " << parameters.iterations_to_merge_ << std::endl;
    out << "min_number_of_neurons" << parameters.min_number_of_neurons_ << std::endl;
    out << "max_number_of_neurons" << parameters.max_number_of_neurons_ << std::endl;
    out << std::endl ;
}

}
