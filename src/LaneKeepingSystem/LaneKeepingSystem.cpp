#include "LaneKeepingSystem/LaneKeepingSystem.hpp"
#define Record 0
#if Record
#include <ctime>
#endif

namespace Xycar {
template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    std::string configPath;
    mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);

    mPID = new PIDController<PREC>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mMovingAverage = new MovingAverageFilter<PREC>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mLaneDetector = new LaneDetector<PREC>(config);
 
    setParams(config);

    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::setParams(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();    
    mDebugging = config["DEBUG"].as<bool>();
#if (Record)
    outputVideo.open(getfilename(),  cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0, cv::Size(640, 480), true);
#endif
}

template <typename PREC>
LaneKeepingSystem<PREC>::~LaneKeepingSystem()
{
    delete mPID;
    delete mMovingAverage;
    delete mLaneDetector;
    // delete your LaneDetector if you add your LaneDetector.
}

template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
    ros::Rate rate(kFrameRate);   
    while (ros::ok())
    {     
        ros::spinOnce();      
        if (mFrame.empty())
            continue;
        double center = mLaneDetector->processImage(mFrame);        
        double error = (center - mLaneDetector->getWidth()/2);
        //mMovingAverage->addSample(error);                
        auto steeringAngle = std::max(std::min(kXycarSteeringAangleLimit,(int32_t)mPID->getControlOutput(error)),-1*kXycarSteeringAangleLimit);
        temp_mDecelerationStep = std::round(std::abs(error)/10)*mDecelerationStep;
        drive(steeringAngle);

        if (mDebugging)
        {       
            cv::imshow("frame", mFrame);
            cv::imshow("roi", mLaneDetector->getDebugROI());
            cv::imshow("Debug", mLaneDetector->getDebugFrame());

#if Record
            outputVideo.write(mLaneDetector->getDebugFrame());
#endif
            cv::waitKey(1);
        }
    }
}

template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);    
}

template <typename PREC>
void LaneKeepingSystem<PREC>::speedControl(PREC steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= temp_mDecelerationStep;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;
    motorMessage.angle = std::round(steeringAngle);
    speedControl(steeringAngle);
    mLaneDetector->setYOffset(mXycarSpeed);
    motorMessage.speed = std::round(mXycarSpeed);
    mPublisher.publish(motorMessage);
}

template <typename PREC>
std::string LaneKeepingSystem<PREC>::getfilename(){
    std::string str_buf;            
    time_t curTime = time(NULL); 
    struct tm* pLocal = localtime(&curTime);
    
    str_buf="/home/nvidia/xycar_ws/src/LaneKeepingSystem/src/LaneKeepingSystem/"+std::to_string(pLocal->tm_year + 1900)+std::to_string(pLocal->tm_mon + 1)+std::to_string(pLocal->tm_mday)+ "_" + std::to_string(pLocal->tm_hour) + std::to_string(pLocal->tm_min) + std::to_string(pLocal->tm_sec)+".mp4";
    return str_buf;
}


template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
} // namespace Xycar
