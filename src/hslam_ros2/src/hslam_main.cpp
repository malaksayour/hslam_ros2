#include <thread>
#include <locale.h>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <boost/thread.hpp>
#include <rclcpp/rclcpp.hpp>
//what to do with setting? 
#include "util/settings.h"

#include "FullSystem/FullSystem.h"
#include "util/Undistort.h"

#include "IOWrapper/Output3DWrapper.h"



#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "util/DatasetReader.h"
#include "util/globalCalib.h"

#include "util/NumType.h"
#include "IOWrapper/Pangolin/PangolinDSOViewer.h"
#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <chrono>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>


using namespace std::chrono_literals;
using namespace HSLAM;

std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;


void interruptHandler(int signal)
{
		if(executor){
			std::cout<<"exiting"<<std::endl;
			executor->cancel();
			std::cout<<"executor canceled"<<std::endl;
}
		//exit(1);
}

class HslamSystem : public rclcpp::Node
{
  public:
  	FullSystem* fullSystem=0;
	//std::shared_ptr<LoopCloser> loopCloser;
	rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub;
	rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr map_sub;
	
    HslamSystem() : Node("HslamSystem")
    {
		//init params
		initROSParams();
		initHSLAMParams();

		undistorter_ = Undistort::getUndistorterForFile(calib_, gammaFile_, vignetteFile_);
		setGlobalCalib(
				(int)undistorter_->getSize()[0],
				(int)undistorter_->getSize()[1],
				undistorter_->getK().cast<float>());

		fullSystem= new FullSystem();
		

		fullSystem->linearizeOperation=false;

		if(undistorter_->photometricUndist != 0)
    		fullSystem->setGammaFunction(undistorter_->photometricUndist->getG());

		frame_id_ = 0;
		reset_=false;

		map_pub_= create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, 10);
		pose_pub_= create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
		path_pub_= create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

		timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		image_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

		timer_ = this->create_wall_timer(0.02s, std::bind(&HslamSystem::mapCallback, this),timer_cb_group_);
		//initializeSubscribers();

		IOWrap::PangolinDSOViewer* viewer = 0;
		if(!disableAllDisplay)
		{
			viewer = new IOWrap::PangolinDSOViewer(
					(int)undistorter_->getSize()[0],
					(int)undistorter_->getSize()[1]);
			fullSystem->outputWrapper.push_back(viewer);
		}

		if(useSampleOutput_){
			fullSystem->outputWrapper.push_back(new IOWrap::SampleOutputWrapper());
		}
		
    }

	~HslamSystem(){
		for(IOWrap::Output3DWrapper* ow : fullSystem->outputWrapper)
		{
			//printf("DELETE VIEWER IO wrapper\n");
			ow->join();
			delete ow;
		}
		delete fullSystem;
		delete undistorter_;
	}
	
	void resetFullSystem(){
		std::vector<IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
		for(IOWrap::Output3DWrapper* ow : wraps) ow->reset();
		usleep(50000); //hack - wait for display wrapper to clean up.		
		if(fullSystem)
		{
			delete fullSystem;
			fullSystem = nullptr;
		}

		fullSystem = new FullSystem();
		fullSystem->setGammaFunction(undistorter_->photometricUndist->getG());
		fullSystem->linearizeOperation = false;
		fullSystem->outputWrapper = wraps;
		setting_fullResetRequested=false;
	}

	void initializeSubscribers() {
		auto img_callback =
		[this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
		{
			if (reset_) return;
			
			////////
			//transfer the image to grayscale without opencv. if opencv dependencies was solved in the future,
			//u can replace this with an opencv command.
			sensor_msgs::msg::Image grayscaleImage;

			// Update grayscale image properties
			grayscaleImage.width = msg->width;
			grayscaleImage.height = msg->height;
			grayscaleImage.encoding = "mono8"; // Grayscale encoding
			grayscaleImage.is_bigendian = false;
			grayscaleImage.step = grayscaleImage.width;
			grayscaleImage.data.resize(grayscaleImage.step * grayscaleImage.height);

			// Convert RGB to grayscale
			for (size_t y = 0; y < msg->height; ++y) {
				for (size_t x = 0; x < msg->width; ++x) {
					size_t index = (y * msg->step) + (x * 3); // Assuming 3 channels (RGB)
					uint8_t r = msg->data[index];
					uint8_t g = msg->data[index + 1];
					uint8_t b = msg->data[index + 2];

					// Calculate grayscale value using luminance formula (weighted average)
					uint8_t grayscaleValue = static_cast<uint8_t>(0.2989 * r + 0.5870 * g + 0.1140 * b);

					// Update grayscale image data
					size_t grayscaleIndex = (y * grayscaleImage.step) + x;
					grayscaleImage.data[grayscaleIndex] = grayscaleValue;
				}
			}
			////

			MinimalImageB minImg((int)msg->width, (int)msg->height,grayscaleImage.data.data());
			ImageAndExposure* undistImg = undistorter_->undistort<unsigned char>(&minImg, 1,0, 1.0f);
			undistImg->timestamp=msg->header.stamp.sec; // relay the timestamp to FSLAM
			fullSystem->addActiveFrame(undistImg, frame_id_);
			if(fullSystem->initFailed || setting_fullResetRequested)
			{
				std::lock_guard<std::mutex> lock(mtx_);
				reset_= true;
			}
			frame_id_++;
			publishResults();
		};

      	rclcpp::SubscriptionOptions options;
      	options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		img_sub=create_subscription<sensor_msgs::msg::Image>(
		"/camera/color/image_raw", rclcpp::QoS(rclcpp::KeepLast(10)).reliable(), img_callback,options);
	}


	void publishResults() {        
		nav_msgs::msg::Path path;
		geometry_msgs::msg::PoseStamped pose_stamped;
		sensor_msgs::msg::PointCloud2 map;
		pcl::PointCloud<pcl::PointXYZ> cloud;

		std::vector<SE3> points;
		std::vector<Eigen::Vector3f> map_points;

		path.header.frame_id="map";
		pose_stamped.header.frame_id="map";

		points=fullSystem->getPath();
		for (size_t i = 0; i < points.size(); i++)
		{
			pose_stamped.pose.position.x=points[i].translation().transpose().x();
			pose_stamped.pose.position.y=points[i].translation().transpose().y();
			pose_stamped.pose.position.z=points[i].translation().transpose().z();
			pose_stamped.pose.orientation.x=points[i].so3().unit_quaternion().x();
			pose_stamped.pose.orientation.y=points[i].so3().unit_quaternion().y();
			pose_stamped.pose.orientation.z=points[i].so3().unit_quaternion().z();
			pose_stamped.pose.orientation.w=points[i].so3().unit_quaternion().w();

			path.poses.push_back(pose_stamped);
		}

		path_pub_->publish(path);
		pose_pub_->publish(pose_stamped);

		map_points=fullSystem->getMap();
		for (size_t i = 0; i < map_points.size(); i++)
		{
			pcl::PointXYZ point;
			point.x=map_points[i].x();
			point.y=map_points[i].y();
			point.z=map_points[i].z();
			cloud.push_back(point);
		}
		pcl::toROSMsg(cloud, map);
		map.header.frame_id="map";
		map_pub_->publish(map);
		
	}

  private:
    std::string map_topic_;
    std::string path_topic_;
    std::string pose_topic_;
	int frame_id_;
	rclcpp::TimerBase::SharedPtr timer_;
	bool reset_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
	rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
	rclcpp::CallbackGroup::SharedPtr image_cb_group_;
	std::mutex mtx_;
	Undistort* undistorter_;
	std::string calib_ ;
	std::string vignetteFile_ ;
	std::string gammaFile_ ;
	std::string saveFile_ ;
	std::string vocabPath_ ;
	bool useSampleOutput_;
	bool loopClosure_;
	int mode_ ;
	int preset_;
	bool quiet_;
	bool noLog_;


	void initROSParams(){
		map_topic_=declare_parameter("mapTopic" ,"/map");
      	path_topic_=declare_parameter("pathTopic" ,"/path");
      	pose_topic_=declare_parameter("poseTopic" ,"/pose");
      	calib_=declare_parameter("calibFile" ,"/home/user/catkin_ws/src/res/camera.txt");
      	vignetteFile_=declare_parameter("vignetteFile" ,"");
      	gammaFile_=declare_parameter("gammaFile" ,"");
      	saveFile_=declare_parameter("saveFile" ,"");
      	vocabPath_=declare_parameter("vocabPath" ,"");
      	mode_=declare_parameter("mode" ,1);
      	preset_=declare_parameter("preset" ,0);
      	useSampleOutput_=declare_parameter("useSampleOutput" ,false);
      	loopClosure_=declare_parameter("loopClosure",true);
		quiet_=declare_parameter("quiet" ,false);
		noLog_=declare_parameter("noLog" ,false);

	}
	void initHSLAMParams(){
		LoopClosure=loopClosure_;
		setting_debugout_runquiet=quiet_;
		setting_logStuff=noLog_;

		//mode params
		if(mode_==1)
		{
			setting_photometricCalibration = 0;
			setting_affineOptModeA = 0; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = 0; //-1: fix. >=0: optimize (with prior, if > 0).
			
		}
		if(mode_==2)
		{
			setting_photometricCalibration = 0;
			setting_affineOptModeA = -1; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_affineOptModeB = -1; //-1: fix. >=0: optimize (with prior, if > 0).
			setting_minGradHistAdd = 3;
		}

		//preset params
		if(preset_ == 0 || preset_ == 1)
		{
			printf("DEFAULT settings:\n"
					"- %s real-time enforcing\n"
					"- 2000 active points\n"
					"- 5-7 active frames\n"
					"- 1-6 LM iteration each KF\n"
					"- original image resolution\n", preset_==0 ? "no " : "1x");
		}
		else if(preset_ == 2 || preset_ == 3)
		{
			printf("FAST settings:\n"
					"- %s real-time enforcing\n"
					"- 800 active points\n"
					"- 4-6 active frames\n"
					"- 1-4 LM iteration each KF\n"
					"- 424 x 320 image resolution\n", preset_==0 ? "no " : "5x");
			setting_desiredImmatureDensity = 600;
			setting_desiredPointDensity = 800;
			setting_minFrames = 4;
			setting_maxFrames = 6;
			setting_maxOptIterations=4;
			setting_minOptIterations=1;

			benchmarkSetting_width = 424;
			benchmarkSetting_height = 320;

			setting_logStuff = false;
		}
		if (LoopClosure) printf("fslam_ros :LOOP CLOSURE IS TURNED ON!\n");


		//vocab check
		if(loopClosure_ && !vocabPath_.empty())
		{
			Vocab.load(vocabPath_);
			printf("Loop Closure ON and loading Vocabulary from %s!\n", vocabPath_);
			if (Vocab.empty())
			{
				printf("failed to load vocabulary! Exit\n");
				exit(1);
			}
		}
	}



	void mapCallback()
	{
		//RCLCPP_INFO(rclcpp::get_logger("hslam_node"),"Imgae receivedd \n");
		if(reset_ )
		{
			resetFullSystem();	
			std::lock_guard<std::mutex> lock(mtx_);
			reset_=false;
			return;
		}
		fullSystem->mapPoints();			
	};



};


int main( int argc, char ** argv){
	//boost::thread exThread = boost::thread(exitThread); // hook crtl+C.

	rclcpp::init(argc, argv);
	
	auto sharedHslamNode=  std::make_shared<HslamSystem>();
  	executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();


	executor->add_node(sharedHslamNode);

	sharedHslamNode->initializeSubscribers();
	signal(SIGINT, interruptHandler);

	executor->spin();


	printf("fslam_ros main cpp has been interuppted.\n"); //debug NA
	printf("waiting till mapping is finished\n"); //debug NA
	sharedHslamNode->fullSystem->blockUntilLoopIsFinished();

	sharedHslamNode->fullSystem->BAatExit();
			
	
	sharedHslamNode->fullSystem->printResult("result.txt"); 
	sharedHslamNode->fullSystem->saveMap("map.pcd"); 

	printf("EXIT NOW\n");
	rclcpp::shutdown();
	return 0;

}