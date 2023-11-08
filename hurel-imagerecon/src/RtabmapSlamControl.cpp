#include "RtabmapSlamControl.h"
#include <future>
#include <mutex>

using namespace HUREL::Compton;



void ShowCV_32FAsJet(cv::Mat img, int size)
{
	if (img.type() != CV_32F)
	{
		return;
	}
	cv::Mat normImg(img.rows, img.cols, CV_8UC1, cv::Scalar(0));
	double minValue;
	double maxValue;
	cv::minMaxIdx(img, &minValue, &maxValue);
	for (int i = 0; i < img.rows; i++)
	{
		for (int j = 0; j < img.cols; j++)
		{
			normImg.at<uchar>(i, j) = static_cast<uchar>((static_cast<double>(img.at<float>(i, j)) - minValue)
				/ (maxValue - minValue) * 255);
		}
	}
	cv::Mat colorImg;
	cv::applyColorMap(normImg, colorImg, cv::COLORMAP_JET);
	cv::Mat showImg;

	int sizeHeight = size;
	int sizeWidth = size;

	if (colorImg.size().height > colorImg.size().width)
	{
		sizeWidth = size * colorImg.size().width / colorImg.size().height;
	}
	else
	{
		sizeHeight = size * colorImg.size().height / colorImg.size().width;
	}

	cv::resize(colorImg, showImg, cv::Size(sizeWidth, sizeHeight), 0, 0, cv::INTER_NEAREST);
	cv::imshow("img", showImg);
	cv::waitKey(0);
}


HUREL::Compton::RtabmapSlamControl::RtabmapSlamControl()
{
	Initiate();
	StartVideoStream();
}

bool HUREL::Compton::RtabmapSlamControl::Initiate()
{
	std::string msg;
	std::string* outMessage = &msg;

	//set default parameters
	{
		UDEBUG("Logging settings changed...");
		ULogger::setType(ULogger::kTypeConsole);
		ULogger::setLevel(ULogger::kDebug);
		// ULogger::setLevel((ULogger::Level)_preferencesDialog->getGeneralLoggerLevel());
	}

	try {
		rs2::context ctx = rs2::context();
		rs2::device_list devs =  ctx.query_devices();
		
		rs2::config cfgD455 = rs2::config();
		int devCounts = devs.size();
		if (devCounts == 0)
		{
			*outMessage += "No cameras";

			spdlog::error("C++::HUREL::Compton::RtabmapSlamControl: {0}", *outMessage);
			return false;
		}
		std::string usbInfo;
		for (int i = 0; i < devCounts; ++i)
		{
			std::string devInof = devs[i].get_info(rs2_camera_info::RS2_CAMERA_INFO_NAME);
			if (devInof == "Intel RealSense D455")
			{
				usbInfo = devs[i].get_info(rs2_camera_info::RS2_CAMERA_INFO_USB_TYPE_DESCRIPTOR);
			}
		}
		*outMessage = "RtabmapSlamControl: d455 connencted with usb " + usbInfo + " \n";

	}
	catch (const rs2::camera_disconnected_error& e)
	{
		*outMessage += "RtabmapSlamControl: Camera was disconnected! Please connect it back ";
		*outMessage += e.what();
		spdlog::error("C++::HUREL::Compton::RtabmapSlamControl: {0}", *outMessage);
		return false;
	}
	// continue with more general cases
	catch (const rs2::recoverable_error& e)
	{
		*outMessage += "RtabmapSlamControl: Operation failed, please try again ";
		*outMessage +=  e.what();
		spdlog::error("C++::HUREL::Compton::RtabmapSlamControl: {0}", *outMessage);

		return false;
	}
	// you can also catch "anything else" raised from the library by catching rs2::error
	catch (const rs2::error& e)
	{
		*outMessage += "RtabmapSlamControl: Some other error occurred! ";
		*outMessage += e.what();
		spdlog::error("C++::HUREL::Compton::RtabmapSlamControl: {0}", *outMessage);

		return false;
	}
	
	try
	{
		mCamera = new rtabmap::CameraRealSense2();

		mCamera->setResolution(848, 480);
		mCamera->setDepthResolution(848, 480);

		//const rtabmap::Transform test(0.000f, 0.13f, 0.0f, 0.0f, 0.0f, 0.0f);
		const rtabmap::Transform test(0.028f, -0.03f, 0.0f, 0.0f, 0.0f, 0.0f);
		//mCamera->setDualMode(true, test);

		//mCamera->setOdomProvided(true, false, true);
		//mCamera->setImagesRectified(true);
	}
	catch(int exp)
	{
		*outMessage += "RtabmapSlamControl: CameraRealSense2 error " + exp ;
	}
	if (!mCamera->init())
	{
		*outMessage += "RtabmapSlamControl: Initiate failed";
		spdlog::error("C++::HUREL::Compton::RtabmapSlamControl: {0}", *outMessage);
		delete mCamera;
		mCamera = nullptr;		
		mIsInitiate = false;
		ULogger::setLevel(ULogger::kError);
		return false;
	}
	else
	{
		*outMessage += "RtabmapSlamControl: Initiate success";
		spdlog::info("C++::HUREL::Compton::RtabmapSlamControl: {0}", *outMessage);
		StartVideoStream();
		mIsInitiate = true;
		ULogger::setLevel(ULogger::kError);

		return true;
	}	
}

RtabmapSlamControl& HUREL::Compton::RtabmapSlamControl::instance()
{
	static RtabmapSlamControl* instance = new RtabmapSlamControl();
	return *instance;
}

HUREL::Compton::RtabmapSlamControl::~RtabmapSlamControl()
{
	delete mCamera;
	delete mCameraThread;
}

static std::future<void> t1;

void HUREL::Compton::RtabmapSlamControl::StartVideoStream()
{
	if (!mIsInitiate || mIsVideoStreamOn)
	{
		return;
	}
	
	spdlog::error("C++::HUREL::Compton::RtabmapSlamControl: {0}", "RtabmapSlamControl start video stream");

	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;


	mCameraThread = new rtabmap::CameraThread(mCamera);
	mCameraThread->start();
	mIsVideoStreamOn = true;

	//auto func = std::bind(&RtabmapSlamControl::VideoStream, this);
	//t1 = std::async(std::launch::async, func);
}

void HUREL::Compton::RtabmapSlamControl::StopVideoStream()
{
	if (!mIsVideoStreamOn)
	{
		return;
	}
	mIsVideoStreamOn = false;
	
	mCameraThread->join(true);
	mCameraThread->kill();
	delete mCameraThread;
	mCameraThread = nullptr;

	//t1.get();
	//HUREL::Logger::Instance().InvokeLog("C++::HUREL::Compton::RtabmapSlamControl", "RtabmapSlamControl stop video stream", eLoggerType::INFO);
	
}

static std::mutex videoStreamMutex;
static std::mutex pcMutex;


void HUREL::Compton::RtabmapSlamControl::VideoStream()
{
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;


	mCameraThread = new rtabmap::CameraThread(mCamera);
	mCameraThread->start();
	mIsVideoStreamOn = true;

	while (mIsVideoStreamOn)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		
		if (data.isValid())
		{
			
			auto img = data.imageRaw();
			auto imgDepth = data.depthOrRightRaw();
			
			videoStreamMutex.lock();
			if (img.cols > 0)
			{			
				mCurrentVideoFrame = img;
			}
			if (imgDepth.cols > 0)
			{
				mCurrentDepthFrame = imgDepth;
			}
			if (mOdo != nullptr && &mOdo->getPose() != nullptr && mIsSlamPipeOn == true )
			{
			
				//mCurrentOdometry = mOdo->getPose().toEigen4d() * t265toLACCAxisTransform;
				videoStreamMutex.unlock();
			}
			else
			{
				videoStreamMutex.unlock();
			}

			//pcMutex.lock();
			//mRealtimePointCloud = *(rtabmap::util3d::cloudRGBFromSensorData(data, 4,           // image decimation before creating the clouds
			//	6.0f,        // maximum depth of the cloud
			//	0.5f));
			//pcMutex.unlock();
		}
	}

	mCameraThread->join(true);
	mCameraThread->kill();
	delete mCameraThread;
	mCameraThread = nullptr;
}


void HUREL::Compton::RtabmapSlamControl::ResetSlam()
{
	if (mOdo == nullptr)
	{
		return;
	}
	videoStreamMutex.lock();
	if (mOdo != nullptr)
	{
		mOdo->reset();
	}
	mOdoInit = false;	
	videoStreamMutex.unlock();
}

void HUREL::Compton::RtabmapSlamControl::GetIntrinsic(float* outfx, float* outfy, float* outcx, float* outcy)
{
	rtabmap::SensorData data = mCamera->takeImage();
	fxValue = *outfx = static_cast<float>(data.cameraModels()[0].fx());
	fyValue = *outfy = static_cast<float>(data.cameraModels()[0].fy());
	cxValue = *outcx = static_cast<float>(data.cameraModels()[0].cx());
	cyValue = *outcy = static_cast<float>(data.cameraModels()[0].cy());
}

void HUREL::Compton::RtabmapSlamControl::GetIntrinsic()
{
	rtabmap::SensorData data = mCamera->takeImage();
	fxValue = static_cast<float>(data.cameraModels()[0].fx());
	fyValue = static_cast<float>(data.cameraModels()[0].fy());
	cxValue = static_cast<float>(data.cameraModels()[0].cx());
	cyValue = static_cast<float>(data.cameraModels()[0].cy());
	std::cout << "fx Valus is: " << fxValue;
	std::cout << "fy Valus is: " << fyValue;
	std::cout << "cx Valus is: " << cxValue;
	std::cout << "cy Valus is: " << cyValue;
	
}

cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentVideoFrame()
{
	cv::Mat img;
	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{

			img = data.imageRaw();

			//get intrinsic of the mCamera
		}
	}
	
	return img;
}

cv::Mat HUREL::Compton::RtabmapSlamControl::GetCurrentDepthFrame()
{
	cv::Mat img;
	if (mIsVideoStreamOn && mCamera != nullptr)
	{
		rtabmap::SensorData data = mCamera->takeImage();
		if (data.isValid())
		{

			img = data.depthRaw();
			//raw depth to distance
			img.convertTo(img, CV_32F, 0.001);
		}
	}
	return img;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr HUREL::Compton::RtabmapSlamControl::generatePointCloud(cv::Mat &depth, cv::Mat &rgb)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (int m = 0; m < depth.rows; m++)
	{
		for (int n = 0; n < depth.cols; n++)
		{
			// Get 3D coordinates
			float d = depth.at<float>(m, n);
			if (d < 0)
			{
				//continue;
				d = 0;
			}
			pcl::PointXYZRGB point;
			point.z = d;
			float X = (n - cxValue) * point.z / fxValue;
			float Y = (m - cyValue) * point.z / fyValue;
			point.x = (-1)*X;
			point.y = (-1)*Y;
			cv::Vec3b color = rgb.at<cv::Vec3b>(m, n);
			point.b = color[0];
			point.g = color[1];
			point.r = color[2];
			cloud->points.push_back(point);
		}
	}
	cloud->height = 1;
	cloud->width = static_cast<uint32_t>(cloud->points.size());
	cloud->is_dense = false;
	return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr HUREL::Compton::RtabmapSlamControl::generatePointCloud(cv::Mat &depth, cv::Mat &rgb, float fx, float fy, float cx, float cy)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int m = 0; m < depth.rows; m++)
	{
		for (int n = 0; n < depth.cols; n++)
		{
			// Get 3D coordinates
			float d = depth.at<ushort>(m,n);
			if (d <= 0)
			{
				continue;
			}

			pcl::PointXYZRGB point;
			point.z = d;
			float X = (n - cx) * point.z / fx;
			float Y = (m - cy) * point.z / fy;
			point.x = (-1)*X;
			point.y = (-1)*Y;
			cv::Vec3b color = rgb.at<cv::Vec3b>(m, n);
			point.b = color[0];
			point.g = color[1];
			point.r = color[2];

			cloud->points.push_back(point);
		}
	}
	cloud->height = 1;
	cloud->width = static_cast<uint32_t>(cloud->points.size());
	cloud->is_dense = false;
	return cloud;
}

void HUREL::Compton::RtabmapSlamControl::LockVideoFrame()
{
	videoStreamMutex.lock();

}

void HUREL::Compton::RtabmapSlamControl::UnlockVideoFrame()
{
	videoStreamMutex.unlock();
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetRTPointCloud()
{
	pcMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mRealtimePointCloud;
	pcMutex.unlock();
	
	open3d::geometry::PointCloud tmpOpen3dPc;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	for (int i = 0; i < tmp.size(); ++i)
	{	
		if (isnan(tmp[i].x) || isinf(tmp[i].x))
		{
			continue;
		}
		Eigen::Vector3d color(tmp[i].r/255.0, tmp[i].g / 255.0, tmp[i].b / 255.0);
		Eigen::Vector3d point(tmp[i].x, tmp[i].y, tmp[i].z);
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(point); 
	}

	return tmpOpen3dPc.Transform(t265toLACCAxisTransform);
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetRTPointCloudTransposed()
{
	pcMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mRealtimePointCloud;
	pcMutex.unlock();
	open3d::geometry::PointCloud tmpOpen3dPc;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	for (int i = 0; i < tmp.size(); ++i)
	{
		if (isnan(tmp[i].x) || isinf(tmp[i].x))
		{
			continue;
		}
		Eigen::Vector3d color(tmp[i].b / 255.0, tmp[i].g / 255.0, tmp[i].r / 255.0);
		Eigen::Vector3d point(tmp[i].x, tmp[i].y, tmp[i].z);
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(point);
	}

	return tmpOpen3dPc.Transform(GetOdomentry()*t265toLACCAxisTransform);
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::RTPointCloudTransposed(open3d::geometry::PointCloud& initialPC, Eigen::Matrix4d transMatrix)
{
	open3d::geometry::PointCloud totalPCtrans;
	totalPCtrans = initialPC;
	totalPCtrans.Transform(transMatrix);
	Eigen::Matrix4d t265toLACCPosTranslate; Eigen::Matrix4d t265toLACCPosTransformInv;
	t265toLACCPosTranslate << 1, 0, 0, T265_TO_LAHGI_OFFSET_X,
		0, 1, 0, T265_TO_LAHGI_OFFSET_Y,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;
	t265toLACCPosTransformInv = t265toLACCPosTranslate.inverse();

	return totalPCtrans.Transform(t265toLACCPosTransformInv);
}

static std::future<void> t2;

void HUREL::Compton::RtabmapSlamControl::StartSlamPipe()
{
	if (!mIsVideoStreamOn || !mIsInitiate)
	{
		spdlog::error("C++::HUREL::Compton::RtabmapSlamControl: {0}", "RtabmapSlamControl fail to start slam pipe, Initiation failed");
	}
		
	if (mIsSlamPipeOn)
	{		
		spdlog::info("C++::HUREL::Compton::RtabmapSlamControl: {0}", "SLAM Pipe is already on");
		return;
	}
	spdlog::info("C++::HUREL::Compton::RtabmapSlamControl: {0}", "RtabmapSlamControl start slam pipe");
	
	auto func = std::bind(&RtabmapSlamControl::SlamPipe, this);
	t2 = std::async(std::launch::async, func);

}

void HUREL::Compton::RtabmapSlamControl::StopSlamPipe()
{
	if (mIsSlamPipeOn)
	{
		mIsSlamPipeOn = false;

		t2.get();
		spdlog::info("C++::HUREL::Compton::RtabmapSlamControl: {0}", "RtabmapSlamControl stop slam pipe");
		
	}
	else {
		spdlog::info("C++::HUREL::Compton::RtabmapSlamControl: {0}", "RtabmapSlamControl is alread stop");

	}
	

}

static std::mutex slamPipeMutex;

void HUREL::Compton::RtabmapSlamControl::SlamPipe()
{
	mOdo = rtabmap::Odometry::create();
	rtabmap::OdometryThread odomThread(mOdo);
	// Create RTAB-Map to process OdometryEvent
	rtabmap::Rtabmap* rtabmap = new rtabmap::Rtabmap();
	rtabmap->init();
	rtabmap::RtabmapThread rtabmapThread(rtabmap); // ownership is transfered
	// Setup handlers
	odomThread.registerToEventsManager();
	rtabmapThread.registerToEventsManager();
	
	UEventsManager::createPipe(mCameraThread, &odomThread, "CameraEvent");
	// Let's start the threads
	rtabmapThread.start();
	odomThread.start();

	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	GetIntrinsic();
	mIsSlamPipeOn = true;
	bool cutOffMode = false;
	bool shotSave = false;
	bool shotSaveSlamedPcl = false;
	while (mIsSlamPipeOn)
	{
		std::map<int, rtabmap::Signature> nodes;
		std::map<int, rtabmap::Transform> optimizedPoses;
		std::multimap<int, rtabmap::Link> links;
		if (!mOdoInit)
		{
			videoStreamMutex.lock();
			rtabmap->resetMemory();
			mInitOdo = t265toLACCAxisTransform* mOdo->getPose().toEigen4d() ;
			mOdoInit = true;
			videoStreamMutex.unlock();
			continue;
		}
		std::map<int, rtabmap::Signature> tmpnodes;
		std::map<int, rtabmap::Transform> tmpOptimizedPoses;
		
		rtabmap->getGraph(tmpOptimizedPoses, links, true, true, &tmpnodes, true, true, true, false);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		int i = 0;
		rtabmap->getStatistics();

		nodes = tmpnodes;
		optimizedPoses = tmpOptimizedPoses;
		std::vector < Eigen::Matrix4d> tempPoses;
		tempPoses.reserve(optimizedPoses.size());
		int k = 0;
		int count = optimizedPoses.size();
		//https://cpp.hotexamples.com/examples/-/Rtabmap/-/cpp-rtabmap-class-examples.html
		for (std::map<int, rtabmap::Transform>::iterator iter = optimizedPoses.begin(); iter != optimizedPoses.end(); ++iter)
		{
			++k;
			if (nodes.count(iter->first) == 0)
			{
				continue;
			}
			if (cutOffMode && k < count - 20)
			{
				continue;
			}

			rtabmap::Signature node = nodes.find(iter->first)->second;
			// uncompress data
			node.sensorData().uncompressData();

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp = rtabmap::util3d::cloudRGBFromSensorData(
				node.sensorData(),
				4,           // image decimation before creating the clouds
				6.0f,        // maximum depth of the cloud
				0.3f);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
			std::vector<int> index;
			pcl::removeNaNFromPointCloud(*tmp,  *tmpNoNaN, index);

			if (!tmpNoNaN->empty())
			{
				*cloud += *rtabmap::util3d::transformPointCloud(tmpNoNaN, iter->second); // transform the point cloud to its pose
			}
			tempPoses.push_back(t265toLACCAxisTransform * iter->second.toEigen4d()  );
			++i;
			tmpNoNaN.reset();
			if (k == count)
			{
				cv::Mat rgbImage = node.sensorData().imageRaw();
				cv::Mat depthImage = node.sensorData().depthRaw();
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr gencloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				//*gencloud = *generatePointCloud(depthImage, rgbImage, fxValue, fyValue, cxValue, cyValue);
				//*gencloud = *generatePointCloud(depthImage, rgbImage);
				///mRealtimePointCloud = *gencloud;
				//mCurrentPointCloud = *gencloud;
				open3d::geometry::PointCloud tmpOpen3dPc;

				if (shotSave)
				{
					std::chrono::milliseconds timeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
					std::string fileName = std::to_string(timeInMili.count());
					rtabmap::SensorData data = mCamera->takeImage();
					const char* home = getenv("HOME");
					std::string folderPath = "/Documents/hurel/Data/Session_1/";
					folderPath = home + folderPath;
					//make folder 
					std::string folderFullPath = folderPath + fileName;
//					cv::imwrite(folderFullPath + "_depth.png", depthImage);
//					cv::imwrite(folderFullPath + "_rgb.png", rgbImage);
					cv::imwrite(folderFullPath + "_depth.png", data.depthRaw());
					cv::imwrite(folderFullPath + "_rgb.png", data.imageRaw());
					open3d::io::WritePointCloudOption option;
					if (shotSaveSlamedPcl)
					{
						mSlamedPointCloud = *cloud;
						tmpOpen3dPc = GetSlamPointCloud();
					}
					else
					{
						mCurrentPointCloud = *gencloud;
						tmpOpen3dPc = RtabmapSlamControl::instance().PclToOpen3d(gencloud);
						//tmpOpen3dPc = GetCurrentPointCloud();
					}
					//open3d::io::WritePointCloudToPLY(folderFullPath + "_.ply", tmpOpen3dPc, option);
				}
			}
		}
		
		slamPipeMutex.lock();
		mSlamedPointCloud = *cloud;
		double gridWidth = 0; double gridHeight = 0; double minX = 0; double minZ = 0;
		float res = 0.05;
		CalOccupancySize(res, &gridWidth, &gridHeight, &minX, &minZ);
		mgridWith = gridWidth;
		mgridHeight = gridHeight;
		mminX = minX;
		mminZ = minZ;
		mOccupancyPCLGrid = createOccupancyPCL(res);

		mPoses = tempPoses;
		slamPipeMutex.unlock();
		uSleep(0);
	}
	mOdoInit = false;
	//mCurrentOdometry = Eigen::Matrix4d::Identity();
	mInitOdo = Eigen::Matrix4d::Identity();
	odomThread.join(true);
	rtabmapThread.join(true);
	odomThread.kill();
	rtabmapThread.kill();
}

open3d::geometry::LineSet HUREL::Compton::RtabmapSlamControl::GetPosePointCloudLineSet()
{
	if (!mIsSlamPipeOn)
	{
		return open3d::geometry::LineSet();
	}
	slamPipeMutex.lock();
	std::vector<Eigen::Matrix4d> tempPoses = mPoses;
	slamPipeMutex.unlock();
	auto posecloud = std::make_shared<open3d::geometry::PointCloud>();
	size_t pose_size = (tempPoses.size());

	if (pose_size< 1)
	{
        return open3d::geometry::LineSet();
    }

	for (size_t pose = 0; pose < pose_size; ++pose)
	{
		double pose_x = static_cast<double>(tempPoses[pose](0, 3));
        double pose_y = static_cast<double>(tempPoses[pose](1, 3)); 
        double pose_z = static_cast<double>(tempPoses[pose](2, 3));
        Eigen::Vector3d color(0, 1, 0);
        Eigen::Vector3d point(pose_x,0, pose_z);
        posecloud->colors_.emplace_back(color);
        posecloud->points_.emplace_back(point);
	}
	auto pathLines = std::make_shared<open3d::geometry::LineSet>();

	pathLines->points_ = posecloud->points_;
	pathLines->colors_ = posecloud->colors_;
	for (int i = 0; i < pose_size - 1; ++i)
	{
		//pathLines->lines_.emplace_back(open3d::geometry::Line(i, i +1));
		pathLines->lines_.emplace_back(i, i+1);
	}

	pathLines->colors_.resize(pathLines->lines_.size(), Eigen::Vector3d(0, 1, 0));
	return *pathLines;
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetSlamPointCloud()
{
	if (!mIsSlamPipeOn)
	{
		return open3d::geometry::PointCloud();
	}
	slamPipeMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mSlamedPointCloud;
	slamPipeMutex.unlock();

	open3d::geometry::PointCloud tmpOpen3dPc;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	for (int i = 0; i < tmp.size(); ++i)
	{
		Eigen::Vector3d color(tmp[i].r/255.0, tmp[i].g / 255.0, tmp[i].b / 255.0);
		//Eigen::Vector3d color(0, 0, 0);
		Eigen::Vector4d point(tmp[i].x, tmp[i].y, tmp[i].z, 1);
		Eigen::Vector4d transFormedpoint = t265toLACCAxisTransform * point;
	
		Eigen::Vector3d inputpoint(transFormedpoint.x(), transFormedpoint.y(), transFormedpoint.z());
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(inputpoint);
		
	}
	open3d::geometry::PointCloud returnPC = *tmpOpen3dPc.VoxelDownSample(0.1);
	return returnPC; 
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetFilteredSlamPointCloud()
{
	if (!mIsSlamPipeOn)
	{
		return open3d::geometry::PointCloud();
	}
	slamPipeMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mSlamedPointCloud;
	slamPipeMutex.unlock();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpNoNaN(new pcl::PointCloud<pcl::PointXYZRGB>);
	*tmpNoNaN  = tmp;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(tmpNoNaN);
	sor.setMeanK(30);
	sor.setStddevMulThresh(1.0);
	sor.filter(*tmpNoNaN);
	tmp = *tmpNoNaN;

	open3d::geometry::PointCloud tmpOpen3dPc;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	for (int i = 0; i < tmp.size(); ++i)
	{
		Eigen::Vector3d color(tmp[i].r/255.0, tmp[i].g / 255.0, tmp[i].b / 255.0);
		Eigen::Vector4d point(tmp[i].x, tmp[i].y, tmp[i].z, 1);
		Eigen::Vector4d transFormedpoint = t265toLACCAxisTransform * point;	
		Eigen::Vector3d inputpoint(transFormedpoint.x(), transFormedpoint.y(), transFormedpoint.z());
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(inputpoint);		
	}

	// print point count
	//open3d::geometry::PointCloud returnPC = *tmpOpen3dPc.VoxelDownSample(0.05);
	open3d::geometry::PointCloud returnPC = tmpOpen3dPc;
	
	return returnPC;
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetCurrentPointCloud()
{
	if (!mIsSlamPipeOn)
	{
		return open3d::geometry::PointCloud();
	}
	slamPipeMutex.lock();
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mCurrentPointCloud;
	slamPipeMutex.unlock();

	open3d::geometry::PointCloud tmpOpen3dPc;
	
	for (int i = 0; i < tmp.size(); ++i)
	{
		Eigen::Vector3d color(tmp[i].r/255.0, tmp[i].g / 255.0, tmp[i].b / 255.0);
		Eigen::Vector4d point(tmp[i].x, tmp[i].y, tmp[i].z, 1);
		Eigen::Vector3d inputpoint(point.x(), point.y(), point.z());
		tmpOpen3dPc.colors_.push_back(color);
		tmpOpen3dPc.points_.push_back(inputpoint);		
	}
	
//	open3d::geometry::PointCloud returnPC = *tmpOpen3dPc.VoxelDownSample(0.1);
	open3d::geometry::PointCloud returnPC = tmpOpen3dPc;

	return returnPC;
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::PclToOpen3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud = *tmp;
	open3d::geometry::PointCloud tmpOpen3dPc;
	for(auto cloud_it = cloud->begin(); cloud_it!= cloud->end(); ++cloud_it)
	{
		Eigen::Vector3d color(cloud_it->r/255.0, cloud_it->g / 255.0, cloud_it->b / 255.0);
        Eigen::Vector4d point(cloud_it->x, cloud_it->y, cloud_it->z, 1);
        Eigen::Vector3d inputpoint(point.x(), point.y(), point.z());
        tmpOpen3dPc.colors_.push_back(color);
        tmpOpen3dPc.points_.push_back(inputpoint);
	}
//	open3d::geometry::PointCloud returnPC = *tmpOpen3dPc.VoxelDownSample(0.1);
	open3d::geometry::PointCloud returnPC = tmpOpen3dPc;
	return returnPC;
}

void HUREL::Compton::RtabmapSlamControl::CalOccupancySize(float res, double* outWidth, double* outHeight, double* outMinX, double* outMinZ)
{
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mSlamedPointCloud;
	const double Y_threshold = 0.5 ;
	double minX = 0; double maxX = 0; double minZ = 0; double maxZ = 0;
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;
	open3d::geometry::PointCloud tmpOpen3dPc;
	for (const auto& point : tmp.points)
	{	
		Eigen::Vector4d transbeforepoint(point.x, point.y, point.z, 1);
		Eigen::Vector4d transFormedpoint = t265toLACCAxisTransform * transbeforepoint;	
		if(transFormedpoint.y() < Y_threshold && transFormedpoint.y() > -0.1)
		{
			minX = std::min(minX, transFormedpoint.x());
			maxX = std::max(maxX, transFormedpoint.x());
			minZ = std::min(minZ, transFormedpoint.z());
			maxZ = std::max(maxZ, transFormedpoint.z());
			Eigen::Vector3d inputpoint(transFormedpoint.x(), transFormedpoint.y(), transFormedpoint.z());
			tmpOpen3dPc.points_.push_back(inputpoint);
		}
	}
	int GRID_WIDTH = *outWidth = static_cast<int>(maxX - minX)/res;
	int GRID_HEIGHT = *outHeight = static_cast<int>(maxZ - minZ)/res;

	double X_range; double Z_range;
	if (abs(minX) < abs(maxX))
	{
		X_range = abs(minX); 
	}
	else
	{
        X_range = abs(maxX);
    } 
	if (abs(minZ) < abs(maxZ))
	{
		Z_range = abs(minZ);
	}
	else
	{
		Z_range = abs(maxZ);
	}
	*outMinX = X_range;
	*outMinZ = Z_range;
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::createOccupancyPCL(float res)
{
	pcl::PointCloud<pcl::PointXYZRGB> tmp = mSlamedPointCloud;

	const double Y_threshold = 0.5 ;
	
	Eigen::Matrix4d t265toLACCAxisTransform;
	t265toLACCAxisTransform << 0, 1, 0, 0,
		0, 0, 1, 0,
		1, 0, 0, 0,
		0, 0, 0, 1;

	open3d::geometry::PointCloud tmpOpen3dPc;
	for (const auto& point : tmp.points)
	{	
		Eigen::Vector4d transbeforepoint(point.x, point.y, point.z, 1);
		Eigen::Vector4d transFormedpoint = t265toLACCAxisTransform * transbeforepoint;	
		if(transFormedpoint.y() < Y_threshold && transFormedpoint.y() > -0.1)
		{
			Eigen::Vector3d inputpoint(transFormedpoint.x(), transFormedpoint.y(), transFormedpoint.z());
			tmpOpen3dPc.points_.push_back(inputpoint);
		}
	}
	
	cv::Mat grid(mgridHeight, mgridWith, CV_8UC3, cv::Scalar(255, 255, 255));

	open3d::geometry::PointCloud gridCloud;
	Eigen::Vector3d point;
	Eigen::Vector3d color;
	for (size_t k = 0; k < tmpOpen3dPc.points_.size(); ++k)
	{
		if(tmpOpen3dPc.points_[k].y() < Y_threshold && tmpOpen3dPc.points_[k].y() > -0.2)
		{
			int x = static_cast<int>((tmpOpen3dPc.points_[k].x() + abs(mminX))/res); 
            int z = static_cast<int>((tmpOpen3dPc.points_[k].z() + abs(mminZ))/res);
			if (grid.at<cv::Vec3b>(z, x) == cv::Vec3b(0, 0, 0))
			{
				continue;
			}
            if (x < 0 || x >= mgridWith || z < 0 || z >= mgridHeight)
            {
                continue;
            }
            else
            {
                grid.at<cv::Vec3b>(z, x) = cv::Vec3b(0, 0, 0);
				point(0) = tmpOpen3dPc.points_[k].x();
				point(1) = 0.5;
				point(2) = tmpOpen3dPc.points_[k].z();
				color(0) = 0;
				color(1) = 0;
				color(2) = 0;
				gridCloud.points_.push_back(point);
				gridCloud.colors_.push_back(color);
            }
		}	
	}
	return gridCloud;
}

void HUREL::Compton::RtabmapSlamControl::PushBackIsotopeData(std::string IsotopeName, double maxLocx, double maxLocy, double maxLocz, double maxValue)
{
	sIsotopePCL temp;
	temp.IsotopeName = IsotopeName;
	temp.maxLocx = maxLocx;
	temp.maxLocy = maxLocy;
	temp.maxLocz = maxLocz;
	temp.maxValue = maxValue;
	mIsotopePCLList.push_back(temp);	
}

/*
open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::createSourceOccupancyPCL(int sourcetype, double value_matrix, double point matrix)
{
	const double Y_threshold = 0.5 ;
	double res = 0.05;
	open3d::geometry::PointCloud gridCloud;
	cv::Mat grid(mgridHeight, mgridWith, CV_8UC3, cv::Scalar(255, 255, 255));

	for (선원 갯수마다)
	{
		각 선원마다의 최대값 구한다.
	}
	for (선원 갯수마다)
	{
		각 선원마다의 최대값으로 영상값 나누기
		나눈 영상값이 cutoff 기준보다 낮으면 0으로 바꾸기
	}
	color list[3][3];
	color list[0] = [255,0,0];
	color list[1] = [0,255,0];
	color list[2] = [0,0,255];
	for (선원 갯수마다 l)
	{
		for (size_t k = 0; k < 선원마다 VALUE matrix size; ++k)
		{	
			if(point_matrix.points_[k].y() < Y_threshold && tmpOpen3dPc.points_[k].y() > -0.2)
			{
				int x = static_cast<int>((point_matrix.points_[k].x() + abs(mminX))/res); 
            	int z = static_cast<int>((point_matrix.points_[k].z() + abs(mminZ))/res);
			}
			if (grid.at<cv::Vec3b>(z, x) != cv::Vec3b(255, 255, 255))
			{
				continue;
			}
            if (x < 0 || x >= mgridWith || z < 0 || z >= mgridHeight)
            {
                continue;
            }
			else
			{
				grid.at<cv::Vec3b>(z, x) = cv::Vec3b(0, 0, 0);
				point(0) = point_matrix.points_[k].x();
				point(1) = 0.5;
				point(2) = point_matrix.points_[k].z();
				color(0) = color list[l][0];
				color(1) = color list[l][1];
				color(2) = color list[l][2];
				gridCloud.points_.push_back(point);
				gridCloud.colors_.push_back(color);
			}
		}
	}
	return gridCloud;
}
*/

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetOccupancyPointCloud()
{
	return mOccupancyPCLGrid;
}

bool HUREL::Compton::RtabmapSlamControl::LoadPlyFile(std::string filePath)
{
	open3d::io::ReadPointCloudOption opt;
	return true;
	//return open3d::io::ReadPointCloudFromPLY(filePath, mLoadedPointcloud, opt);
}

open3d::geometry::PointCloud HUREL::Compton::RtabmapSlamControl::GetLoadedPointCloud()
{
	return mLoadedPointcloud;
}

Eigen::Matrix4d HUREL::Compton::RtabmapSlamControl::GetOdomentry()
{
	Eigen::Matrix4d odo = Eigen::Matrix4d::Identity();;
	
	//videoStreamMutex.lock();
	if (mOdo != nullptr && &mOdo->getPose() != nullptr && mIsSlamPipeOn == true)
	{
		odo =  mOdo->getPose().toEigen4d() ;
	}
	//videoStreamMutex.unlock();
	return odo;
}

std::vector<Eigen::Matrix4d> HUREL::Compton::RtabmapSlamControl::GetOptimizedPoses()
{
	slamPipeMutex.lock();
	std::vector<Eigen::Matrix4d> tempPoses = mPoses;
	slamPipeMutex.unlock();
	return tempPoses;
}
