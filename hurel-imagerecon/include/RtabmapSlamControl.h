#pragma once


#include "rtabmap/core/util3d.h"
#include "rtabmap/core/util3d_filtering.h"
#include "rtabmap/core/util3d_transforms.h"
#include "rtabmap/core/RtabmapEvent.h"
#include "rtabmap/core/OccupancyGrid.h"

#include "rtabmap/utilite/UStl.h"
#include "rtabmap/utilite/UConversion.h"
#include "rtabmap/utilite/UEventsHandler.h"
#include "rtabmap/utilite/ULogger.h"
#include "rtabmap/core/OdometryEvent.h"
#include "rtabmap/core/CameraThread.h"

#include <iostream>
#include <algorithm>

#include <rtabmap/core/Odometry.h>
#include "rtabmap/core/Rtabmap.h"
#include "rtabmap/core/RtabmapThread.h"
#include "rtabmap/core/camera/CameraRealSense2.h"
#include "rtabmap/core/camera/CameraRGBDImages.h"
#include "rtabmap/core/CameraStereo.h"
#include "rtabmap/core/CameraThread.h"
#include "rtabmap/core/OdometryThread.h"
#include "rtabmap/core/Graph.h"
#include "rtabmap/utilite/UEventsManager.h"
#include "rtabmap/core/Parameters.h"
#include "rtabmap/core/CameraRGBD.h"
#include "rtabmap/core/Signature.h"
#include "rtabmap/core/OccupancyGrid.h"

#include <open3d/geometry/PointCloud.h>
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/io/PointCloudIO.h>
#include <open3d/geometry/LineSet.h>

#include <librealsense2/rs.hpp>


#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>


#include <spdlog/spdlog.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>


//#define T265_TO_LAHGI_OFFSET_X (-0.0)
//#define T265_TO_LAHGI_OFFSET_Y (-0.35)
//#define T265_TO_LAHGI_OFFSET_Z (-0.40)
//고방복탐
#define T265_TO_LAHGI_OFFSET_X (0.295)
#define T265_TO_LAHGI_OFFSET_Y (-0.035)
#define T265_TO_LAHGI_OFFSET_Z (-0.100)


#define T265_To_Mask_OFFSET_X (T265_TO_LAHGI_OFFSET_X)
#define T265_To_Mask_OFFSET_Y (T265_TO_LAHGI_OFFSET_Y)
#define T265_To_Mask_OFFSET_Z (0.00)

namespace HUREL
{
	namespace Compton
	{

		class RtabmapSlamControl
		{

		private:
			
			Eigen::Matrix4d mInitOdo = Eigen::Matrix4d::Identity();

			rtabmap::CameraRealSense2* mCamera = nullptr;
			rtabmap::CameraThread* mCameraThread = nullptr;
			rtabmap::Odometry* mOdo = nullptr;// = rtabmap::Odometry::create();;
			
			cv::Mat mCurrentVideoFrame = cv::Mat();
			cv::Mat mCurrentDepthFrame = cv::Mat();
			void VideoStream();

			pcl::PointCloud<pcl::PointXYZRGB> mRealtimePointCloud = pcl::PointCloud<pcl::PointXYZRGB>();
			pcl::PointCloud<pcl::PointXYZRGB> mSlamedPointCloud = pcl::PointCloud<pcl::PointXYZRGB>();
			pcl::PointCloud<pcl::PointXYZRGB> mCurrentPointCloud = pcl::PointCloud<pcl::PointXYZRGB>();

			open3d::geometry::PointCloud mOccupancyPCLGrid;
			cv::Mat mOccupancyGrid;

			void SlamPipe();

			RtabmapSlamControl();

			open3d::geometry::PointCloud mLoadedPointcloud = open3d::geometry::PointCloud();
			std::vector < Eigen::Matrix4d> mPoses = std::vector<Eigen::Matrix4d>();
			void LockVideoFrame();

			void UnlockVideoFrame();
			void LockDepthFrame();

			void UnlockDepthFrame();

			float	fxValue;
			float	fyValue;
			float	cxValue;
			float	cyValue;

			double mgridWith;
			double mgridHeight;
			double mminX;
			double mminZ;


		public:
			bool mIsInitiate = false;
			bool mIsVideoStreamOn = false;
			bool mIsSlamPipeOn = false;
			bool mOdoInit = false;

			int rowSize;
			int colSize;

			struct sIsotopePCL
			{
				std::string IsotopeName;
				double maxLocx;
				double maxLocy;
				double maxLocz;
				double maxValue;
			};
			std::vector<sIsotopePCL> mIsotopePCLList = std::vector<sIsotopePCL>();

			bool Initiate();

			void PushBackIsotopeData(std::string IsotopeName, double maxLocx, double maxLocy, double maxLocz, double maxValue);

			void GetIntrinsic(float* outfx, float* outfy, float* outcx, float* outcy);
			void GetIntrinsic();

			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			Eigen::Matrix4d GetOdomentry();

			std::vector< Eigen::Matrix4d> GetOptimizedPoses();

			cv::Mat GetCurrentVideoFrame();
			cv::Mat GetCurrentDepthFrame();

			void CalOccupancySize(float res, double* outWidth, double* outHeight, double* outMinX, double* outMinZ);
			open3d::geometry::PointCloud createOccupancyPCL(float res);
	

			open3d::geometry::PointCloud PclToOpen3d(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp);

			open3d::geometry::PointCloud GetRTPointCloud();
			open3d::geometry::PointCloud GetRTPointCloudTransposed();
			open3d::geometry::PointCloud RTPointCloudTransposed(open3d::geometry::PointCloud& initialPC, Eigen::Matrix4d transMatrix);

			void StartVideoStream();
			void StopVideoStream();
			
			void StartSlamPipe();
			void StopSlamPipe();

			void ResetSlam();

			open3d::geometry::PointCloud GetSlamPointCloud();
			open3d::geometry::PointCloud GetPosePointCloud();
			open3d::geometry::LineSet GetPosePointCloudLineSet();
			open3d::geometry::PointCloud GetCurrentPointCloud();
			open3d::geometry::PointCloud GetFilteredSlamPointCloud();
			open3d::geometry::PointCloud GetOccupancyPointCloud();

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloud(cv::Mat &depth, cv::Mat &rgb, float fx, float fy, float cx, float cy);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr generatePointCloud(cv::Mat &depth, cv::Mat &rgb);
			
			bool LoadPlyFile(std::string filePath);

			open3d::geometry::PointCloud GetLoadedPointCloud();
		public:

			static RtabmapSlamControl& instance();
			~RtabmapSlamControl();
		};


	};
};
