#pragma once

#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>


#include <opencv2/highgui.hpp>

#include <open3d/geometry/Octree.h>

#include <omp.h>

#include "ListModeData.h"
#include "ReconPointCloud.h"
#include "spdlog/spdlog.h"



namespace HUREL {
	namespace Compton
	{
		using namespace cv;
		using namespace Eigen;		
	
		const static bool mCodeMask[37][37] = {
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true,true,false,true,true,false,false,false,false,true,false,true,false,true,true,true,true,false,false,true},
		{true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false,true,true,false,false,true,true,true,true,false,true,false,true,false,false,false,false,true,true,false},
		};
		
		enum class eRadiationImagingMode
		{
			CODED,
			COMPTON,
			HYBRID
		};

		constexpr double RadiationImageFOV = 130;
		constexpr double RadiationImageSeperateDegree = 5;
		constexpr double RadiationImageResponseSurfaceDistance = 3;

		class RadiationImage
		{
		private:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		public:	
			
			std::vector<ListModeData> mListedListModeData;			
			Eigen::Matrix4d mDetectorTransformation;


			//		  ^
			//<-- x   | 
			//        y 
			Mat mDetectorResponseImage;

			//		  ^
			//<-- x   | 
			//        y 
			Mat mComptonImage;
			Mat mCodedImage;
			Mat mHybridImage;

		    static void ShowCV_32SAsJet(cv::Mat img, int size);
			static cv::Mat GetCV_32SAsJet(cv::Mat img, int size);
			static cv::Mat GetCV_32SAsJet(cv::Mat img, int sizeh, int sizew, double minValuePortion, double opacity);
			RadiationImage() {};
			RadiationImage(std::vector<ListModeData>& data);			
			RadiationImage(std::vector<sInteractionData>& dataVector, double s2M, double resImprov, double m2D, double hFov, double wFov);
			RadiationImage(std::vector<sInteractionData>& dataVector, double s2M, double resImprov, double hFov, double wFov);
			RadiationImage(std::vector<sInteractionData> &dataVector, double s2M, double resImprov, open3d::geometry::PointCloud& reconPointCloud, double* outmaxValue, double* outmaxLocx, double* outmaxLocy, double* outmaxLocz);
			RadiationImage(std::vector<sInteractionData> &dataVector, double s2M, double resImprov, open3d::geometry::PointCloud& reconPointCloud, Eigen::Matrix4d transMatrix, double* outmaxValue, double* outmaxLocx, double* outmaxLocy, double* outmaxLocz);
			
			double OverlayValue(Eigen::Vector3d point, eRadiationImagingMode mode);
		};
	};
};

