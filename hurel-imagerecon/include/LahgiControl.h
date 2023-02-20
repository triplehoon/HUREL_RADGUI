#pragma once

#include <vector>

// #include "RealsenseControl.h"

#include "RtabmapSlamControl.h"

#include "spdlog/spdlog.h"

#include "Module.h"
#include "ListModeData.h"
#include "ReconPointCloud.h"
#include "SessionData.h"

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <tbb/concurrent_vector.h>
#include <tbb/concurrent_queue.h>
#include <thread>
#include <future>
#include <array>

#define ACTIVE_AREA_LENGTH 0.

namespace HUREL
{
	namespace Compton
	{
		struct sEnergyCheck
		{
			double minE;
			double maxE;
		};

		class LahgiControl
		{
		private:
			Module **mScatterModules;  //./Module information/MONOScatter1/Gain.csv, LUT.csv ...
			Module **mAbsorberModules; //./Module information/QUADScatter1/Gain.csv, LUT.csv ...
			eMouduleType mModuleType;
			// tbb::concurrent_vector <ListModeData> mListedListModeData;
			// tbb::concurrent_vector <EnergyTimeData> mListedEnergyTimeData;

			LahgiControl();
			inline static ListModeData MakeListModeData(const eInterationType &iType, Eigen::Vector4d &scatterPoint, Eigen::Vector4d &absorberPoint, double &scatterEnergy, double &absorberEnergy, Eigen::Matrix4d &transformation, std::chrono::milliseconds &timeInMili);
			inline static ListModeData MakeListModeData(const eInterationType &iType, Eigen::Vector4d &scatterPoint, Eigen::Vector4d &absorberPoint, double &scatterEnergy, double &absorberEnergy, Eigen::Matrix4d &transformation);
			// CodeMaks Setting
			double mMaskThickness = 0.006;

			tbb::concurrent_queue<std::array<unsigned short, 144>> mShortByteDatas;
			std::future<void> ListModeDataListeningThread;
			std::mutex eChksMutex;
			bool mIsListModeDataListeningThreadStart = false;

			void ListModeDataListening();

			double mListModeImgInterval;

			std::vector<sEnergyCheck> eChk;

			SessionData mLiveSessionData;

			void AddListModeDataWithTransformationLoop(std::array<unsigned short, 144> byteData, std::chrono::milliseconds &timeInMili, Eigen::Matrix4d &deviceTransformation);
			/*
			void AddListModeData(const unsigned short(byteData)[144], Eigen::Matrix4d deviceTransformation);
			void AddListModeDataWithTransformation(const unsigned short byteData[144]);
			void AddListModeDataWithTransformationVerification(const unsigned short byteData[]);
			*/
		
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			static LahgiControl &instance();

			void SetEchk(std::vector<sEnergyCheck> eChksInput);

			bool SetType(eMouduleType type);

			~LahgiControl();

			eMouduleType GetDetectorType();

			std::vector<ListModeData> GetListedListModeData();
			std::vector<ListModeData> GetListedListModeData(long long timeInMililseconds);

			std::vector<EnergyTimeData> GetListedEnergyTimeData();
			std::vector<EnergyTimeData> GetListedEnergyTimeData(long long timeInMililseconds);

			size_t GetListedListModeDataSize();

			void ResetListedListModeData();
			void SaveListedListModeData(std::string fileDir);

			EnergySpectrum &GetEnergySpectrum(int fpgaChannelNumber);
			EnergySpectrum GetSumEnergySpectrum();
			EnergySpectrum GetAbsorberSumEnergySpectrum();
			EnergySpectrum GetScatterSumEnergySpectrum();

			std::tuple<double, double, double> GetEcalValue(int fpgaChannelNumber);
			void SetEcalValue(int fpgaChannelNumber, std::tuple<double, double, double> ecal);
			void ResetEnergySpectrum();
			void ResetEnergySpectrum(int fpgaChannelNumber);

			ReconPointCloud GetReconRealtimePointCloudComptonUntransformed(open3d::geometry::PointCloud &pc, double time);
			ReconPointCloud GetReconRealtimePointCloudCompton(open3d::geometry::PointCloud &pc, double time);

			ReconPointCloud GetReconOverlayPointCloudCoded(open3d::geometry::PointCloud &pc, double time);
			ReconPointCloud GetReconOverlayPointCloudCompton(open3d::geometry::PointCloud &pc, double time);
			ReconPointCloud GetReconOverlayPointCloudHybrid(open3d::geometry::PointCloud &pc, double time);

			cv::Mat GetResponseImage(int imgSize, int pixelCount = 80, double timeInSeconds = 0, bool isScatter = true);

			Eigen::Matrix4d t265toLACCPosTransform;
			Eigen::Matrix4d t265toLACCPosTransformInv;
			Eigen::Matrix4d t265toLACCPosTranslate;
		};
	}
}
