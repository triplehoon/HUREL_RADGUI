#pragma once

#include <opencv2/opencv.hpp>

#include <tbb/concurrent_vector.h>
#include <tbb/concurrent_queue.h>

#include <thread>
#include <cassert>

#include "ReconPointCloud.h"
#include "EnergySpectrum.h"
#include "ListModeData.h"
#include "RtabmapSlamControl.h"

namespace HUREL
{
    namespace Compton
    {
     
        class SessionData
        {

        private:
            std::mutex mResetListModeDataMutex;
            std::mutex mListModeDataMutex;

            // data
            tbb::concurrent_vector<ListModeData> mListedListModeData;
            tbb::concurrent_vector<EnergyTimeData> mListedEnergyTimeData;

            size_t UpdateInteractionImageListedListModeDataIndex = 0;
            tbb::concurrent_vector<std::pair<sEnergyCheck, std::vector<sInteractionData>>> mInteractionImages;

            // recon
            ReconPointCloud mReconPointCloud;
            open3d::geometry::PointCloud mImageSpacePointCloud;

            bool mIsLoaded;
            std::string mLoadFileDir;

            std::chrono::milliseconds mLoadedDataStartTime;
            std::chrono::milliseconds mLoadedDataLastTime;

            EnergySpectrum mEnergySpectrums[16];

            void SetNeedToUpatesAsTrue();

            std::vector<double> mCountRate = std::vector<double>();
            std::map<sEnergyCheck, std::vector<double>> mCountRateByEnergyCheck = std::map<sEnergyCheck, std::vector<double>>();

            // static functions
            static size_t GetIndexOfTime(tbb::concurrent_vector<ListModeData> &listedModeData, long long traget);
            static size_t GetIndexOfTime(tbb::concurrent_vector<EnergyTimeData> &listedModeData, long long traget);

            static bool IsSame(ListModeData &lmData, sInteractionData& interactionData, double timeDifference = 10, Eigen::Vector3d positionDifference = Eigen::Vector3d(0.1, 0.1, 0.1));
        public:
            SessionData();
            SessionData(std::string fileDir);
            ~SessionData();

            SessionData(const SessionData &other) = delete;
            SessionData &operator=(const SessionData &other) = delete;

            bool Save(std::string fileDir);

            std::string GetLoadFileDir();

            std::vector<ListModeData> GetListedListModeData();
            std::vector<ListModeData> GetListedListModeData(sEnergyCheck echk);
            std::vector<ListModeData> GetListedListModeData(std::chrono::milliseconds timeInMiliseconds);
            std::vector<ListModeData> GetListedListModeData(std::chrono::milliseconds timeInMiliseconds, sEnergyCheck echk);
            std::vector<ListModeData> GetListedListModeData(size_t count);
            std::vector<ListModeData> GetListedListModeData(size_t count, sEnergyCheck echk);
            
            std::vector<sInteractionData> GetInteractionData(sEnergyCheck echk, std::chrono::milliseconds timeInMiliseconds = std::chrono::milliseconds(0));
            std::vector<sInteractionData> GetInteractionData(sEnergyCheck echk, size_t count);
            std::vector<sInteractionData> GetInteractionData(IsotopeData iso, std::chrono::milliseconds timeInMiliseconds = std::chrono::milliseconds(0));
            std::vector<sInteractionData> GetInteractionData(IsotopeData iso, size_t count);

            std::vector<std::pair<sEnergyCheck, std::vector<sInteractionData>>> GetInteractionImages();

            std::vector<sEnergyCheck> GetEnergyCheckList();


            std::vector<EnergyTimeData> GetListedEnergyTimeData();
            std::vector<EnergyTimeData> GetListedEnergyTimeData(long long timeInMiliseconds);

            // return the cout rate every 500 ms
            std::vector<double> GetCountRateEvery500ms();
            std::vector<double> GetCountRateEvery500ms(sEnergyCheck echk);

            

            std::chrono::milliseconds GetStartTime();
            std::chrono::milliseconds GetEndTime();

            void LockMutexAddingData();
            void AddEnergyTime(EnergyTimeData &datum);
            void AddListModeData(ListModeData &datum);
            void UnlockMutexAddingData();
            void UpdateInteractionImage();

            void Reset();
            void ResetSpectrum();
            void ResetSpectrum(size_t fpgaChannel);

            EnergySpectrum GetEnergySpectrum(size_t fpgaChannel);
            EnergySpectrum GetEnergySpectrum(int *fpageChannels, int size, long long timeInMiliseconds);
            EnergySpectrum GetScatterEnergySpectrum(long long timeInMiliseconds = -1);
            EnergySpectrum GetAbsroberEnergySpectrum(long long timeInMiliseconds = -1);

            

            size_t GetSizeListedListModeData();
            size_t GetSizeListedEnergyTimeData();
            cv::Mat GetRGBImage();
            cv::Mat GetDepthImage();
            open3d::geometry::PointCloud GetPointCloud();
            open3d::geometry::PointCloud GetSlamPointCloud();
            open3d::geometry::PointCloud GetOccupancyPointCloud();
            Eigen::Matrix4d GetTransMatrix();

            cv::Mat mRgbImage;
            cv::Mat mDepthImage;
            open3d::geometry::PointCloud mPointCloud;
            open3d::geometry::PointCloud mSlamPointCloud;
            open3d::geometry::PointCloud mOccupancyPointCloud;
            Eigen::Matrix4d mTransMatrix;

        };
    } // namespace Compton

} // namespace HUREL
