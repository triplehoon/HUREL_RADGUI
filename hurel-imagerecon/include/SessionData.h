#pragma once

#include <opencv2/opencv.hpp>

#include <tbb/concurrent_vector.h>
#include <tbb/concurrent_queue.h>

#include <thread>

#include "ReconPointCloud.h"
#include "EnergySpectrum.h"
#include "ListModeData.h"

namespace HUREL
{
    namespace Compton
    {
     
        class SessionData
        {

        private:
            std::mutex mResetListModeDataMutex;
            std::mutex mResetEnergySpectrumMutex;

            // data
            tbb::concurrent_vector<ListModeData> mListedListModeData;
            tbb::concurrent_vector<EnergyTimeData> mListedEnergyTimeData;

            size_t UpdateInteractionImageListedListModeDataIndex = 0;
            std::vector<std::vector<sInteractionData>> mInteractionImages;

            ReconPointCloud mReconPointCloud;
            open3d::geometry::PointCloud mImageSpacePointCloud;

            bool mIsLoaded;
            std::string mLoadFileDir;

            std::chrono::milliseconds mLoadedDataStartTime;
            std::chrono::milliseconds mLoadedDataLastTime;

            EnergySpectrum mEnergySpectrums[16];

            void SetNeedToUpatesAsTrue();
            bool mIsNeedToUpdateCountRate;

            std::vector<double> mCountRate;

            // static functions
            static size_t GetIndexOfTime(tbb::concurrent_vector<ListModeData> &listedModeData, long long traget);
            static size_t GetIndexOfTime(tbb::concurrent_vector<EnergyTimeData> &listedModeData, long long traget);

        public:
            SessionData();
            SessionData(std::string fileDir);

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

            std::vector<std::vector<sInteractionData>> GetInteractionImages();

            std::vector<EnergyTimeData> GetListedEnergyTimeData();
            std::vector<EnergyTimeData> GetListedEnergyTimeData(long long timeInMiliseconds);

            // return the cout rate every a seconds
            std::vector<double> GetCountRate();

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

            size_t GetSizeListedListModeData();
            size_t GetSizeListedEnergyTimeData();

            cv::Mat mRgbImage;
            cv::Mat mDepthImage;
        };
    } // namespace Compton

} // namespace HUREL
