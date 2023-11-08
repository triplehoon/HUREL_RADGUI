#include "SessionData.h"

using namespace HUREL;
using namespace Compton;

inline int findPositionIndex(double value, double min, double max, int spaceCount)
{
    double pixelSize = (max - min) / spaceCount;
    value += pixelSize / 2;
    if (value - min <= 0)
    {
        return -1;
    }
    if (value - max >= 0)
    {
        return -1;
    }
    int pValue = static_cast<int>(round((value - min) / pixelSize));
    return pValue == spaceCount ? spaceCount - 1: pValue;
}

void HUREL::Compton::SessionData::SetNeedToUpatesAsTrue()
{
}

HUREL::Compton::SessionData::SessionData() : mIsLoaded(false),
                                             mLoadedDataStartTime(std::chrono::milliseconds::zero()),
                                             mLoadFileDir(""),
                                             mListedEnergyTimeData(0),
                                             mListedListModeData(0),
                                             mCountRate(std::vector<double>(0))
{
    mListedEnergyTimeData.reserve(1000000);
    mListedListModeData.reserve(1000000);
    for (int i = 0; i < 16; i++)
    {
        mEnergySpectrums[i] = EnergySpectrum();
    }
}

HUREL::Compton::SessionData::SessionData(std::string fileDir) : mIsLoaded(true),
                                                                mCountRate(std::vector<double>(0))
{
    for (int i = 0; i < 16; i++)
    {
        mEnergySpectrums[i] = EnergySpectrum();
    }
    std::ifstream loadFile;
    std::string listmodeFileName = fileDir + "/LmData.csv";
    loadFile.open(listmodeFileName);
    if (!loadFile.is_open())
    {
        spdlog::error("Load list mode data fail: fail to open the file");

        loadFile.close();
        return;
    }
    mListedListModeData.clear();
    std::string buffer;
    char line[2048];
    while (loadFile.good())
    {
        ListModeData temp;
        std::getline(loadFile, buffer);
        if (temp.ReadListModeData(buffer))
        {
            mListedListModeData.push_back(temp);
        }
    }

    loadFile.close();

    std::string energyFileName = fileDir + "/LmEnergyData.csv";

    loadFile.open(energyFileName);
    if (!loadFile.is_open())
    {
        spdlog::error("Load list mode data fail: fail to open the file");

        loadFile.close();
        return;
    }
    
    while (loadFile.good())
    {
        ListModeData temp;
        std::getline(loadFile, buffer);
        std::vector<std::string> words;
        std::stringstream sstream(buffer);
        std::string word;
        while (getline(sstream, word, ','))
        {
            words.push_back(word);
        }
        if (words.size() != 3)
        {
            continue;
        }
        EnergyTimeData datum;
        datum.InteractionTimeInMili = std::chrono::milliseconds(stoll(words[0]));
        datum.InteractionChannel = stoi(words[1]);
        datum.Energy = stod(words[2]);
        mListedEnergyTimeData.push_back(datum);
        mEnergySpectrums[datum.InteractionChannel].AddEnergy(datum.Energy);
    }

    if (mListedListModeData.size() == 0)
    {
        spdlog::error("Load list mode data fail: no data");

        return;
    }

    if (mListedEnergyTimeData.size() == 0)
    {
        spdlog::error("Load list mode energy time data fail: no data");

        return;
    }

    mLoadedDataStartTime = mListedEnergyTimeData.front().InteractionTimeInMili;
    mLoadedDataLastTime = mListedEnergyTimeData.back().InteractionTimeInMili;

    std::string colorimg = fileDir + "/rgb.png";
    mRgbImage = cv::imread(colorimg, IMREAD_ANYCOLOR);

    std::string depthimg = fileDir + "/depth.png";
    mDepthImage = cv::imread(colorimg, IMREAD_ANYDEPTH);

    // need to load slam and realtime pointcloud;
    mLoadFileDir = fileDir;
    mIsLoaded = true;

    spdlog::info("C++HUREL::Compton::LahgiControl: Load lm data: {0}", fileDir);
    UpdateInteractionImage();
    SetNeedToUpatesAsTrue();

    return;
}
HUREL::Compton::SessionData::~SessionData()
{
}

bool HUREL::Compton::SessionData::Save(std::string fileDir)
{
    std::string fullFilePath = fileDir + "/LmData.csv";
    // change / to \//
    std::ofstream saveFile(fullFilePath, std::ios::out);
    // saveFile.open("fullFilePath", std::ios::out);
    if (!saveFile.is_open())
    {
        std::cout << "File is not opened" << std::endl;
        saveFile.close();
        return false;
    }
    std::vector<ListModeData> data = this->GetListedListModeData();
    for (unsigned int i = 0; i < data.size(); ++i)
    {
        ListModeData &d = data[i];
        saveFile << d.WriteListModeData() << std::endl;
    }
    saveFile.flush();
    saveFile.close();


    fullFilePath = fileDir + "/LmTransData.csv";
    // change / to \//
    saveFile.open(fullFilePath, std::ios::out);
    if (!saveFile.is_open())
    {
        std::cout << "File is not opened" << std::endl;
        saveFile.close();
        return false;
    }
    std::vector<ListModeData> data2 = this->GetListedListModeData();
    for (unsigned int i = 0; i < data2.size(); ++i)
    {
        ListModeData &d = data2[i];
        saveFile << d.WriteListModeTransData() << std::endl;
    }
    // flush
    saveFile.flush();
    saveFile.close();


    fullFilePath = fileDir + "/LmEnergyData.csv";
    // change / to \//
    saveFile.open(fullFilePath, std::ios::out);
    if (!saveFile.is_open())
    {
        std::cout << "File is not opened" << std::endl;
        saveFile.close();
        return false;
    }
    std::vector<EnergyTimeData> edata = this->GetListedEnergyTimeData();
    for (unsigned int i = 0; i < edata.size(); ++i)
    {
        if (saveFile.bad())
        {
            std::cout << "File is bad" << std::endl;
            break;
        }

        EnergyTimeData &d = edata[i];

        std::string line = "";
        line += std::to_string(d.InteractionTimeInMili.count());
        line += ",";
        line += std::to_string(d.InteractionChannel);
        line += ",";
        line += std::to_string(d.Energy);

        saveFile << line << std::endl;
    }
    // flush
    saveFile.flush();
    saveFile.close();

    //save when the stop botton is pressed
    //new
    open3d::geometry::PointCloud pcl = RtabmapSlamControl::instance().GetSlamPointCloud();
    open3d::io::WritePointCloudOption option;
    open3d::io::WritePointCloudToPLY(fileDir + "/slam.ply", pcl, option);

    //origin
    cv::imwrite(fileDir + "/rgb.png", mRgbImage);
    cv::imwrite(fileDir + "/depth.png", mDepthImage);
    return true;
}

std::string HUREL::Compton::SessionData::GetLoadFileDir()
{
    if (mIsLoaded)
    {
        return mLoadFileDir;
    }
    return "Not loaded";
}

std::vector<ListModeData> HUREL::Compton::SessionData::GetListedListModeData()
{
    mResetListModeDataMutex.lock();
    //spdlog::info("GetListedListModeData1: lock");

    size_t size = mListedListModeData.size();
    std::vector<ListModeData> lmData;
    lmData.reserve(size);
    for (int i = 0; i < size; ++i)
    {
        lmData.push_back(mListedListModeData[i]);
    }
    mResetListModeDataMutex.unlock();

    return lmData;
}

std::vector<ListModeData> HUREL::Compton::SessionData::GetListedListModeData(sEnergyCheck echk)
{
    mResetListModeDataMutex.lock();
    //spdlog::info("GetListedListModeData2: lock");

    size_t size = mListedListModeData.size();
    std::vector<ListModeData> lmData;
    lmData.reserve(size);
    for (int i = 0; i < size; ++i)
    {
        ListModeData &d = mListedListModeData[i];
        if (d.EnergyCheck == echk)
        {
            lmData.push_back(d);
        }
    }
    mResetListModeDataMutex.unlock();
    return lmData;
}
cv::Mat HUREL::Compton::SessionData::GetRGBImage()
{ return mRgbImage; }
cv::Mat HUREL::Compton::SessionData::GetDepthImage()
{ return mDepthImage; }
open3d::geometry::PointCloud HUREL::Compton::SessionData::GetPointCloud()
{ return mPointCloud; }
open3d::geometry::PointCloud HUREL::Compton::SessionData::GetSlamPointCloud()
{ return mSlamPointCloud; }
open3d::geometry::PointCloud HUREL::Compton::SessionData::GetOccupancyPointCloud()
{ return mOccupancyPointCloud; }
Eigen::Matrix4d HUREL::Compton::SessionData::GetTransMatrix()
{ return mTransMatrix; }

std::vector<EnergyTimeData> HUREL::Compton::SessionData::GetListedEnergyTimeData()
{
    mResetListModeDataMutex.lock();
    //spdlog::info("GetListedEnergyTimeData: lock");
    std::vector<EnergyTimeData> lmData;

    size_t size = mListedEnergyTimeData.size();

    lmData.reserve(size);
    for (int i = 0; i < size; ++i)
    {
        lmData.push_back(mListedEnergyTimeData[i]);
    }
    mResetListModeDataMutex.unlock();
    return lmData;
}

std::vector<ListModeData> HUREL::Compton::SessionData::GetListedListModeData(std::chrono::milliseconds timeInMililsecondsChrono)
{
    long long timeInMililseconds = timeInMililsecondsChrono.count();
    if (timeInMililseconds <= 0)
    {
        return GetListedListModeData();
    }
    mResetListModeDataMutex.lock();
    //spdlog::info("GetListedListModeData3: lock");

    std::vector<ListModeData> lmData;

    long long currentTime;
    if (mIsLoaded)
    {
        currentTime = mLoadedDataLastTime.count();
    }
    else
    {
        std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        currentTime = t.count();
    }

    size_t getIndexStart = GetIndexOfTime(mListedListModeData, currentTime - timeInMililseconds);

    size_t size = mListedListModeData.size();

    lmData.reserve(size - getIndexStart);
    for (int i = getIndexStart; i < size; ++i)
    {
        lmData.push_back(mListedListModeData[i]);
    }
    mResetListModeDataMutex.unlock();

    return lmData;
}

std::vector<ListModeData> HUREL::Compton::SessionData::GetListedListModeData(std::chrono::milliseconds timeInMililsecondsChrono, sEnergyCheck echk)
{

    long long timeInMililseconds = timeInMililsecondsChrono.count();
    if (timeInMililseconds <= 0)
    {
        return GetListedListModeData(echk);
    }
    mResetListModeDataMutex.lock();
    //spdlog::info("GetListedListModeData4: lock");

    std::vector<ListModeData> lmData;

    long long currentTime;
    if (mIsLoaded)
    {
        currentTime = mLoadedDataLastTime.count();
    }
    else
    {
        std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        currentTime = t.count();
    }

    size_t getIndexStart = GetIndexOfTime(mListedListModeData, currentTime - timeInMililseconds);

    size_t size = mListedListModeData.size();

    lmData.reserve(size - getIndexStart);
    for (int i = getIndexStart; i < size; ++i)
    {
        ListModeData &d = mListedListModeData[i];
        if (d.EnergyCheck == echk)
        {
            lmData.push_back(d);
        }
    }
    mResetListModeDataMutex.unlock();
    return lmData;
}

std::vector<ListModeData> HUREL::Compton::SessionData::GetListedListModeData(size_t count)
{
    mResetListModeDataMutex.lock();
    //spdlog::info("GetListedListModeData5: lock");

    if (mListedListModeData.size() < count)
    {
        count = mListedListModeData.size();
    }
    std::vector<ListModeData> lmData;
    if (count == 0 || mListedListModeData.size() == 0)
    {
        mResetListModeDataMutex.unlock();
        return lmData;
    }
    lmData.reserve(count);
    //spdlog::info("GetListedListModeData size: {}", mListedListModeData.size());
    for (int i = mListedListModeData.size() - 1; i > mListedListModeData.size() - count; --i)
    {
        lmData.push_back(mListedListModeData[i]);
    }
    mResetListModeDataMutex.unlock();

    return lmData;
}

std::vector<ListModeData> HUREL::Compton::SessionData::GetListedListModeData(size_t count, sEnergyCheck echk)
{
    mResetListModeDataMutex.lock();
    //spdlog::info("GetListedListModeData6: lock");
    if (mListedListModeData.size() < count)
    {
        count = mListedListModeData.size();
    }
    std::vector<ListModeData> lmData;
    if (count ==0 || mListedListModeData.size() == 0)
    {
        mResetListModeDataMutex.unlock();

        return lmData;
    }
    lmData.reserve(count);
    for (int i = mListedListModeData.size() - 1; i > mListedListModeData.size() - count; --i)
    {
        ListModeData &d = mListedListModeData[i];
        if (d.EnergyCheck == echk)
        {
            lmData.push_back(d);
        }
    }
    mResetListModeDataMutex.unlock();

    return lmData;
}

std::vector<sInteractionData> HUREL::Compton::SessionData::GetInteractionData(sEnergyCheck echk, std::chrono::milliseconds timeInMiliseconds)
{
    mResetListModeDataMutex.lock();
    //spdlog::info("GetInteractionData1: lock");
    if (mInteractionImages.size() == 0)
    {
        mResetListModeDataMutex.unlock();
        return std::vector<sInteractionData>();
    }
    // check if the echk is in the vector
    int findIndex = -1;
    for (int i = 0; i < mInteractionImages.size(); ++i)
    {
        if (mInteractionImages[i].first == echk)
        {
            findIndex = i;
            break;
        }
    }
    if (findIndex == -1)
    {
        mResetListModeDataMutex.unlock();
        return std::vector<sInteractionData>();
    }

    auto it = mInteractionImages.begin() + findIndex;

    if (it->second.size() == 0)
    {
        mResetListModeDataMutex.unlock();
        return std::vector<sInteractionData>();
    }

    long long timeInMilisecondsLong = timeInMiliseconds.count();

    long long lastTime = this->GetEndTime().count();

    std::vector<sInteractionData> tempInteractionData = it->second;
    std::vector<sInteractionData> output = std::vector<sInteractionData>();

    for (int i = tempInteractionData.size() - 1; i >= 0; --i)
    {
        if (lastTime - tempInteractionData[i].StartInteractionTimeInMili.count() <= timeInMilisecondsLong)
        {
            // insert at the beginning
            output.insert(output.begin(), tempInteractionData[i]);
        }
        else
        {
            break;
        }
    }

    mResetListModeDataMutex.unlock();
    return output;
}

std::vector<sInteractionData> HUREL::Compton::SessionData::GetInteractionData(sEnergyCheck echk, size_t count)
{
    mResetListModeDataMutex.lock();
    //spdlog::info("GetInteractionData2: lock");
    if (mInteractionImages.size() == 0)
    {
        mResetListModeDataMutex.unlock();
        return std::vector<sInteractionData>();
    }
    // check if the echk is in the vector
    int findIndex = -1;
    for (int i = 0; i < mInteractionImages.size(); ++i)
    {
        if (mInteractionImages[i].first == echk)
        {
            findIndex = i;
            break;
        }
    }
    if (findIndex == -1)
    {
        mResetListModeDataMutex.unlock();
        return std::vector<sInteractionData>();
    }

    auto it = mInteractionImages.begin() + findIndex;

    if (it->second.size() == 0)
    {
        mResetListModeDataMutex.unlock();
        return std::vector<sInteractionData>();
    }

    std::vector<sInteractionData> tempInteractionData = it->second;
    std::vector<sInteractionData> output = std::vector<sInteractionData>();

    size_t currentSize = 0;
    for (int i = tempInteractionData.size() - 1; i >= 0; --i)
    {
        if (currentSize < count)
        {
            // insert at the beginning
            output.insert(output.begin(), tempInteractionData[i]);
            currentSize += tempInteractionData[i].interactionCount;
        }
        else
        {
            break;
        }
    }
    mResetListModeDataMutex.unlock();
    if (currentSize < count)
    {
        return std::vector<sInteractionData>();
    }
    return output;
}

std::vector<sInteractionData> HUREL::Compton::SessionData::GetInteractionData(IsotopeData iso, std::chrono::milliseconds timeInMiliseconds)
{
    for (auto &i : iso.GetEnergyCheck())
    {
        auto temp = this->GetInteractionData(i, timeInMiliseconds);
        if (temp.size() > 0)
        {
            return temp;
        }
    }
    return std::vector<sInteractionData>();
}

std::vector<sInteractionData> HUREL::Compton::SessionData::GetInteractionData(IsotopeData iso, size_t count)
{
    mResetListModeDataMutex.lock();
    //spdlog::info("GetInteractionData3: lock");
    if (mInteractionImages.size() == 0)
    {
        mResetListModeDataMutex.unlock();
        return std::vector<sInteractionData>();
    }
    // check if the echk is in the vector
    std::vector<int> findIndexes = std::vector<int>();
    for (int i = 0; i < mInteractionImages.size(); ++i)
    {
         std::vector<Compton::sEnergyCheck> energyChecks = iso.GetEnergyCheck();
        for (int j = 0; j <energyChecks.size(); ++j)
        {
            if (mInteractionImages[i].first == energyChecks[j])
            {
                if (mInteractionImages[i].second.size() > 0)
                {
                    findIndexes.push_back(i);
                }
            }
        }
    }
    if (findIndexes.size() == 0)
    {
        mResetListModeDataMutex.unlock();
        return std::vector<sInteractionData>();
    }
    size_t currentSize = 0;
    
    std::vector<int> searchIndex = std::vector<int>();
    for (auto index : findIndexes)
    {
        auto it = mInteractionImages.begin() + index;
        if (it->second.size() > 0)
        {
            searchIndex.push_back(it->second.size() - 1);

        }
        else
        {
            searchIndex.push_back(0);
        }
    }
    std::vector<sInteractionData> output = std::vector<sInteractionData>();
    while (currentSize < count)
    {
        int i = 0;
        bool isDone = true;
        for (auto index : findIndexes)
        {
            auto it = mInteractionImages.begin() + index;

            if (it->second.size() == 0 || searchIndex[i] < 0)
            {
                ++i;
                continue;
            }

            std::vector<sInteractionData> tempInteractionData = it->second;

            output.insert(output.begin(), tempInteractionData[searchIndex[i]]);
            currentSize += tempInteractionData[searchIndex[i]].interactionCount;
            --searchIndex[i];
            if (searchIndex[i] >= 0)
            {
                isDone = false;
            }
            ++i;

        }
        if (isDone)
        {
            break;
        }
    }
    mResetListModeDataMutex.unlock();
    return output;
}

std::vector<std::pair<sEnergyCheck, std::vector<sInteractionData>>> HUREL::Compton::SessionData::GetInteractionImages()
{
    std::vector<std::pair<sEnergyCheck, std::vector<sInteractionData>>> tempImages;
    if (mInteractionImages.size() == 0)
    {
        return tempImages;
    }

    for (auto &i : mInteractionImages)
    {
        tempImages.push_back(i);
    }

    return tempImages;
}

std::vector<sEnergyCheck> HUREL::Compton::SessionData::GetEnergyCheckList()
{
    // get the list of energy check from mInteractionImages
    mResetListModeDataMutex.lock();
    //spdlog::info("GetEnergyCheckList: lock");
    if (mInteractionImages.size() > 0)
    {
        std::vector<sEnergyCheck> echkList;
        echkList.reserve(mInteractionImages.size());
        for (auto &i : mInteractionImages)
        {
            echkList.push_back(i.first);
        }
        mResetListModeDataMutex.unlock();
        return echkList;
    }
    mResetListModeDataMutex.unlock();
    return std::vector<sEnergyCheck>();
}

std::vector<EnergyTimeData> HUREL::Compton::SessionData::GetListedEnergyTimeData(long long timeInMiliSecond)
{
    if (timeInMiliSecond <= 0)
    {
        return GetListedEnergyTimeData();
    }
    mResetListModeDataMutex.lock();
    //spdlog::info("GetListedEnergyTimeData: lock");

    std::vector<EnergyTimeData> lmData;
    long long currentTime;
    if (mIsLoaded)
    {
        currentTime = mLoadedDataLastTime.count();
    }
    else
    {
        std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
        currentTime = t.count();
    }

    size_t getIndexStart = GetIndexOfTime(mListedEnergyTimeData, currentTime - timeInMiliSecond);

    size_t size = mListedEnergyTimeData.size();

    lmData.reserve(size - getIndexStart);

    for (int i = getIndexStart; i < size; ++i)
    {
        lmData.push_back(mListedEnergyTimeData[i]);
    }
    mResetListModeDataMutex.unlock();

    return lmData;
}

std::vector<double> HUREL::Compton::SessionData::GetCountRateEvery500ms()
{
    double timeStep = 1000; // ms
    if (mListedEnergyTimeData.empty())
    {
        return std::vector<double>();
    }
    size_t listModeDataSize = mListedEnergyTimeData.size();
    mResetListModeDataMutex.lock();
    //spdlog::info("GetCountRateEvery500ms: lock");
    if (mCountRate.size() == 0)
    {
        std::chrono::milliseconds startTime = mListedEnergyTimeData[0].InteractionTimeInMili;
        std::chrono::milliseconds endTime = mListedEnergyTimeData[listModeDataSize - 1].InteractionTimeInMili;
        std::chrono::milliseconds timeDiff2 = endTime - startTime;
        if (timeDiff2.count() <= 0)
        {
            mResetListModeDataMutex.unlock();
            return std::vector<double>();
        }

        std::vector<double> countRate = std::vector<double>();
        countRate.reserve(timeDiff2.count() / timeStep);

        size_t index = 0;
        for (int i = 0; i < timeDiff2.count(); i += timeStep)
        {
            size_t count = 0;
            while (index < listModeDataSize && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i)
            {
                ++index;
            }
            while (index < listModeDataSize && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i + timeStep)
            {
                ++count;
                ++index;
            }
            if (count == 0)
            {
                count = countRate[countRate.size() - 1] * (timeStep / 1000) * 1000;
            }
            countRate.push_back(static_cast<double>(count) / (timeStep / 1000) / 1000);
        }
        mCountRate = countRate;
        mResetListModeDataMutex.unlock();
        return mCountRate;
    }
    else
    {
        std::vector<double> countRate = mCountRate;
        long long elapsedTimeInMiliSecond = countRate.size() * timeStep;
        std::chrono::milliseconds startTime = mListedEnergyTimeData[0].InteractionTimeInMili + std::chrono::milliseconds(elapsedTimeInMiliSecond);
        std::chrono::milliseconds endTime = mListedEnergyTimeData[listModeDataSize - 1].InteractionTimeInMili;
        size_t index = GetIndexOfTime(mListedEnergyTimeData, startTime.count());

        std::chrono::milliseconds timeDiff2 = endTime - startTime;
        for (int i = 0; i < timeDiff2.count(); i += timeStep)
        {
            size_t count = 0;
            while (index < listModeDataSize && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i)
            {
                ++index;
            }
            while (index < listModeDataSize && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i + timeStep)
            {
                ++count;
                ++index;
            }
            if (count == 0)
            {
                count = countRate[countRate.size() - 1] * (timeStep / 1000) * 1000;
            }
            countRate.push_back(static_cast<double>(count) / (timeStep / 1000) / 1000);
        }
        mCountRate = countRate;
        mResetListModeDataMutex.unlock();
        return mCountRate;
    }
}

std::vector<double> HUREL::Compton::SessionData::GetCountRateEvery500ms(sEnergyCheck echk)
{
    if (mListedListModeData.empty())
    {
        return std::vector<double>();
    }
    mResetListModeDataMutex.lock();
    //spdlog::info("GetCountRateEvery500ms: lock");

    size_t listModeDataSize = mListedEnergyTimeData.size();
    // find count rate data matching echk
    std::vector<double> countRate;
    if (mCountRateByEnergyCheck.find(echk) == mCountRateByEnergyCheck.end())
    {
        countRate = std::vector<double>();
        mCountRateByEnergyCheck.emplace(echk, countRate);
    }
    else
    {
        countRate = mCountRateByEnergyCheck[echk];
    }

    if (countRate.size() == 0)
    {
        std::chrono::milliseconds startTime = mListedEnergyTimeData[0].InteractionTimeInMili;
        std::chrono::milliseconds endTime = mListedEnergyTimeData[listModeDataSize - 1].InteractionTimeInMili;
        std::chrono::milliseconds timeDiff2 = endTime - startTime;
        if (timeDiff2.count() <= 0)
        {
            mResetListModeDataMutex.unlock();
            return std::vector<double>();
        }

        std::vector<double> countRate;
        countRate.reserve(timeDiff2.count() / 500);

        size_t index = 0;
        for (int i = 0; i < timeDiff2.count(); i += 500)
        {
            size_t count = 0;
            while (index < listModeDataSize && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i)
            {
                ++index;
            }
            while (index < listModeDataSize && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i + 500)
            {
                ++count;
                ++index;
            }
            if (count == 0)
            {
                count = countRate[countRate.size() - 1] * 0.5 * 1000;
            }
            countRate.push_back(count / 0.5 / 1000);
        }
        mCountRateByEnergyCheck[echk] = countRate;
        mResetListModeDataMutex.unlock();
        return countRate;
    }
    else
    {
        long long elapsedTimeInMiliSecond = countRate.size() * 500;
        std::chrono::milliseconds startTime = mListedEnergyTimeData[0].InteractionTimeInMili + std::chrono::milliseconds(startTime.count() + elapsedTimeInMiliSecond);
        std::chrono::milliseconds endTime = mListedEnergyTimeData[listModeDataSize - 1].InteractionTimeInMili;
        size_t index = GetIndexOfTime(mListedEnergyTimeData, startTime.count());

        std::chrono::milliseconds timeDiff2 = endTime - startTime;
        for (int i = 0; i < timeDiff2.count(); i += 500)
        {
            size_t count = 0;
            while (index < listModeDataSize && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i)
            {
                ++index;
            }
            while (index < listModeDataSize && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i + 500)
            {
                ++count;
                ++index;
            }
            if (count == 0)
            {
                count = countRate[countRate.size() - 1] * 0.5 * 1000;
            }
            countRate.push_back(count / 0.5 / 1000);
        }
        mCountRateByEnergyCheck[echk] = countRate;
        mResetListModeDataMutex.unlock();
        return countRate;
    }
}

std::chrono::milliseconds HUREL::Compton::SessionData::GetStartTime()
{
    mResetListModeDataMutex.lock();
    //spdlog::info("GetStartTime: lock");
    if (mListedEnergyTimeData.size() > 0)
    {
        std::chrono::milliseconds startTime = mListedEnergyTimeData[0].InteractionTimeInMili;
        mResetListModeDataMutex.unlock();
        return startTime;
    }
    mResetListModeDataMutex.unlock();
    return std::chrono::milliseconds();
}

std::chrono::milliseconds HUREL::Compton::SessionData::GetEndTime()
{
    mResetListModeDataMutex.lock();
    //spdlog::info("GetEndTime: lock");
    if (mListedEnergyTimeData.size() > 0)
    {
        std::chrono::milliseconds endtime = mListedEnergyTimeData[mListedEnergyTimeData.size() - 1].InteractionTimeInMili;
        mResetListModeDataMutex.unlock();
        return endtime;
    }
    mResetListModeDataMutex.unlock();
    return std::chrono::milliseconds();
}

void HUREL::Compton::SessionData::AddEnergyTime(EnergyTimeData &datum)
{
    mListedEnergyTimeData.push_back(datum);
    mEnergySpectrums[datum.InteractionChannel].AddEnergy(datum.Energy);
    SetNeedToUpatesAsTrue();
}

void HUREL::Compton::SessionData::AddListModeData(ListModeData &datum)
{
    mListedListModeData.push_back(datum);
    SetNeedToUpatesAsTrue();
}

size_t HUREL::Compton::SessionData::GetIndexOfTime(tbb::concurrent_vector<ListModeData> &listedModeData, long long target)
{
    if (listedModeData.size() == 0)
    {
        return 0;
    }
    // use binary search
    size_t listSize = listedModeData.size();
    size_t left = 0;
    size_t right = listSize - 1;

    while (left <= right)
    {
        size_t mid = (left + right) / 2;
        long long midData = listedModeData.at(mid).InteractionTimeInMili.count();
        if (target < midData)
        {
            right = mid - 1;
            if (mid == 0)
            {
                return 0;
            }
        }
        else
        {
            left = mid + 1;
        }
    }
    if (listSize <= left)
    {
        left = listSize - 1;
    }
    while (left != 0 && listedModeData.at(left).InteractionTimeInMili.count() >= target)
    {
        --left;
    }
    if (left != 0)
    {
        ++left;
    }

    return left;
}

size_t HUREL::Compton::SessionData::GetIndexOfTime(tbb::concurrent_vector<EnergyTimeData> &listedModeData, long long target)
{
    if (listedModeData.size() == 0)
    {
        return 0;
    }
    // use binary search
    size_t listSize = listedModeData.size();
    size_t left = 0;
    size_t right = listSize - 1;

    while (left <= right)
    {
        size_t mid = (left + right) / 2;
        long long midData = listedModeData.at(mid).InteractionTimeInMili.count();
        if (target < midData)
        {
             if (mid == 0)
            {
                return 0;
            }
            right = mid - 1;
        }
        else
        {
            left = mid + 1;
        }
    }
    if (listSize <= left)
    {
        left = listSize - 1;
    }
    while (left != 0 && listedModeData.at(left).InteractionTimeInMili.count() >= target)
    {
        --left;
    }
    if (left != 0)
    {
        ++left;
    }

    return left;
}

bool HUREL::Compton::SessionData::IsSame(ListModeData &lmData, sInteractionData &interactionData, double timeDifference, Eigen::Vector3d positionDifference)
{
    // set spdlog level debug
    spdlog::set_level(spdlog::level::info);
    if (lmData.InteractionTimeInMili.count() - interactionData.EndInteractionTimeInMili.count() > timeDifference * 1000)
    {
        spdlog::info("Time difference is {} ms", lmData.InteractionTimeInMili.count() - interactionData.EndInteractionTimeInMili.count());
        return false;
    }
    spdlog::set_level(spdlog::level::info);

    // get Position from Eigen::Matrix4d
    Eigen::Vector3d position1;
    position1 << lmData.DetectorTransformation(0, 3), lmData.DetectorTransformation(1, 3), lmData.DetectorTransformation(2, 3);
    Eigen::Vector3d position2;
    position2 << interactionData.DetectorTransformation(0, 3), interactionData.DetectorTransformation(1, 3), interactionData.DetectorTransformation(2, 3);

    if ((position1 - position2).norm() > positionDifference.norm())
    {
        return false;
    }

    return true;
}

void HUREL::Compton::SessionData::LockMutexAddingData()
{
    mListModeDataMutex.lock();
}

void HUREL::Compton::SessionData::UnlockMutexAddingData()
{
    mListModeDataMutex.unlock();
}

void HUREL::Compton::SessionData::UpdateInteractionImage()
{
    size_t startIndex = UpdateInteractionImageListedListModeDataIndex;

    for (int i = startIndex; i < mListedListModeData.size(); ++i)
    {
        ListModeData &lmData = mListedListModeData[i];

        // check if the echk is in the vector
        int findIndex = -1;
        for (int i = 0; i < mInteractionImages.size(); ++i)
        {
            if (mInteractionImages[i].first == lmData.EnergyCheck)
            {
                findIndex = i;
                break;
            }
        }

        if (findIndex == -1)
        {
            mInteractionImages.push_back(std::make_pair(lmData.EnergyCheck, std::vector<sInteractionData>()));
            findIndex = mInteractionImages.size() - 1;
        }

        auto item = mInteractionImages.begin() + findIndex;
        std::vector<sInteractionData> &interactionData = item->second;

        if (interactionData.size() == 0)
        {
            interactionData.push_back(sInteractionData());
            sInteractionData &data = interactionData[0];
            data.StartInteractionTimeInMili = lmData.InteractionTimeInMili;
            data.EndInteractionTimeInMili = lmData.InteractionTimeInMili;
            data.DetectorTransformation = lmData.DetectorTransformation;
            ++data.interactionCount;
            if (lmData.Type == eInterationType::COMPTON)
            {
                data.ComptonListModeData->push_back(lmData);
            }
            else if (lmData.Type == eInterationType::CODED)
            {
                int positionX = findPositionIndex(lmData.Scatter.RelativeInteractionPoint(0), -0.15, 0.15, INTERACTION_GRID_SIZE);
                int positionY = findPositionIndex(lmData.Scatter.RelativeInteractionPoint(1), -0.15, 0.15, INTERACTION_GRID_SIZE);

                if (positionX != -1 && positionY != -1)
                {
                     data.RelativeInteractionPoint(positionY, positionX) += 1;
                }
            }
        }
        else
        {
            sInteractionData &data = interactionData[interactionData.size() - 1];
            if (IsSame(lmData, data) && data.interactionCount < 2000)
            {
                data.EndInteractionTimeInMili = lmData.InteractionTimeInMili;
                ++data.interactionCount;
                if (lmData.Type == eInterationType::COMPTON)
                {
                    data.ComptonListModeData->push_back(lmData);
                }
                else if (lmData.Type == eInterationType::CODED)
                {
                    int positionX = findPositionIndex(lmData.Scatter.RelativeInteractionPoint(0), -0.15, 0.15, INTERACTION_GRID_SIZE);
                    int positionY = findPositionIndex(lmData.Scatter.RelativeInteractionPoint(1), -0.15, 0.15, INTERACTION_GRID_SIZE);

                    if (positionX != -1 && positionY != -1)
                    {
                         data.RelativeInteractionPoint(positionY, positionX) += 1;
                    }
                }
            }
            else
            {
                interactionData.push_back(sInteractionData());
                sInteractionData &data = interactionData[interactionData.size() - 1];
                data.StartInteractionTimeInMili = lmData.InteractionTimeInMili;
                data.EndInteractionTimeInMili = lmData.InteractionTimeInMili;
                data.DetectorTransformation = lmData.DetectorTransformation;
                ++data.interactionCount;

                if (lmData.Type == eInterationType::COMPTON)
                {
                    data.ComptonListModeData->push_back(lmData);
                }
                else if (lmData.Type != eInterationType::CODED)
                {
                    int positionX = findPositionIndex(lmData.Scatter.RelativeInteractionPoint(0), -0.15, 0.15, INTERACTION_GRID_SIZE);
                    int positionY = findPositionIndex(lmData.Scatter.RelativeInteractionPoint(1), -0.15, 0.15, INTERACTION_GRID_SIZE);

                    if (positionX != -1 && positionY != -1)
                    {
                        //data.RelativeInteractionPoint(positionX, positionY) += 1;
                        data.RelativeInteractionPoint(positionY, positionX) += 1;
                    }
                }
            }
        }

        item->second = interactionData;
    }
    UpdateInteractionImageListedListModeDataIndex = mListedListModeData.size();
}

void HUREL::Compton::SessionData::Reset()
{
    mResetListModeDataMutex.lock();
    //spdlog::info("Reset lock()");
    mListModeDataMutex.lock();
    mListedListModeData.clear();
    mListedListModeData.shrink_to_fit();
    mListedListModeData.reserve(50000);

    mListedEnergyTimeData.clear();
    mListedEnergyTimeData.shrink_to_fit();
    mListedEnergyTimeData.reserve(50000);

    // iterate through the map using iterator
    for (auto it = mInteractionImages.begin(); it != mInteractionImages.end(); ++it)
    {
        it->second.clear();
    }

    mCountRate.clear();
    for (auto it = mCountRateByEnergyCheck.begin(); it != mCountRateByEnergyCheck.end(); ++it)
    {
        it->second.clear();
    }

    for (int i = 0; i < 16; ++i)
    {
        mEnergySpectrums[i].Reset();
    }

    mCountRateByEnergyCheck.clear();

    mInteractionImages.clear();

    UpdateInteractionImageListedListModeDataIndex = 0;

    mPointCloud = open3d::geometry::PointCloud();
    mSlamPointCloud = open3d::geometry::PointCloud();
    mRgbImage = cv::Mat();
    mDepthImage = cv::Mat();
    mListModeDataMutex.unlock();
    mResetListModeDataMutex.unlock();
    mOccupancyPointCloud = open3d::geometry::PointCloud();
    SetNeedToUpatesAsTrue();
}

void HUREL::Compton::SessionData::ResetSpectrum(size_t fpgaChannel)
{
    mResetListModeDataMutex.lock();
    //spdlog::info("ResetSpectrum lock()");

    mEnergySpectrums[fpgaChannel].Reset();

    mResetListModeDataMutex.unlock();
    SetNeedToUpatesAsTrue();
}

EnergySpectrum HUREL::Compton::SessionData::GetEnergySpectrum(size_t fpgaChannel)
{
    mResetListModeDataMutex.lock();
    //spdlog::info("GetEnergySpectrum lock()");
    if (fpgaChannel >= 16)
    {
        mResetListModeDataMutex.unlock();
        ;
        return EnergySpectrum();
    }
    EnergySpectrum espect = mEnergySpectrums[fpgaChannel];
    mResetListModeDataMutex.unlock();
    return espect;
}

EnergySpectrum HUREL::Compton::SessionData::GetEnergySpectrum(int *fpgaChannels, int size, long long timeInMiliseconds)
{
    EnergySpectrum espect;
    if (timeInMiliseconds <= 0 || fpgaChannels == nullptr)
    {
        for (int i = 0; i < 16; ++i)
        {
            espect = espect + mEnergySpectrums[i];
        }
    }
    else
    {
        std::vector<EnergyTimeData> etData = GetListedEnergyTimeData(timeInMiliseconds);
        for (int i = 0; i < etData.size(); ++i)
        {
            EnergyTimeData &etDatum = etData.at(i);

            for (int j = 0; j < size; ++j)
            {
                if (fpgaChannels[j] == etDatum.InteractionChannel)
                {
                    espect.AddEnergy(etDatum.Energy);
                    break;
                }
            }
        }
    }

    return espect;
}

EnergySpectrum HUREL::Compton::SessionData::GetScatterEnergySpectrum(long long timeInMiliseconds)
{
    EnergySpectrum espect;
    if (timeInMiliseconds <= 0)
    {
        for (int i = 0; i < 8; ++i)
        {
            espect = espect + mEnergySpectrums[i];
        }
    }
    else
    {
        int fpgaChannels[8] = {0, 1, 2, 3, 4, 5, 6, 7};
        espect = GetEnergySpectrum(fpgaChannels, 8, timeInMiliseconds);
    }
    return espect;
}

EnergySpectrum HUREL::Compton::SessionData::GetAbsroberEnergySpectrum(long long timeInMiliseconds)
{
    EnergySpectrum espect;
    if (timeInMiliseconds <= 0)
    {
        for (int i = 8; i < 16; ++i)
        {
            espect = espect + mEnergySpectrums[i];
        }
    }
    else
    {
        int fpgaChannels[8] = {8, 9, 10, 11, 12, 13, 14, 15};
        espect = GetEnergySpectrum(fpgaChannels, 8, timeInMiliseconds);
    }
    return espect;
}

size_t HUREL::Compton::SessionData::GetSizeListedListModeData()
{
    return mListedListModeData.size();
}

size_t HUREL::Compton::SessionData::GetSizeListedEnergyTimeData()
{
    return mListedEnergyTimeData.size();
}
