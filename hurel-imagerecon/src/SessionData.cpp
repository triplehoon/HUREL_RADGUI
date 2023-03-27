#include "SessionData.h"

using namespace HUREL;
using namespace Compton;

inline int findPositionIndex(double value, double min, double max, int spaceCount)
{

    if (value - min <= 0)
    {
        return -1;
    }
    if (value - max >= 0)
    {
        return -1;
    }
    double pixelSize = (max - min) / spaceCount;
    return static_cast<int>(floor((value - min) / pixelSize));
}

void HUREL::Compton::SessionData::SetNeedToUpatesAsTrue()
{
    mIsNeedToUpdateCountRate = true;
}

HUREL::Compton::SessionData::SessionData() : mIsLoaded(false),
                                             mLoadedDataStartTime(std::chrono::milliseconds::zero()),
                                             mLoadFileDir(""),
                                             mListedEnergyTimeData(0),
                                             mListedListModeData(0),
                                             mIsNeedToUpdateCountRate(true)
{
    mListedEnergyTimeData.reserve(1000000);
    mListedListModeData.reserve(1000000);
}

HUREL::Compton::SessionData::SessionData(std::string fileDir) : mIsLoaded(true)
{
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
            if (temp.Type == eInterationType::CODED)
            {
                if (temp.Scatter.InteractionEnergy < 662 || temp.Scatter.InteractionEnergy > 720)
                {
                    continue;
                }
            }
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
/*
HUREL::Compton::SessionData::SessionData(const SessionData &other)
{
}

SessionData &HUREL::Compton::SessionData::operator=(const SessionData &other)
{
    if (this == &other)
    {
        return *this;
    }

    return *this;
}
*/
bool HUREL::Compton::SessionData::Save(std::string fileDir)
{
    std::ofstream saveFile;
    saveFile.open(fileDir + "/LmData.csv");
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
    saveFile.close();

    saveFile.open(fileDir + "/LmEnergyData.csv");
    if (!saveFile.is_open())
    {
        std::cout << "File is not opened" << std::endl;
        saveFile.close();
        return false;
    }
    std::vector<EnergyTimeData> edata = this->GetListedEnergyTimeData();
    for (unsigned int i = 0; i < edata.size(); ++i)
    {
        EnergyTimeData &d = edata[i];

        std::string line = "";
        line += std::to_string(d.InteractionTimeInMili.count());
        line += ",";
        line += std::to_string(d.InteractionChannel);
        line += ",";
        line += std::to_string(d.Energy);

        saveFile << line << std::endl;
    }
    saveFile.close();

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

std::vector<EnergyTimeData> HUREL::Compton::SessionData::GetListedEnergyTimeData()
{
    mResetListModeDataMutex.lock();
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

    if (mListedListModeData.size() < count)
    {
        count = mListedListModeData.size();
    }
    std::vector<ListModeData> lmData;
    if (count == 0)
    {
        mResetListModeDataMutex.unlock();
        return lmData;
    }
    lmData.reserve(count);
    for (int i = mListedListModeData.size() - 1; i >= mListedListModeData.size() - count; --i)
    {
        lmData.push_back(mListedListModeData[i]);
    }
        mResetListModeDataMutex.unlock();

    return lmData;
}

std::vector<ListModeData> HUREL::Compton::SessionData::GetListedListModeData(size_t count, sEnergyCheck echk)
{
    mResetListModeDataMutex.lock();
    if (mListedListModeData.size() < count)
    {
        count = mListedListModeData.size();
    }
    std::vector<ListModeData> lmData;
    if (count == 0)
    {
        mResetListModeDataMutex.unlock();

        return lmData;
    }
    lmData.reserve(count);
    for (int i = mListedListModeData.size() - 1; i >= mListedListModeData.size() - count; --i)
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

std::vector<std::vector<sInteractionData>> HUREL::Compton::SessionData::GetInteractionImages()
{
    mResetListModeDataMutex.lock();
    std::vector<std::vector<sInteractionData>> tempImages;
    if (mInteractionImages.size() == 0)
    {
        mResetListModeDataMutex.unlock();

        return tempImages;
    }
    tempImages.reserve(mInteractionImages.size());
    for (int i = 0; i < mInteractionImages.size(); ++i)
    {
        tempImages.push_back(mInteractionImages[i]);
    }

    mResetListModeDataMutex.unlock();

    return tempImages;
}

std::vector<EnergyTimeData> HUREL::Compton::SessionData::GetListedEnergyTimeData(long long timeInMiliSecond)
{
    if (timeInMiliSecond <= 0)
    {
        return GetListedEnergyTimeData();
    }
    mResetListModeDataMutex.lock();

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

std::vector<double> HUREL::Compton::SessionData::GetCountRate()
{
    if (!mIsNeedToUpdateCountRate)
    {
        return mCountRate;
    }
    if (mListedListModeData.empty())
    {
        return std::vector<double>();
    }

    if (mCountRate.size() == 0)
    {
        std::chrono::milliseconds startTime = mListedEnergyTimeData[0].InteractionTimeInMili;
        std::chrono::milliseconds endTime = mListedEnergyTimeData[mListedEnergyTimeData.size() - 1].InteractionTimeInMili;
        std::chrono::milliseconds timeDiff2 = endTime - startTime;
        if (timeDiff2.count() <= 0)
        {
            return std::vector<double>();
        }

        std::vector<double> countRate;
        countRate.reserve(timeDiff2.count() / 500);

        size_t index = 0;
        for (int i = 0; i < timeDiff2.count(); i += 500)
        {
            size_t count = 0;
            while (index < mListedEnergyTimeData.size() && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i)
            {
                ++index;
            }
            while (index < mListedEnergyTimeData.size() && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i + 500)
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
        mCountRate = countRate;
        mIsNeedToUpdateCountRate = false;
        return mCountRate;
    }
    else
    {
        std::vector<double> countRate = mCountRate;
        long long elapsedTimeInMiliSecond = countRate.size() * 500;
        std::chrono::milliseconds startTime = mListedEnergyTimeData[0].InteractionTimeInMili + std::chrono::milliseconds(startTime.count() + elapsedTimeInMiliSecond);
        std::chrono::milliseconds endTime = mListedEnergyTimeData[mListedEnergyTimeData.size() - 1].InteractionTimeInMili;
        size_t index = GetIndexOfTime(mListedEnergyTimeData, startTime.count());

        std::chrono::milliseconds timeDiff2 = endTime - startTime;
        for (int i = 0; i < timeDiff2.count(); i += 500)
        {
            size_t count = 0;
            while (index < mListedEnergyTimeData.size() && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i)
            {
                ++index;
            }
            while (index < mListedEnergyTimeData.size() && mListedEnergyTimeData[index].InteractionTimeInMili.count() < startTime.count() + i + 500)
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
        mCountRate = countRate;
        mIsNeedToUpdateCountRate = false;
        return mCountRate;
    }
}

std::chrono::milliseconds HUREL::Compton::SessionData::GetStartTime()
{
    if (mListedEnergyTimeData.size() > 0)
    {
        return mListedEnergyTimeData[0].InteractionTimeInMili;
    }
    return std::chrono::milliseconds();
}

std::chrono::milliseconds HUREL::Compton::SessionData::GetEndTime()
{
    if (mListedEnergyTimeData.size() > 0)
    {
        return mListedEnergyTimeData[mListedEnergyTimeData.size() - 1].InteractionTimeInMili;
    }
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
    size_t left = 0;
    size_t right = listedModeData.size() - 1;

    while (left <= right)
    {
        size_t mid = (left + right) / 2;
        long long midData = listedModeData.at(mid).InteractionTimeInMili.count();
        if (target < midData)
        {
            right = mid - 1;
        }
        else
        {
            left = mid + 1;
        }
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
    size_t left = 0;
    size_t right = listedModeData.size() - 1;

    while (left <= right)
    {
        size_t mid = (left + right) / 2;
        long long midData = listedModeData.at(mid).InteractionTimeInMili.count();
        if (target < midData)
        {
            right = mid - 1;
        }
        else
        {
            left = mid + 1;
        }
    }

    return left;
}

void HUREL::Compton::SessionData::LockMutexAddingData()
{
    mResetEnergySpectrumMutex.lock();
    mResetListModeDataMutex.lock();
}

void HUREL::Compton::SessionData::UnlockMutexAddingData()
{
    mResetListModeDataMutex.unlock();
    mResetEnergySpectrumMutex.unlock();
}

void HUREL::Compton::SessionData::UpdateInteractionImage()
{
    mResetListModeDataMutex.lock();
    size_t startIndex = UpdateInteractionImageListedListModeDataIndex;

    for (int i = startIndex; i < mListedListModeData.size(); ++i)
    {
        ListModeData &lmData = mListedListModeData[i];

        if (lmData.Type != eInterationType::CODED)
        {
            continue;
        }

        int interactionEnergyIndex = -1;
        for (int i = 0; i < mInteractionImages.size(); ++i)
        {
            if (mInteractionImages[i][0].EnergyCheck == mListedListModeData[i].EnergyCheck)
            {
                interactionEnergyIndex = i;
                break;
            }
        }

        int positionX = findPositionIndex(lmData.Scatter.RelativeInteractionPoint(0), -0.15, 0.15, 60);
        int positionY = findPositionIndex(lmData.Scatter.RelativeInteractionPoint(1), -0.15, 0.15, 60);

        if (positionX != -1 && positionY != -1)
        {
            if (interactionEnergyIndex == -1)
            {
                mInteractionImages.push_back(std::vector<sInteractionData>());
                interactionEnergyIndex = mInteractionImages.size() - 1;
                sInteractionData data;
                data.StartInteractionTimeInMili = lmData.InteractionTimeInMili;
                data.EndInteractionTimeInMili = lmData.InteractionTimeInMili;
                data.DetectorTransformation = lmData.DetectorTransformation;
                data.EnergyCheck = lmData.EnergyCheck;                
                data.RelativeInteractionPoint(positionX, positionY) += 1;
                mInteractionImages[interactionEnergyIndex].push_back(data);
                continue;
            }
            size_t lastIndex = mInteractionImages[interactionEnergyIndex].size() - 1;
            if (mInteractionImages[interactionEnergyIndex][lastIndex].DetectorTransformation.isApprox(lmData.DetectorTransformation))
            {
                mInteractionImages[interactionEnergyIndex][lastIndex].RelativeInteractionPoint(positionX, positionY) += 1;
                mInteractionImages[interactionEnergyIndex][lastIndex].EndInteractionTimeInMili = lmData.InteractionTimeInMili;
            }
            else
            {
                sInteractionData data;
                data.StartInteractionTimeInMili = lmData.InteractionTimeInMili;
                data.EndInteractionTimeInMili = lmData.InteractionTimeInMili;
                data.DetectorTransformation = lmData.DetectorTransformation;
                data.EnergyCheck = lmData.EnergyCheck;
                data.RelativeInteractionPoint(positionX, positionY) = 1;
                mInteractionImages[interactionEnergyIndex].push_back(data);
            }
        }
    }
    UpdateInteractionImageListedListModeDataIndex = mListedListModeData.size();
 
    mResetListModeDataMutex.unlock();
}

void HUREL::Compton::SessionData::Reset()
{
    mResetListModeDataMutex.lock();
    mResetEnergySpectrumMutex.lock();

    mListedListModeData.clear();
    mListedListModeData.shrink_to_fit();
    mListedListModeData.reserve(50000);

    mListedEnergyTimeData.clear();
    mListedEnergyTimeData.shrink_to_fit();
    mListedEnergyTimeData.reserve(50000);

    for (int i = 0; i < mInteractionImages.size(); ++i)
    {
        mInteractionImages[i].clear();
    }
    mInteractionImages.clear();

    UpdateInteractionImageListedListModeDataIndex = 0;

    mResetListModeDataMutex.unlock();
    mResetEnergySpectrumMutex.unlock();
    SetNeedToUpatesAsTrue();
}

void HUREL::Compton::SessionData::ResetSpectrum()
{
    mResetEnergySpectrumMutex.lock();
    for (int i = 0; i < 16; ++i)
    {
        mEnergySpectrums[i].Reset();
    }

    mResetEnergySpectrumMutex.unlock();
    SetNeedToUpatesAsTrue();
}

void HUREL::Compton::SessionData::ResetSpectrum(size_t fpgaChannel)
{
    mResetEnergySpectrumMutex.lock();

    mEnergySpectrums[fpgaChannel].Reset();

    mResetEnergySpectrumMutex.unlock();
    SetNeedToUpatesAsTrue();
}

EnergySpectrum HUREL::Compton::SessionData::GetEnergySpectrum(size_t fpgaChannel)
{
    return mEnergySpectrums[fpgaChannel];
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

size_t HUREL::Compton::SessionData::GetSizeListedListModeData()
{
    return mListedListModeData.size();
}

size_t HUREL::Compton::SessionData::GetSizeListedEnergyTimeData()
{
    return mListedEnergyTimeData.size();
}
