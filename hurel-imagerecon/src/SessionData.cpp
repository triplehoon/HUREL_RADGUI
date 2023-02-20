#include "SessionData.h"

using namespace HUREL;
using namespace Compton;

HUREL::Compton::SessionData::SessionData() : mIsLoaded(false),
                                             mLoadedDataStartTime(std::chrono::milliseconds::zero()),
                                             mLoadFileDir("")
{
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
        datum.InteractionTimeInMili  = std::chrono::milliseconds(stoll(words[0]));
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

    //need to load slam and realtime pointcloud;
    mLoadFileDir = fileDir;
    mIsLoaded = true;

    spdlog::info("C++HUREL::Compton::LahgiControl: Load lm data: {0}", fileDir);
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

std::vector<ListModeData> HUREL::Compton::SessionData::GetListedListModeData(long long timeInMililseconds)
{
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

std::vector<ListModeData> HUREL::Compton::SessionData::GetListedListModeDataCount(size_t count)
{
    size_t size;
    if (mListedListModeData.size() >= count)
    {
        size = count;
    }
    else
    {
        size = mListedListModeData.size();
    }
    std::vector<ListModeData> lmData;
    lmData.reserve(size);
    for (int i = mListedListModeData.size() - 1; i >= mListedListModeData.size() - size; --i)
    {
        lmData.push_back(mListedListModeData[i]);
    }
    return lmData;
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

void HUREL::Compton::SessionData::AddEnergyTime(EnergyTimeData &datum)
{
    mListedEnergyTimeData.push_back(datum);
}

void HUREL::Compton::SessionData::AddListModeData(ListModeData &datum)
{
    mListedListModeData.push_back(datum);
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

    mResetListModeDataMutex.unlock();
    mResetEnergySpectrumMutex.unlock();
}

void HUREL::Compton::SessionData::ResetSpectrum()
{
    mResetEnergySpectrumMutex.lock();
    for (int i = 0; i < 16; ++i)
    {
        mEnergySpectrums[i].Reset();
    }

    mResetEnergySpectrumMutex.unlock();
}

void HUREL::Compton::SessionData::ResetSpectrum(size_t fpgaChannel)
{
    mResetEnergySpectrumMutex.lock();

    mEnergySpectrums[fpgaChannel].Reset();

    mResetEnergySpectrumMutex.unlock();
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
