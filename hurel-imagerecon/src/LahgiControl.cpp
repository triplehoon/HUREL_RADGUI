#include "LahgiControl.h"
#include <future>
#include <mutex>
#include <open3d/visualization/utility/DrawGeometry.h>

// static std::mutex mListModeDataMutex;
// static std::mutex mResetListModeDataMutex;
// static std::mutex mListModeImageMutex;
static std::mutex mResetEnergySpectrumMutex;

using namespace HUREL;
using namespace Compton;

ListModeData HUREL::Compton::LahgiControl::MakeListModeData(const eInterationType &iType, Eigen::Vector4d &scatterPoint, Eigen::Vector4d &absorberPoint, double &scatterEnergy, double &absorberEnergy, Eigen::Matrix4d &transformation, std::chrono::milliseconds &timeInMili, sEnergyCheck& echk)
{
	InteractionData scatter;
	InteractionData absorber;
	ListModeData listmodeData;

	if (!isnan(scatterEnergy))
	{
		scatter.InteractionEnergy = scatterEnergy;
	}
	scatter.RelativeInteractionPoint = scatterPoint;
	scatter.TransformedInteractionPoint = transformation * scatterPoint;

	if (!isnan(absorberEnergy))
	{
		absorber.InteractionEnergy = absorberEnergy;
	}
	if (iType == eInterationType::COMPTON)
	{

		absorber.RelativeInteractionPoint = absorberPoint;
		absorber.TransformedInteractionPoint = transformation * absorberPoint;
	}

	listmodeData.Type = iType;
	listmodeData.Scatter = scatter;
	listmodeData.Absorber = absorber;
	listmodeData.DetectorTransformation = transformation;
	listmodeData.InteractionTimeInMili = timeInMili;
	listmodeData.EnergyCheck = echk;

	return listmodeData;
}

HUREL::Compton::LahgiControl::LahgiControl() : mAbsorberModules(NULL),
											   mScatterModules(NULL),
											   mModuleType(HUREL::Compton::eMouduleType::MONO),
											   mCruxellIO(CRUXELL::CruxellIO::instance())
{
	//lahgicontrol 생성자, cruxellIO 생성자도 instance로 생성됨
	Eigen::Matrix4d test;
	test = Matrix4d::Ones();
	Eigen::Vector3d test2 = Eigen::Vector3d(1, 1, 1);

	Eigen::Vector4d test3;
	test3 = Eigen::Vector4d(1, 2, 3, 4);
	test3.normalize();

	t265toLACCPosTransform << 0, 1, 0, T265_TO_LAHGI_OFFSET_X,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Y,
		1, 0, 0, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;

	t265toLACCPosTranslate << 1, 0, 0, T265_TO_LAHGI_OFFSET_X,
		0, 1, 0, T265_TO_LAHGI_OFFSET_Y,
		0, 0, 1, T265_TO_LAHGI_OFFSET_Z,
		0, 0, 0, 1;
	t265toLACCPosTransformInv = t265toLACCPosTransform.inverse();
	spdlog::info("C++HUREL::Compton::LahgiControl: Logger loaded in cpp!");
	mCruxellIO.SetByteDataAddingFunction([](const unsigned short* data){
		HUREL::Compton::LahgiControl::instance().AddShortByteData(data);
	});

	ListModeDataListeningThread = std::async([this]
											 { this->ListModeDataListening(); });
}

void HUREL::Compton::LahgiControl::ListModeDataListening()
{
	mIsListModeDataListeningThreadStart = true;
	size_t bufferSize = 1000000;
	std::vector<std::array<unsigned short, 144>> tempVector;
	tempVector.reserve(bufferSize);
	std::future<void> getEchkFuture;
	bool isEchkFutureReady = true;
	std::chrono::milliseconds timeInMiliBefore = std::chrono::milliseconds(0);
	std::chrono::milliseconds timeInMiliCurrent = std::chrono::milliseconds(0);


	while (mIsListModeDataListeningThreadStart)
	{
		std::array<unsigned short, 144> out;
		
		while (mShortByteDatas.try_pop(out) && tempVector.size() < bufferSize)
		{
			tempVector.push_back(out);
		}
		if (tempVector.size() == 0)
		{
			std::this_thread::sleep_for(std::chrono::nanoseconds(1));
			continue;
		}
		spdlog::debug("C++HUREL::Compton::LahgiControl: ListModeDataListening: tempVector.size() = {}", tempVector.size());
		spdlog::debug("C++HUREL::Compton::LahgiControl: ListModeDataListening: mShortByteDatas.size() = {}", mShortByteDatas.unsafe_size());

		eChksMutex.lock();
		mLiveSessionData.LockMutexAddingData();
		// get current time
		
		
		Eigen::Matrix4d deviceTransformation = t265toLACCPosTransform * RtabmapSlamControl::instance().GetOdomentry() * t265toLACCPosTransformInv * t265toLACCPosTranslate;
		//Eigen::Matrix4d deviceTransformation = t265toLACCPosTransform * RtabmapSlamControl::instance().GetOdomentry() * t265toLACCPosTransformInv;

		timeInMiliCurrent = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		spdlog::debug("C++HUREL::Compton::LahgiControl: ListModeDataListening: elasped time: {} ms", timeInMiliCurrent.count() - timeInMiliBefore.count());
		
		#pragma omp parallel for
		for (int i = 0; i < tempVector.size(); ++i)
		{
			if (timeInMiliBefore != std::chrono::milliseconds(0))
			{
				std::chrono::milliseconds timeInMili = timeInMiliBefore + (timeInMiliCurrent - timeInMiliBefore) * i / tempVector.size();
				AddListModeDataWithTransformationLoop(tempVector[i], timeInMili, deviceTransformation);
			}
		}
		timeInMiliBefore = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		mLiveSessionData.UpdateInteractionImage();
		mLiveSessionData.UnlockMutexAddingData();		
		eChksMutex.unlock();


		// printf("done\n");
		tempVector.clear();

		if (getEchkFuture.valid() && getEchkFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
		{
			getEchkFuture.get();
			isEchkFutureReady = true;
		}
		if (isEchkFutureReady)
		{
			isEchkFutureReady = false;
			getEchkFuture = std::async([this]
									   {
										cv::Mat tempImage = RtabmapSlamControl::instance().GetCurrentVideoFrame();
										if (!tempImage.empty())
										{
											mLiveSessionData.mRgbImage = tempImage;
										}
										tempImage = RtabmapSlamControl::instance().GetCurrentDepthFrame();
										if (!tempImage.empty())
										{
											mLiveSessionData.mDepthImage = tempImage;
										}
										open3d::geometry::PointCloud tempPointCloud;
										tempPointCloud = RtabmapSlamControl::instance().GetSlamPointCloud();
										if (!tempPointCloud.IsEmpty())
										{
											mLiveSessionData.mSlamPointCloud = tempPointCloud;
										}
										pcl::PointCloud<pcl::PointXYZRGB>::Ptr gencloud(new pcl::PointCloud<pcl::PointXYZRGB>);
										*gencloud = *RtabmapSlamControl::instance().generatePointCloud(mLiveSessionData.mDepthImage, mLiveSessionData.mRgbImage);
										tempPointCloud = RtabmapSlamControl::instance().PclToOpen3d(gencloud);
										if (!tempPointCloud.IsEmpty())
										{
											mLiveSessionData.mPointCloud = tempPointCloud;
										}
										
										Eigen::Matrix4d tempTransMatrix = t265toLACCPosTransform * RtabmapSlamControl::instance().GetOdomentry() * t265toLACCPosTransformInv * t265toLACCPosTranslate;
										mLiveSessionData.mTransMatrix = tempTransMatrix;	
								});
		}

		std::this_thread::sleep_for(std::chrono::nanoseconds(1));
	}
}

HUREL::Compton::LahgiControl &HUREL::Compton::LahgiControl::instance()
{
	static LahgiControl &instance = *(new LahgiControl()); //lahgi control 생성자로 이동
	return instance;
}

void HUREL::Compton::LahgiControl::SetEchk(std::vector<sEnergyCheck> eChksInput)
{
	eChksMutex.lock();

	eChk = eChksInput;

	eChksMutex.unlock();
}

bool HUREL::Compton::LahgiControl::SetType(eMouduleType type)
{
	spdlog::info("C++::HUREL::Compton::LahgiControl: Set type");

	mLiveSessionData.Reset();
	mModuleType = type;
	bool successFlag = true;

	switch (type)
	{
	case HUREL::Compton::eMouduleType::MONO:
		assert(false);
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{
		mScatterModules = new Module *[4];
		mAbsorberModules = new Module *[4];
		double offset = 0.083;
		// 3 0
		// 2 1
		//
		double xOffset[4]{-offset, -offset, +offset, +offset};
		double yOffset[4]{+offset, -offset, -offset, +offset};
		std::string scatterSerial = "50777";
		std::string absorberSerial = "51516";
		//std::string scatterSerial = "50776";
		//std::string absorberSerial = "50785";
		
		for (int i = 0; i < 4; ++i)
		{
			double gain[10];

			//double offsetZ = -(0.265);
			double offsetZ = -(0.255);
			//mScatterModules[i] = new Module(eMouduleType::QUAD, "./config/QUAD", scatterSerial + std::string("_Scint") + std::to_string(i), xOffset[i], yOffset[i], -0.055);
			mScatterModules[i] = new Module(eMouduleType::QUAD, "./config/QUAD", scatterSerial + std::string("_Scint") + std::to_string(i), xOffset[i], yOffset[i], -0.0);
			if (!mScatterModules[i]->IsModuleSet())
			{
				successFlag = successFlag & false;
			}
			//mAbsorberModules[i] = new Module(eMouduleType::QUAD, "./config/QUAD", absorberSerial + std::string("_Scint") + std::to_string(i), xOffset[i], yOffset[i], -0.055 + offsetZ);
			mAbsorberModules[i] = new Module(eMouduleType::QUAD, "./config/QUAD", absorberSerial + std::string("_Scint") + std::to_string(i), xOffset[i], yOffset[i], -0.0 + offsetZ);
			if (!mAbsorberModules[i]->IsModuleSet())
			{
				successFlag = successFlag & false;
			}
			
		}

		if (successFlag)
		{
			spdlog::info("Sucess fully load modules (Scatter: " + scatterSerial + ", Absorber: " + absorberSerial + ")");
		}
		break;
	}
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
	{
		break;
	}
	case HUREL::Compton::eMouduleType::TEST:
	{
		mScatterModules = new Module *[1];
		mAbsorberModules = new Module *[1];
		mScatterModules[0] = new Module();
		mAbsorberModules[0] = new Module();
		break;
	}
	default:
		assert(false);
		break;
	}

	return successFlag;
}

HUREL::Compton::LahgiControl::~LahgiControl()
{
	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		assert(false);
		break;
	case HUREL::Compton::eMouduleType::QUAD:
		for (int i = 0; i < 4; ++i)
		{
			delete mScatterModules[i];
			delete mAbsorberModules[i];
		}
		break;
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
		break;
	case HUREL::Compton::eMouduleType::TEST:
		delete mScatterModules[0];
		delete mAbsorberModules[0];
		break;
	default:
		assert(false);
		break;
	}
	delete[] mScatterModules;
	delete[] mAbsorberModules;

	mIsListModeDataListeningThreadStart = false;
	ListModeDataListeningThread.get();
}

void HUREL::Compton::LahgiControl::AddListModeDataWithTransformationLoop(std::array<unsigned short, 144> byteData, std::chrono::milliseconds &timeInMili, Eigen::Matrix4d &deviceTransformation)
{
	switch (mModuleType)
	{
	case HUREL::Compton::eMouduleType::MONO:
		break;
	case HUREL::Compton::eMouduleType::QUAD:
	{
		Eigen::Array<float, 1, 9> scatterShorts[4];
		Eigen::Array<float, 1, 9> absorberShorts[4];

		// Channel 4 to 7
		for (int i = 4; i < 8; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				scatterShorts[i - 4][j] = static_cast<float>(byteData[i * 9 + j]);
			}
		}

		// Channel 12 to 16
		for (int i = 12; i < 16; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				absorberShorts[i - 12][j] = static_cast<float>(byteData[i * 9 + j]);
			}
		}

		double scattersEnergy[4];
		double absorbersEnergy[4];

		int scatterInteractionCount = 0;
		int absorberInteractionCount = 0;
		int scatterInteractModuleNum[4];
		int absorberInteractModuleNum[4];

		//For loop for each scintillator
		for (int i = 0; i < 4; ++i)
		{
			scattersEnergy[i] = mScatterModules[i]->GetEcal(scatterShorts[i]);
			if (!isnan(scattersEnergy[i]))
			{
				EnergyTimeData eTime{i + 4, scattersEnergy[i], timeInMili};
				mScatterModules[i]->GetEnergySpectrum().AddEnergy(scattersEnergy[i]);

				mLiveSessionData.AddEnergyTime(eTime);
				scatterInteractModuleNum[scatterInteractionCount++] = i;
			}
			absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
			if (!isnan(absorbersEnergy[i]))
			{
				EnergyTimeData eTime{i + 12, absorbersEnergy[i], timeInMili};
				mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i]);

				mLiveSessionData.AddEnergyTime(eTime);
				absorberInteractModuleNum[absorberInteractionCount++] = i;
			}
		}

		if (scatterInteractionCount == 1)
		{
			double sEnergy = scattersEnergy[scatterInteractModuleNum[0]];
			double aEnergy = nan("");
			if (absorberInteractionCount == 1)
			{
				double aEnergy = absorbersEnergy[absorberInteractModuleNum[0]];

				// Compton
				for (int i = 0; i < eChk.size(); ++i)
				{

					if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
					{
						eInterationType type = eInterationType::COMPTON;

						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum[0]]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum[0]]);
						Eigen::Vector4d absorberPoint = mAbsorberModules[absorberInteractModuleNum[0]]->FastMLPosEstimation(absorberShorts[absorberInteractModuleNum[0]]);
						ListModeData datum = MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation, timeInMili, eChk[i]);
						mLiveSessionData.AddListModeData(datum);
					}
				}

				// eChksMutex.unlock();
			}
			else if (absorberInteractionCount == 0)
			{
				// Coded Apature
				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy < eChk[i].maxE && sEnergy > eChk[i].minE)
					{
						eInterationType type = eInterationType::CODED;
						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum[0]]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum[0]]);
						Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);
						aEnergy = nan("");
						ListModeData datum = MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation, timeInMili, eChk[i]);
						mLiveSessionData.AddListModeData(datum);
					}
				}
				// eChksMutex.unlock();
			}
		}
		else if (scatterInteractionCount > 1)
		{
			eInterationType type = eInterationType::CODED;
			for (int interDet = 0; interDet < scatterInteractionCount; ++interDet)
			{
				double sEnergy = scattersEnergy[scatterInteractModuleNum[interDet]];
				double aEnergy = nan("");

				for (int i = 0; i < eChk.size(); ++i)
				{
					if (sEnergy < eChk[i].maxE && sEnergy > eChk[i].minE)
					{
						eInterationType type = eInterationType::CODED;
						Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum[interDet]]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum[interDet]]);
						Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);
						ListModeData datum = MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation, timeInMili, eChk[i]);
						mLiveSessionData.AddListModeData(datum);
					}
				}
			}
		}

		break;
	}
	case HUREL::Compton::eMouduleType::QUAD_DUAL:
	{
		break;
	}
	default:
	{
		// Do nothing
		break;
	}
	}
}

void HUREL::Compton::LahgiControl::AddShortByteData(const unsigned short* byteData)
{
	std::array<unsigned short, 144> pushData;
	std::array<unsigned short, 144> pushData2;


	memcpy(&pushData.front(), byteData, 144 * sizeof(unsigned short));
	if (mShortByteDatas.unsafe_size() > 100000000)
	{
		spdlog::warn("ShortByteData is too big");
		return;
	}
	mShortByteDatas.push(pushData);
}
SessionData &HUREL::Compton::LahgiControl::GetLiveSessionData()
{
    // TODO: insert return statement here
	return mLiveSessionData;
}
bool HUREL::Compton::LahgiControl::StartSession()
{
	if (mCruxellIO.IsConnect() && mCruxellIO.IsRunning() )
	{
		spdlog::warn("Session is already running");
		return true;
	}
/*
	if (!LahgiSerialControl::GetInstance().CheckConnection())
	{
		spdlog::warn("Lahgi is not connected");
		//return false;
	}
	if (!LahgiSerialControl::GetInstance().mIsFpgaOn)
	{
		spdlog::warn("FPGA is not on");
		if (LahgiSerialControl::GetInstance().SetFpga(true) == false)
		{
			spdlog::error("Failed to turn on FPGA");
			//return false;
		}		
	}
*/
	mLiveSessionData.Reset();

	ResetEnergySpectrum();

	if (!mCruxellIO.IsConnect())
	{	
		spdlog::warn("Trying to reconnect to CruxellIO");
		mCruxellIO.Connect();
		if (!mCruxellIO.IsConnect())
		{	
			spdlog::error("Failed to connect to CruxellIO");
			return false;
		}
	}

	//try to start SLAM
	RtabmapSlamControl::instance().StartSlamPipe();

	const char* home = getenv("HOME");
	std::string command3 = std::string("mkdir ") + home + std::string("/Documents/hurel/Data/Session_1");
	system(command3.c_str());
    return mCruxellIO.Run();
}

void HUREL::Compton::LahgiControl::StopSession(std::string savePath)
{
	if (mCruxellIO.IsConnect() && mCruxellIO.IsRunning())
	{
		mCruxellIO.Stop();
	}
	else
	{
		spdlog::warn("Session is not running");
	}

	
	//make folder for saving at ~/Documents/Data
	const char* home = getenv("HOME");
	std::string folderPath = "/Documents/hurel/Data/";
	folderPath = home + folderPath;
	//make folders hurel
	std::string command1 = std::string("mkdir ") + home + std::string("/Documents/hurel");
	system(command1.c_str());
	//make folders Data
	std::string command2 = std::string("mkdir ") + home + std::string("/Documents/hurel/Data");
	system(command2.c_str());


	// timeString as "yyyy-mm-dd_hh-mm-ss"
	std::time_t t = std::time(nullptr);
	std::tm tm = *std::localtime(&t);
	std::stringstream ss;
	ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
	std::string timeFormat = ss.str();
	std::string folderName = "Session_" + timeFormat + "_" + savePath;

	//make folder
	std::string folderFullPath = folderPath + folderName;
	std::string command3 = "mkdir " + folderFullPath;
	system(command3.c_str());
	

	open3d::geometry::PointCloud  pc = RtabmapSlamControl::instance().GetSlamPointCloud();
	open3d::io::WritePointCloudOption option;

	open3d::io::WritePointCloudToPLY(folderFullPath + "/_slam.ply", pc, option);

	//try to stop SLAM
	RtabmapSlamControl::instance().StopSlamPipe();
	
	
	mLiveSessionData.Save(folderFullPath);

	mLiveSessionData.Reset();
	
}
bool HUREL::Compton::LahgiControl::GetFPGAStatus()
{
    return true;
}
bool HUREL::Compton::LahgiControl::GetHvStatus()
{
    return true;
}
bool HUREL::Compton::LahgiControl::GetCameraStatus()
{
    return true;
}
/*
void HUREL::Compton::LahgiControl::AddListModeDataWithTransformationVerification(const unsigned short byteData[])
{
    Eigen::Matrix4d t265toLACCPosTransform;
    t265toLACCPosTransform << 0, 1, 0, T265_TO_LAHGI_OFFSET_X,
        0, 0, 1, T265_TO_LAHGI_OFFSET_Y,
        1, 0, 0, T265_TO_LAHGI_OFFSET_Z,
        0, 0, 0, 1;
    Eigen::Matrix4d deviceTransformation = t265toLACCPosTransform * RtabmapSlamControl::instance().GetOdomentry();

    switch (mModuleType)
    {
    case HUREL::Compton::eMouduleType::MONO:
        break;
    case HUREL::Compton::eMouduleType::QUAD:
    {
        const unsigned short *scatterShorts[4];
        const unsigned short *absorberShorts[4];

        for (int i = 4; i < 8; ++i)
        {
            scatterShorts[i - 4] = &byteData[i * 9];
        }

        for (int i = 12; i < 16; ++i)
        {
            absorberShorts[i - 12] = &byteData[i * 9];
        }

        double scattersEnergy[4];
        double absorbersEnergy[4];

        int scatterInteractionCount = 0;
        int absorberInteractionCount = 0;
        int scatterInteractModuleNum = 4;
        int absorberInteractModuleNum = 4;

        for (int i = 0; i < 4; ++i)
        {
            scattersEnergy[i] = mScatterModules[i]->GetEcal(scatterShorts[i]);
            if (!isnan(scattersEnergy[i]))
            {
                mScatterModules[i]->GetEnergySpectrum().AddEnergy(scattersEnergy[i]);
                scatterInteractModuleNum = i;
                ++scatterInteractionCount;
            }
            absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
            if (!isnan(absorbersEnergy[i]))
            {
                mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i]);
                absorberInteractModuleNum = i;
                ++absorberInteractionCount;
            }
        }

        if (scatterInteractionCount == 1)
        {
            double sEnergy = scattersEnergy[scatterInteractModuleNum];
            double aEnergy = absorbersEnergy[absorberInteractModuleNum];

            if (absorberInteractionCount == 1)
            {
                // Compton
                for (int i = 0; i < eChk.size(); ++i)
                {
                    if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
                    {
                        eInterationType type = eInterationType::COMPTON;
                        Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimationVerification(scatterShorts[scatterInteractModuleNum]);
                        Eigen::Vector4d absorberPoint = mAbsorberModules[absorberInteractModuleNum]->FastMLPosEstimationVerification(absorberShorts[absorberInteractModuleNum]);

                        mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
                    }
                }
            }
            else if (absorberInteractionCount == 0)
            {
                // Coded Apature
                for (int i = 0; i < eChk.size(); ++i)
                {
                    if (sEnergy < eChk[i].maxE && sEnergy > eChk[i].minE)
                    {
                        eInterationType type = eInterationType::CODED;
                        Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimationVerification(scatterShorts[scatterInteractModuleNum]);
                        Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);

                        mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
                    }
                }
            }
        }
        else
        {
            eInterationType type = eInterationType::NONE;
        }

        break;
    }
    case HUREL::Compton::eMouduleType::QUAD_DUAL:
    {
        break;
    }
    default:
    {
        // Do nothing
        break;
    }
    }
}

void HUREL::Compton::LahgiControl::AddListModeData(const unsigned short(byteData)[144], Eigen::Matrix4d deviceTransformation)
{
    switch (mModuleType)
    {
    case HUREL::Compton::eMouduleType::MONO:
        break;
    case HUREL::Compton::eMouduleType::QUAD:
    {
        unsigned short scatterShorts[4][9];
        unsigned short absorberShorts[4][9];
        // Channel 4 to 8
        for (int i = 4; i < 8; ++i)
        {
            for (int j = 0; j < 9; ++j)
            {
                scatterShorts[i - 4][j] = byteData[i * 9 + j];
            }
        }

        // Channel 12 to 16
        for (int i = 12; i < 16; ++i)
        {
            for (int j = 0; j < 9; ++j)
            {
                absorberShorts[i - 12][j] = byteData[i * 9 + j];
            }
        }

        double scattersEnergy[4];
        double absorbersEnergy[4];

        int scatterInteractionCount = 0;
        int absorberInteractionCount = 0;
        int scatterInteractModuleNum = 4;
        int absorberInteractModuleNum = 4;

        for (int i = 0; i < 4; ++i)
        {
            scattersEnergy[i] = mScatterModules[i]->GetEcal(scatterShorts[i]);
            if (!isnan(scattersEnergy[i]))
            {
                mScatterModules[i]->GetEnergySpectrum().AddEnergy(scattersEnergy[i]);
                scatterInteractModuleNum = i;
                ++scatterInteractionCount;
            }
            absorbersEnergy[i] = mAbsorberModules[i]->GetEcal(absorberShorts[i]);
            if (!isnan(absorbersEnergy[i]))
            {
                mAbsorberModules[i]->GetEnergySpectrum().AddEnergy(absorbersEnergy[i]);
                absorberInteractModuleNum = i;
                ++absorberInteractionCount;
            }
        }

        if (scatterInteractionCount == 1)
        {
            double sEnergy = scattersEnergy[scatterInteractModuleNum];
            double aEnergy = absorbersEnergy[absorberInteractModuleNum];

            if (absorberInteractionCount == 1)
            {
                // Compton
                eInterationType type = eInterationType::COMPTON;
                for (int i = 0; i < eChk.size(); ++i)
                {
                    if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
                    {

                        Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum]);
                        Eigen::Vector4d absorberPoint = mAbsorberModules[absorberInteractModuleNum]->FastMLPosEstimation(scatterShorts[absorberInteractModuleNum]);
                        mListModeDataMutex.lock();

                        mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
                        mListModeDataMutex.unlock();
                    }
                }
            }
            else if (absorberInteractionCount == 0)
            {
                // Coded Apature
                eInterationType type = eInterationType::CODED;
                for (int i = 0; i < eChk.size(); ++i)
                {
                    if (sEnergy + aEnergy < eChk[i].maxE && sEnergy + aEnergy > eChk[i].minE)
                    {

                        Eigen::Vector4d scatterPoint = mScatterModules[scatterInteractModuleNum]->FastMLPosEstimation(scatterShorts[scatterInteractModuleNum]);
                        Eigen::Vector4d absorberPoint = Eigen::Vector4d(0, 0, 0, 1);
                        mListModeDataMutex.lock();

                        mListedListModeData.push_back(MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation));
                        mListModeDataMutex.unlock();
                    }
                }
            }
        }
        else
        {
            eInterationType type = eInterationType::NONE;
        }

        break;
    }
    case HUREL::Compton::eMouduleType::QUAD_DUAL:
    {
        break;
    }
    default:
    {
        // Do nothing
        break;
    }
    }
}
*/
HUREL::Compton::eMouduleType HUREL::Compton::LahgiControl::GetDetectorType()
{
	return mModuleType;
}

EnergySpectrum &HUREL::Compton::LahgiControl::GetEnergySpectrum(int fpgaChannelNumber)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			return mScatterModules[0]->GetEnergySpectrum();
		}
		else if (fpgaChannelNumber == 8)
		{
			return mAbsorberModules[0]->GetEnergySpectrum();
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber >= 4 && fpgaChannelNumber < 8)
		{
			return mScatterModules[fpgaChannelNumber - 4]->GetEnergySpectrum();
		}
		else if (fpgaChannelNumber >= 12 && fpgaChannelNumber < 16)
		{
			return mAbsorberModules[fpgaChannelNumber - 12]->GetEnergySpectrum();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			return mScatterModules[fpgaChannelNumber]->GetEnergySpectrum();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			return mAbsorberModules[fpgaChannelNumber - 8]->GetEnergySpectrum();
		}
		break;
	}
	EnergySpectrum spect= EnergySpectrum();
	return spect;
}

EnergySpectrum HUREL::Compton::LahgiControl::GetSumEnergySpectrum()
{
	EnergySpectrum spect= EnergySpectrum();
	switch (mModuleType)
	{
	case eMouduleType::MONO:

		spect = mScatterModules[0]->GetEnergySpectrum() + mAbsorberModules[0]->GetEnergySpectrum();
		break;

	case eMouduleType::QUAD:
		for (int i = 0; i < 4; ++i)
		{
			spect = spect + mScatterModules[i]->GetEnergySpectrum();
			spect = spect + mAbsorberModules[i]->GetEnergySpectrum();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		for (int i = 0; i < 8; ++i)
		{
			spect = spect + mScatterModules[i]->GetEnergySpectrum();
			spect = spect + mAbsorberModules[i]->GetEnergySpectrum();
		}
		break;
	case eMouduleType::TEST:
		break;
	}
	return spect;
}

EnergySpectrum HUREL::Compton::LahgiControl::GetAbsorberSumEnergySpectrum()
{
	EnergySpectrum spect = EnergySpectrum();
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		spect = mAbsorberModules[0]->GetEnergySpectrum();
		break;

	case eMouduleType::QUAD:
		for (int i = 0; i < 4; ++i)
		{

			spect = spect + mAbsorberModules[i]->GetEnergySpectrum();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		for (int i = 0; i < 8; ++i)
		{

			spect = spect + mAbsorberModules[i]->GetEnergySpectrum();
		}
		break;
	default:
		break;
	}
	return spect;
}

EnergySpectrum HUREL::Compton::LahgiControl::GetScatterSumEnergySpectrum()
{
	EnergySpectrum spect = EnergySpectrum();
	switch (mModuleType)
	{
	case eMouduleType::MONO:

		spect = mScatterModules[0]->GetEnergySpectrum();
		break;

	case eMouduleType::QUAD:
		for (int i = 0; i < 4; ++i)
		{
			spect = spect + mScatterModules[i]->GetEnergySpectrum();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		for (int i = 0; i < 8; ++i)
		{
			spect = spect + mScatterModules[i]->GetEnergySpectrum();
		}
		break;
	default:
		break;
	}
	return spect;
}

std::tuple<double, double, double> HUREL::Compton::LahgiControl::GetEcalValue(int fpgaChannelNumber)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			return mScatterModules[0]->GetEnergyCalibration();
		}
		else if (fpgaChannelNumber == 8)
		{
			return mAbsorberModules[0]->GetEnergyCalibration();
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber >= 4 && fpgaChannelNumber < 8)
		{
			return mScatterModules[fpgaChannelNumber - 4]->GetEnergyCalibration();
		}
		else if (fpgaChannelNumber >= 12 && fpgaChannelNumber < 16)
		{
			return mAbsorberModules[fpgaChannelNumber - 12]->GetEnergyCalibration();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			return mScatterModules[fpgaChannelNumber]->GetEnergyCalibration();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			return mAbsorberModules[fpgaChannelNumber - 8]->GetEnergyCalibration();
		}
		break;
	default:
		break;
	}
	return std::tuple<double, double, double>();
}

void HUREL::Compton::LahgiControl::SetEcalValue(int fpgaChannelNumber, std::tuple<double, double, double> ecal)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			mScatterModules[0]->SetEnergyCalibration(std::get<0>(ecal), std::get<1>(ecal), std::get<2>(ecal));
		}
		else if (fpgaChannelNumber == 8)
		{
			mAbsorberModules[0]->SetEnergyCalibration(std::get<0>(ecal), std::get<1>(ecal), std::get<2>(ecal));
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber >= 4 && fpgaChannelNumber < 8)
		{
			mScatterModules[fpgaChannelNumber - 4]->SetEnergyCalibration(std::get<0>(ecal), std::get<1>(ecal), std::get<2>(ecal));
		}
		else if (fpgaChannelNumber >= 12 && fpgaChannelNumber < 16)
		{
			mAbsorberModules[fpgaChannelNumber - 12]->SetEnergyCalibration(std::get<0>(ecal), std::get<1>(ecal), std::get<2>(ecal));
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			mScatterModules[fpgaChannelNumber]->SetEnergyCalibration(std::get<0>(ecal), std::get<1>(ecal), std::get<2>(ecal));
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			mAbsorberModules[fpgaChannelNumber - 8]->SetEnergyCalibration(std::get<0>(ecal), std::get<1>(ecal), std::get<2>(ecal));
		}
		break;
	default:
		break;
	}
}

void HUREL::Compton::LahgiControl::ResetEnergySpectrum()
{
	for (int i = 0; i < 16; ++i)
	{
		ResetEnergySpectrum(i);
	}
}

void HUREL::Compton::LahgiControl::ResetEnergySpectrum(int fpgaChannelNumber)
{
	switch (mModuleType)
	{
	case eMouduleType::MONO:
		if (fpgaChannelNumber == 0)
		{
			mScatterModules[0]->GetEnergySpectrum().Reset();
		}
		else if (fpgaChannelNumber == 8)
		{
			mAbsorberModules[0]->GetEnergySpectrum().Reset();
		}
		break;

	case eMouduleType::QUAD:
		if (fpgaChannelNumber >= 4 && fpgaChannelNumber < 8)
		{
			mScatterModules[fpgaChannelNumber - 4]->GetEnergySpectrum().Reset();
		}
		else if (fpgaChannelNumber >= 12 && fpgaChannelNumber < 16)
		{
			mAbsorberModules[fpgaChannelNumber - 12]->GetEnergySpectrum().Reset();
		}
		break;
	case eMouduleType::QUAD_DUAL:
		if (fpgaChannelNumber >= 0 && fpgaChannelNumber < 8)
		{
			mScatterModules[fpgaChannelNumber]->GetEnergySpectrum().Reset();
		}
		else if (fpgaChannelNumber >= 8 && fpgaChannelNumber < 16)
		{
			mAbsorberModules[fpgaChannelNumber - 8]->GetEnergySpectrum().Reset();
		}
		break;
	default:
		break;
	}
	return;
}
