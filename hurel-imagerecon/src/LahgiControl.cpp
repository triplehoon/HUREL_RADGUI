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

ListModeData HUREL::Compton::LahgiControl::MakeListModeData(const eInterationType &iType, Eigen::Vector4d &scatterPoint, Eigen::Vector4d &absorberPoint, double &scatterEnergy, double &absorberEnergy, Eigen::Matrix4d &transformation, std::chrono::milliseconds &timeInMili)
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

	return listmodeData;
}

ListModeData HUREL::Compton::LahgiControl::MakeListModeData(const eInterationType &iType, Eigen::Vector4d &scatterPoint, Eigen::Vector4d &absorberPoint, double &scatterEnergy, double &absorberEnergy, Eigen::Matrix4d &transformation)
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
	listmodeData.InteractionTimeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	;

	return listmodeData;
}

HUREL::Compton::LahgiControl::LahgiControl() : mAbsorberModules(NULL),
											   mScatterModules(NULL),
											   mModuleType(HUREL::Compton::eMouduleType::MONO)
{
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

	ListModeDataListeningThread = std::async([this]
											 { this->ListModeDataListening(); });
}

void HUREL::Compton::LahgiControl::ListModeDataListening()
{
	mIsListModeDataListeningThreadStart = true;
	size_t bufferSize = 1000000;
	std::vector<std::array<unsigned short, 144>> tempVector;
	tempVector.reserve(bufferSize);

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
		mLiveSessionData.LockMutexAddingData();
		eChksMutex.lock();

		std::chrono::milliseconds timeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
		Eigen::Matrix4d deviceTransformation = t265toLACCPosTransform * RtabmapSlamControl::instance().GetOdomentry() * t265toLACCPosTransformInv * t265toLACCPosTranslate;

#pragma omp parallel for
		for (int i = 0; i < tempVector.size(); ++i)
		{
			AddListModeDataWithTransformationLoop(tempVector[i], timeInMili, deviceTransformation);
		}

		mLiveSessionData.UnlockMutexAddingData();
		eChksMutex.unlock();
		// printf("done\n");
		tempVector.clear();
		std::this_thread::sleep_for(std::chrono::nanoseconds(1));
	}
}

HUREL::Compton::LahgiControl &HUREL::Compton::LahgiControl::instance()
{
	static LahgiControl &instance = *(new LahgiControl());
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
		std::string absorberSerial = "50784";
		for (int i = 0; i < 4; ++i)
		{
			double gain[10];

			double offsetZ = -(0.245);
			mScatterModules[i] = new Module(eMouduleType::QUAD, "./config/QUAD", scatterSerial + std::string("_Scint") + std::to_string(i), xOffset[i], yOffset[i], -0.055);
			if (!mScatterModules[i]->IsModuleSet())
			{
				successFlag = successFlag & false;
			}
			mAbsorberModules[i] = new Module(eMouduleType::QUAD, "./config/QUAD", absorberSerial + std::string("_Scint") + std::to_string(i), xOffset[i], yOffset[i], -0.055 + offsetZ);
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
		// double offset = 0.083;

		// double xOffset[8]{ -offset, +offset, -offset, +offset };
		// double yOffset[8]{ -offset, -offset, +offset, +offset };
		// mScatterModules = new Module * [8];
		// mAbsorberModules = new Module * [8];
		// for (int i = 0; i < 8; ++i)
		//{
		//	string slutFileDirectory = string("config\\QUAD\\Scatter\\LUT\\Lut_scintillator_") + to_string(i) + string(".csv");
		//	string sgainFileDirectory = string("config\\QUAD\\Scatter\\Gain\\Energy_gain_scintillator_") + to_string(i) + string(".csv");

		//	string alutFileDirectory = string("config\\QUAD\\Absorber\\LUT\\Lut_scintillator_") + to_string(i) + string(".csv");
		//	string againFileDirectory = string("config\\QUAD\\Absorber\\Gain\\Energy_gain_scintillator_") + to_string(i) + string(".csv");

		//	double gain[9];
		//	Module::LoadGain(sgainFileDirectory, type, gain);

		//	double offsetZ = -(0.251 + (31.5 - 21.5) / 1000);
		//	mScatterModules[i] = new Module(eMouduleType::QUAD, gain, gain, slutFileDirectory, xOffset[i], yOffset[i]);

		//	Module::LoadGain(againFileDirectory, type, gain);
		//	mAbsorberModules[i] = new Module(eMouduleType::QUAD, gain, gain, alutFileDirectory, xOffset[i], yOffset[i]);

		//}
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
		/*mScatterModules = new Module * [8];
		mAbsorberModules = new Module * [8];
		for (int i = 0; i < 8; ++i)
		{
			delete mScatterModules[i];
			delete mAbsorberModules[i];
		}*/
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
				scatterShorts[i - 4][j] = static_cast<double>(byteData[i * 9 + j]);
			}
		}

		// Channel 12 to 16
		for (int i = 12; i < 16; ++i)
		{
			for (int j = 0; j < 9; ++j)
			{
				absorberShorts[i - 12][j] = static_cast<double>(byteData[i * 9 + j]);
			}
		}

		double scattersEnergy[4];
		double absorbersEnergy[4];

		int scatterInteractionCount = 0;
		int absorberInteractionCount = 0;
		int scatterInteractModuleNum[4];
		int absorberInteractModuleNum[4];

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
						ListModeData datum = MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation, timeInMili);
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
						ListModeData datum = MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation, timeInMili);
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
						ListModeData datum = MakeListModeData(type, scatterPoint, absorberPoint, sEnergy, aEnergy, deviceTransformation, timeInMili);
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
/*
void HUREL::Compton::LahgiControl::AddListModeDataWithTransformation(const unsigned short byteData[144])
{
	std::array<unsigned short, 144> pushData;

	memcpy(&pushData.front(), byteData, 144 * sizeof(unsigned short));
	mShortByteDatas.push(pushData);
}

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

size_t HUREL::Compton::LahgiControl::GetListedListModeDataSize()
{
	return mLiveSessionData.GetSizeListedListModeData();
}

std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData()
{
	return mLiveSessionData.GetListedListModeData(0);
}

std::vector<ListModeData> HUREL::Compton::LahgiControl::GetListedListModeData(long long timeInMililseconds)
{
	return mLiveSessionData.GetListedListModeData(timeInMililseconds);
}

std::vector<EnergyTimeData> HUREL::Compton::LahgiControl::GetListedEnergyTimeData(long long timeInMiliSecond)
{
	return mLiveSessionData.GetListedEnergyTimeData(timeInMiliSecond);
}
std::vector<EnergyTimeData> HUREL::Compton::LahgiControl::GetListedEnergyTimeData()
{
	return mLiveSessionData.GetListedEnergyTimeData(0);
}

void HUREL::Compton::LahgiControl::ResetListedListModeData()
{
	mLiveSessionData.Reset();
}

void HUREL::Compton::LahgiControl::SaveListedListModeData(std::string fileDir)
{
	mLiveSessionData.Save(fileDir);
}

bool replace(std::string &str, const std::string &from, const std::string &to)
{
	size_t start_pos = str.find(from);
	if (start_pos == std::string::npos)
		return false;
	str.replace(start_pos, from.length(), to);
	return true;
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
	default:
		EnergySpectrum e = EnergySpectrum(5, 3000);
		return e;
		break;
	}
}

EnergySpectrum HUREL::Compton::LahgiControl::GetSumEnergySpectrum()
{
	EnergySpectrum spect(5, 3000);
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
	EnergySpectrum spect(5, 3000);
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
	EnergySpectrum spect(5, 3000);
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
/*
ReconPointCloud HUREL::Compton::LahgiControl::GetReconRealtimePointCloudComptonUntransformed(open3d::geometry::PointCloud &outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

	std::vector<ListModeData> tempLMData = GetListedListModeData();

	// std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	// std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if ((t - tempLMData[i].InteractionTimeInMili).count() < seconds * 1000)
		{
			reconStartIndex = i;
			break;
		}
	}
#pragma omp parallel for
	for (int i = reconStartIndex; i < tempLMData.size(); ++i)
	{
		reconPC.CalculateReconPoint(tempLMData[i], ReconPointCloud::SimpleComptonBackprojectionUntransformed);
	}

	std::cout << "End Recon: " << tempLMData.size() << std::endl;

	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconRealtimePointCloudCompton(open3d::geometry::PointCloud &outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

	std::vector<ListModeData> tempLMData = GetListedListModeData();

	// std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	// std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (seconds == 0)
		{
			reconStartIndex = 0;
			break;
		}
		if (t.count() - tempLMData[i].InteractionTimeInMili.count() < static_cast<int64_t>(seconds))
		{
			reconStartIndex = i;
			break;
		}
	}

	std::vector<ListModeData> reconLm;
	reconLm.reserve(tempLMData.size());
	// assert(0, "temp energy search");
	for (const auto lm : tempLMData)
	{
		if (lm.Absorber.InteractionEnergy + lm.Scatter.InteractionEnergy > 620 && lm.Scatter.InteractionEnergy + lm.Absorber.InteractionEnergy < 700)
		{
			reconLm.push_back(lm);
		}
	}

#pragma omp parallel for
	for (int i = 0; i < reconLm.size(); ++i)
	{
		reconPC.CalculateReconPoint(reconLm[i], ReconPointCloud::SimpleComptonBackprojection);
	}
	// HUREL::Logger::Instance().InvokeLog("C++HUREL::Compton::LahgiControl", "GetReconRealtimePointCloudCompton End Recon: " + reconLm.size(), eLoggerType::INFO);

	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconOverlayPointCloudCoded(open3d::geometry::PointCloud &outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

	std::vector<RadiationImage> tempLMData;

	// std::cout << "Start Recon (LM): " << tempLMData.size() << std::endl;
	// std::cout << "Start Recon (PC): " << reconPC.points_.size() << std::endl;
	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t.count() - tempLMData[i].mListedListModeData[0].InteractionTimeInMili.count() < static_cast<int64_t>(seconds))
		{
			reconStartIndex = i;
			break;
		}
	}

	for (int i = reconStartIndex; i < tempLMData.size(); ++i)
	{
		reconPC.CalculateReconPointCoded(tempLMData[i]);
	}
	std::cout << "End GetReconOverlayPointCloudCoded: " << tempLMData.size() << std::endl;

	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconOverlayPointCloudCompton(open3d::geometry::PointCloud &outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
	mListModeImageMutex.lock();

	std::vector<RadiationImage> tempLMData;
	mListModeImageMutex.unlock();

	int reconStartIndex = 0;
	for (int i = 0; i < tempLMData.size(); ++i)
	{

		if (t.count() - tempLMData[i].mListedListModeData[0].InteractionTimeInMili.count() < static_cast<int64_t>(seconds))
		{
			reconStartIndex = i;
			break;
		}
	}

	for (int i = reconStartIndex; i < tempLMData.size(); ++i)
	{
		reconPC.CalculateReconPointCompton(tempLMData[i]);
	}
	// std::cout << "End GetReconOverlayPointCloudCompton: " << tempLMData.size() << std::endl;

	return reconPC;
}

ReconPointCloud HUREL::Compton::LahgiControl::GetReconOverlayPointCloudHybrid(open3d::geometry::PointCloud &outPC, double seconds)
{
	HUREL::Compton::ReconPointCloud reconPC = HUREL::Compton::ReconPointCloud(outPC);

	std::vector<ListModeData> lmData = GetListedListModeData();
	RadiationImage radimg = RadiationImage(lmData);
	reconPC.CalculateReconPointHybrid(radimg);

	return reconPC;
}

inline int findIndex(double value, double min, double pixelSize)
{
	if (value - min <= 0)
	{
		return -1;
	}
	return static_cast<int>(floor((value + 0.00001 - min) / pixelSize));
}

cv::Mat HUREL::Compton::LahgiControl::GetResponseImage(int imgSize, int pixelCount, double timeInSeconds, bool isScatter)
{
	std::chrono::milliseconds t = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());

	std::vector<ListModeData> tempLMData = GetListedListModeData(timeInSeconds * 1000);

	constexpr double Det_W = 0.400;
	Mat responseImg(pixelCount, pixelCount, CV_32S, Scalar(0));
	int32_t *responseImgPtr = static_cast<int32_t *>(static_cast<void *>(responseImg.data));

	for (ListModeData lm : tempLMData)
	{

		double interactionPoseX = -999999;
		double interactionPoseY = -999999;

		switch (lm.Type)
		{
		case eInterationType::COMPTON:
			if (isScatter)
			{
				interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
				interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];
			}
			else
			{
				interactionPoseX = lm.Absorber.RelativeInteractionPoint[0];
				interactionPoseY = lm.Absorber.RelativeInteractionPoint[1];
			}

			break;
		case eInterationType::CODED:
			if (isScatter)
			{
				interactionPoseX = lm.Scatter.RelativeInteractionPoint[0];
				interactionPoseY = lm.Scatter.RelativeInteractionPoint[1];
			}
			break;
		default:
			continue;
			break;
		}

		int iX = findIndex(interactionPoseX, -Det_W / 2, Det_W / pixelCount);
		int iY = findIndex(interactionPoseY, -Det_W / 2, Det_W / pixelCount);
		if (iX >= 0 && iY >= 0 && iX < pixelCount && iY < pixelCount)
		{
			++responseImgPtr[pixelCount * iY + iX];
		}
	}

	return HUREL::Compton::RadiationImage::GetCV_32SAsJet(responseImg, imgSize);
}
*/