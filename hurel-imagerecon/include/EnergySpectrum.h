#pragma once
#define PY_SSIZE_T_CLEAN
//#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <python3.8/Python.h>
//include numpy array
#include <numpy/arrayobject.h>

#include <thread>
#include <string>
#include <vector>
#include <chrono>
#include <memory>
#include <tbb/concurrent_vector.h>
#include <mutex>
#include <fstream>
#include <iostream>
#include <sstream>

#include "spdlog/spdlog.h"

#include "EnergySpectrumData.h"

#define ENERGY_SPECTRUM_BIN_SIZE (10)
#define ENERGY_SPECTRUM_MAX (3000)
#define DOSE_RATE_COEFFICIENT (3.8)

namespace HUREL {
	namespace Compton {
		class EnergySpectrum
		{
		public:
			EnergySpectrum();
			
			std::vector<BinningEnergy> GetHistogramEnergy();
			void AddEnergy(double Energy);
			void Reset();
			EnergySpectrum operator+ (const EnergySpectrum& rhs) const;
			static std::vector<double> GetSpectrumPeaks(EnergySpectrum& spectrum,
														std::vector<double>* outSnrVector,
														double ref_x = 662, 
														double ref_fwhm = 20, 
														double fwhm_at_0 = 1, 
														double min_snr = 5);
			static void EnergySpectrumTestCode();	
			/// @brief 
			/// @param spect 
			/// @return get dose rate as uSv/h 
			static double GetDoseRate(EnergySpectrum& spect, double timeInSecond);

		private:
			EnergySpectrum(unsigned int binSize, double maxEnergy);

			inline static bool mIsDoseRateInitComplete = false;
			
			inline static std::vector<double> GCoeff = std::vector<double>();
			inline static std::vector<double> H10Coeff = std::vector<double>();
			inline static std::vector<double> KermaCoeff = std::vector<double>();
			static void InitDoseRate();
			inline static bool mIsPythonInitialized = false;
			std::vector<BinningEnergy> mHistogramEnergy = std::vector<BinningEnergy>();
			std::vector<double> mEnergyBin = std::vector<double>();
			double mBinSize = 0;
			double mMaxEnergy = 0;
		};
	}
}


