#pragma once

#include <chrono>
#include <iostream>
#include <time.h>
#include <ctime>
#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "tbb/concurrent_unordered_map.h"

#include "IsotopeData.h"

#include "spdlog/spdlog.h"

#define INTERACTION_GRID_SIZE 120
namespace HUREL
{
	class IsotopeAnalysis;	
	namespace Compton
	{

		enum class eInterationType
		{
			NONE,
			COMPTON,
			CODED
		};

		/// <summary>
		/// meter and keV
		/// </summary>
		struct InteractionData
		{
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			Eigen::Vector4d RelativeInteractionPoint = Eigen::Vector4d(nan(""), nan(""), nan(""), 1);
			Eigen::Vector4d TransformedInteractionPoint = Eigen::Vector4d(nan(""), nan(""), nan(""), 1);
			double InteractionEnergy = 0;
		};

		class ListModeData
		{
		private:
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			eInterationType Type = eInterationType::NONE;
			InteractionData Scatter;
			InteractionData Absorber;
			// time_t InteractionTime = 0;
			std::chrono::milliseconds InteractionTimeInMili = std::chrono::milliseconds(0);
			Eigen::Matrix4d DetectorTransformation = Eigen::Matrix4d::Zero();

			sEnergyCheck EnergyCheck = sEnergyCheck{};

			std::string WriteListModeData();
			bool ReadListModeData(std::string data);
		};

		struct sInteractionData
		{
			Eigen::MatrixXd RelativeInteractionPoint = Eigen::MatrixXd::Zero(INTERACTION_GRID_SIZE, INTERACTION_GRID_SIZE);
			Eigen::Matrix4d DetectorTransformation;
			std::shared_ptr<std::vector<ListModeData>> ComptonListModeData = std::make_shared<std::vector<ListModeData>>(std::vector<ListModeData>());
			//set default time as current time			
			std::chrono::milliseconds StartInteractionTimeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
			std::chrono::milliseconds EndInteractionTimeInMili = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
			size_t interactionCount = 0;

			
		};
	}
}
