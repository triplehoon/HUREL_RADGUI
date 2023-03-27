#pragma once

#include <chrono>
#include <iostream>
#include <time.h>
#include <ctime>
#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "spdlog/spdlog.h"

namespace HUREL
{
	namespace Compton
	{

		struct sEnergyCheck
		{
			std::string sourceName = "none";
			double minE;
			double maxE;
			bool operator==(const sEnergyCheck &other) const
			{
				return (minE == other.minE && maxE == other.maxE);
			}
		};
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

			sEnergyCheck EnergyCheck;

			std::string WriteListModeData();
			bool ReadListModeData(std::string data);
		};

	   struct sInteractionData
        {
            Eigen::MatrixXd RelativeInteractionPoint = Eigen::MatrixXd::Zero(60,60);
            Eigen::Matrix4d DetectorTransformation;
            sEnergyCheck EnergyCheck;
            std::chrono::milliseconds StartInteractionTimeInMili;
            std::chrono::milliseconds EndInteractionTimeInMili;
        };
	}
}
