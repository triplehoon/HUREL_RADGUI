#pragma once

#include <string>
#include <vector>
#include <chrono>
#include <memory>


namespace HUREL {
	namespace Compton {
		struct BinningEnergy {
			double Energy;
			int Count;
		};
		struct EnergyTimeData {
			int InteractionChannel;
			double Energy;
			std::chrono::milliseconds InteractionTimeInMili;
		};

	};
};