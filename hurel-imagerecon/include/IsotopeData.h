#pragma once

#include <string>
#include <map>
#include <vector>
#include <cmath>

namespace HUREL
{
    class IsotopeData;
    namespace Compton
    {
        
		struct sEnergyCheck
		{
			std::string sourceName = "none";
			double minE = 0;
			double maxE = 0;
			bool operator==(const sEnergyCheck &other) const
			{
				return (minE == other.minE && maxE == other.maxE && sourceName == other.sourceName);
			}
			bool operator<(const sEnergyCheck &other) const
			{	
				if (minE == other.minE)
				{
					return (maxE < other.maxE);
				}					
				else
				{
					return (minE < other.minE);
				}
			}
		};
    }; // namespace Compton
    
    
    class IsotopeAnalysis
    {
    private:
        /* data */
        inline static std::map<std::string, IsotopeData> mIsotopeDataList =  std::map<std::string, IsotopeData> ();
        inline static bool mIsotopeDataListInit = false;

    public:
        IsotopeAnalysis() = delete;
        static void Init();

        static std::vector<IsotopeData> FindIsotope(std::vector<double> energy, double refEnergy = 662, double refEnergyFwhm = 50, double fwhmAtZero = 0.1);

    public:
        static double GetFwhm(double energy, double refEnergy = 662, double refEnergyFwhm = 50, double fwhmAtZero = 0.1);
        static Compton::sEnergyCheck GetEnergyCheck(std::string sourceName, double energy);
    };

    class IsotopeData
    {
    private:
        /* data */
        std::string mIsotopeName = "none";
        std::vector<double> mEnergyPeaks = std::vector<double> ();
        int priority = 0;
        std::string mDescription = "none";
        

    public:
        IsotopeData(std::string isotopeName, std::vector<double> energyPeaks, int priority = 0, std::string description = "none") : mIsotopeName(isotopeName), mEnergyPeaks(energyPeaks), priority(priority), mDescription(description) {};
        IsotopeData() {};
        std::vector<Compton::sEnergyCheck> GetEnergyCheck()
        {
            std::vector<Compton::sEnergyCheck> energyChecks;
            for (size_t i = 0; i < mEnergyPeaks.size(); i++)
            {
                Compton::sEnergyCheck energyCheck;
                energyCheck.sourceName = mIsotopeName;
                energyCheck.minE = mEnergyPeaks[i] - IsotopeAnalysis::GetFwhm(mEnergyPeaks[i]) / 2;
                energyCheck.maxE = mEnergyPeaks[i] + IsotopeAnalysis::GetFwhm(mEnergyPeaks[i]) / 2;
                energyChecks.push_back(energyCheck);
            }
            return energyChecks;
        }
        inline std::string GetIsotopeName()
        {
            return mIsotopeName;
        }
        inline std::vector<double> GetEnergyPeaks()
        {
            return mEnergyPeaks;
        }
        inline int GetPriority()
        {
            return priority;
        }
        inline std::string GetDescription()
        {
            return mDescription;
        }

        bool operator==(const IsotopeData &other) const
        {
            return (mIsotopeName == other.mIsotopeName);
        }
    };

} // namespace HUREL
