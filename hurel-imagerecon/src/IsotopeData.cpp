#include "IsotopeData.h"


void HUREL::IsotopeAnalysis::Init()
{
    if (mIsotopeDataListInit == false)
    {
        // make isotope data list
        // 137-Cs
        IsotopeData Cs137("Cs137", {661.657}, 0, "산업용");

        // 60-Co
        IsotopeData Co60("Co60", {1173.228, 1332.492}, 1, "산업용");

        // 22-Na
        IsotopeData Na22("Na22", {511.0, 1274.5}, 2, "산업용");

        // 40-K
        IsotopeData K40("K40", {1460.8}, -1, "자연방사선");

        // 241-Am
        IsotopeData Am241("Am241", {59.54}, 3, "산업용");

        // 99m-Tc
        IsotopeData Tc99m("Tc99m", {140.3}, 4, "의료용");

        // 131-I
        IsotopeData I131("I131", {364.0}, 4, "의료용");

        // Depleted Uranium
        IsotopeData DU("DU", {186.2, 344.4, 609.3, 778.5, 964.8, 1120.2, 1460.8, 1764.5, 2614.5, 3447.0}, 1, "핵물질");

        // 208-Tl
        IsotopeData Tl208("Tl208", {2614.5}, -1, "자연방사선");

        // annihilation peak
        IsotopeData Annihilation("Annihilation", {511.0}, 2, "쌍소멸선");

        // Add all IsotopeData to mIsotopeDataList
        mIsotopeDataList.insert(std::make_pair(Cs137.GetIsotopeName(), Cs137));
        mIsotopeDataList.insert(std::make_pair(Co60.GetIsotopeName(), Co60));
        mIsotopeDataList.insert(std::make_pair(Na22.GetIsotopeName(), Na22));
        mIsotopeDataList.insert(std::make_pair(K40.GetIsotopeName(), K40));
        mIsotopeDataList.insert(std::make_pair(Am241.GetIsotopeName(), Am241));
        //mIsotopeDataList.insert(std::make_pair(Tc99m.GetIsotopeName(), Tc99m));
        mIsotopeDataList.insert(std::make_pair(I131.GetIsotopeName(), I131));
        mIsotopeDataList.insert(std::make_pair(DU.GetIsotopeName(), DU));
        mIsotopeDataList.insert(std::make_pair(Tl208.GetIsotopeName(), Tl208));
        mIsotopeDataList.insert(std::make_pair(Annihilation.GetIsotopeName(), Annihilation));

        mIsotopeDataListInit = true;
    }
}

double HUREL::IsotopeAnalysis::GetFwhm(double energy, double refEnergy, double refEnergyFwhm, double fwhmAtZero)
{
    return refEnergyFwhm * sqrt(energy / refEnergy) + fwhmAtZero;
}

HUREL::Compton::sEnergyCheck HUREL::IsotopeAnalysis::GetEnergyCheck(std::string sourceName, double Energy)
{
    if (mIsotopeDataListInit == false)
    {
        Init();
    }
    // find isotope data key
    auto isotopeData = mIsotopeDataList.find(sourceName);
    if (isotopeData != mIsotopeDataList.end())
    {

        for (auto isotopeEnergy : isotopeData->second.GetEnergyPeaks())
        {
            double halfOfFwhm = GetFwhm(isotopeEnergy) / 2;
            if (abs(Energy - isotopeEnergy) < halfOfFwhm)
            {
                return Compton::sEnergyCheck{sourceName, isotopeEnergy - halfOfFwhm, isotopeEnergy + halfOfFwhm};
            }
        }
        return Compton::sEnergyCheck();
    }
}

std::vector<HUREL::IsotopeData> HUREL::IsotopeAnalysis::FindIsotope(std::vector<double> energy, double refEnergy, double refEnergyFwhm, double fwhmAtZero)
{
    Init();
    std::vector<IsotopeData> isotopeDataList;
    for (auto isotopeData : mIsotopeDataList)
    {
        int nPeaks = isotopeData.second.GetEnergyPeaks().size();
        int nPeaksFound = 0;
        for (auto isotopeEnergy : isotopeData.second.GetEnergyPeaks())
        {
            // function of fwhm by energy fwhm = fwhmAtZero * sqrt(energy/refEnergy)
            double energyFwhm = GetFwhm(isotopeEnergy, refEnergy, refEnergyFwhm, fwhmAtZero);
            for (auto e : energy)
            {
                if (abs(e - isotopeEnergy) < energyFwhm / 2)
                {
                    ++nPeaksFound;
                }
            }
        }
        if (nPeaksFound == nPeaks)
        {
            isotopeDataList.push_back(isotopeData.second);
        }
    }
    //if there is annihilation check and remove it
    if (isotopeDataList.size() > 1)
    {
        int i = 0;
        for (auto isotopeData : isotopeDataList)
        {
            if (isotopeData.GetIsotopeName() == "Annihilation")
            {
                break;
            }
            ++i;
        }
        if (i == isotopeDataList.size())
        {
            return isotopeDataList;
        }
        //if any of isotopeData has annihilation peak remove i
        for (auto isotopeData : isotopeDataList)
        {
            if (isotopeData.GetIsotopeName() != "Annihilation")
            {
                for (auto isotopeEnergy : isotopeData.GetEnergyPeaks())
                {
                    if (isotopeEnergy == 511.0)
                    {
                        isotopeDataList.erase(isotopeDataList.begin() + i);
                        return isotopeDataList;
                    }
                }
            }
        }
    }
    return isotopeDataList;
}
