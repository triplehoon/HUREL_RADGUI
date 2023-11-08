#pragma once
#include <future>
#include <map>  

#include "imgui.h"
#include "imgui_internal.h"
#include "implot.h"
#include "nfd.h"

#include "SessionData.h"
#include "LahgiControl.h"
#include "IsotopeData.h"

#include "MyVisualizer.h"

namespace HUREL {
    class GUI{

    private: 
        inline static bool mIsRealtimeRun = true;
        inline static bool isImageGet = false;

        inline static bool mIsSettingValuesInit = false;

        inline static std::map<std::string, std::string> mSettingValues = std::map<std::string, std::string>();

        inline static std::vector<HUREL::IsotopeData> mIsotopeDataList = std::vector<HUREL::IsotopeData>();
        inline static HUREL::IsotopeData mSelectedIsotopeData = HUREL::IsotopeData();

        static void ChangeSettingValues(std::string key, std::string value);
    public:
        static void Init();
        static void Reconstruction3D(HUREL::MyVisualizer& vis);
        static void EnergySpectrumWindow(bool initial, Compton::SessionData*& sessionDataOrNull);
        static bool Reconstrcution2D(bool initial, Compton::SessionData*& sessionDataOrNUll);
        static void InformationWindow(bool initial, Compton::SessionData*& sessionDataOrNull);
        static void ControlWindow(bool initial, Compton::SessionData*& sessionDataOrNull);
        static void SettingWindow(bool initial, Compton::SessionData*& sessionDataOrNull);
        static void MakeBeep(double timeItervalInSecond);
    };

};

