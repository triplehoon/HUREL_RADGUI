#pragma once
#include <future>

#include "imgui.h"
#include "implot.h"
#include "nfd.h"

#include "SessionData.h"
#include "LahgiControl.h"

#include "MyVisualizer.h"

namespace HUREL {
    class GUI{

    private: 
        inline static bool mIsRealtimeRun = false;
        inline static bool isImageGet = false;

    public:    
        static void Reconstruction3D(HUREL::MyVisualizer& vis);
        static void EnergySpectrumWindow(bool initial, Compton::SessionData*& sessionDataOrNull);
        static bool Reconstrcution2D(bool initial, Compton::SessionData*& sessionDataOrNUll);
        static void InformationWindow(bool initial, Compton::SessionData*& sessionDataOrNull);
        static void ControlWindow(bool initial, Compton::SessionData*& sessionDataOrNull);
    };

};

