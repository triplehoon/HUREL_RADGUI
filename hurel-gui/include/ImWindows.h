#pragma once
#include "imgui.h"
#include "implot.h"
#include "nfd.h"

#include "SessionData.h"

#include "MyVisualizer.h"

namespace HUREL {
    class GUI{
    public:
        static void Reconstruction3D(HUREL::MyVisualizer& vis);
        static void EnergySpectrumWindow(bool initial, Compton::SessionData*& sessionDataOrNull);
        static bool Reconstrcution2D(bool initial, Compton::SessionData*& sessionDataOrNUll);
    };

};

