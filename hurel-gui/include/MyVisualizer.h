#pragma once

#include <open3d/Open3D.h>



namespace HUREL
{
    class MyVisualizer : public open3d::visualization::Visualizer
    {
    private:
        bool mIsVisible = true;
        /* data */
    public:
        bool CreateVisualizerWindow(const std::string &window_name = "Open3D",
                                const int width = 640,
                                const int height = 480,
                                const int left = 50,
                                const int top = 50,
                                const bool visible = true);

        void SetWindowPosition(int x, int y);
        void SetWindowSize(int w, int h);
        void SetWinodwVisibility(bool isVisible);
  
    };
    
    
    
} // namespace HUREL

