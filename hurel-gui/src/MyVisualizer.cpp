#include "MyVisualizer.h"

bool HUREL::MyVisualizer::CreateVisualizerWindow(const std::string &window_name, const int width, const int height, const int left, const int top, const bool visible)
{    
    bool isValid = open3d::visualization::Visualizer::CreateVisualizerWindow(window_name, width, height, left, top, visible);
    glfwSetWindowAttrib(window_, GLFW_DECORATED, GLFW_FALSE);
    return isValid;
}

void HUREL::MyVisualizer::SetWindowPosition(int x, int y)
{
    if (window_ == nullptr || !mIsVisible)
    {
        return;
    }
    glfwSetWindowPos(window_, x, y);
    glfwSetWindowAttrib(window_, GLFW_FLOATING, GL_TRUE);
}

void HUREL::MyVisualizer::SetWindowSize(int w, int h)
{
    if (window_ == nullptr || !mIsVisible)
    {
        return;
    }
    glfwSetWindowSize(window_, w, h);
}

void HUREL::MyVisualizer::SetWinodwVisibility(bool isVisible)
{
    if (window_ == nullptr)
    {
        return;
    }
    // if it is different, update
    if (isVisible == mIsVisible)
    {
        return;
    }

}

GLFWwindow *HUREL::MyVisualizer::GetWindow()
{
    return window_;
}

bool HUREL::MyVisualizer::InitOpenGL()
{
    return open3d::visualization::Visualizer::InitOpenGL();
}
