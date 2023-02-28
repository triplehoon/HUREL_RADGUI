#include "nfd.h"
// #include "glew.h"
// #include "glad/glad.h"


#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"


#include <iostream>
#include <unistd.h>
#include <future>

#include "implot.h"

#include "spdlog/sinks/stdout_color_sinks.h"

#include "MyVisualizer.h"
#include "LahgiControl.h"
#include "SessionData.h"
#include "RadiationImage.h"

#include "ImWindows.h"

#define FRAME_RATE (60)

static void glfw_error_callback(int error, const char *description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

int main(int argv, char **argc)
{

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    spdlog::logger logger("my_logger", {console_sink});

    std::string application_name = "hurel";
    logger.set_pattern("[%H:%M:%S] [" + application_name + "] [%^%l%$] %v");
    auto loggetPtr = std::make_shared<spdlog::logger>(logger);
    spdlog::set_default_logger(loggetPtr);

    {
        char cwd[PATH_MAX];
        if (getcwd(cwd, sizeof(cwd)) == nullptr)
        {
            perror("getdwd() error");
            return 1;
        }

        spdlog::info(std::string("CWD: ") + cwd);
    }
    // open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
    HUREL::Compton::LahgiControl &lahgi = HUREL::Compton::LahgiControl::instance();

    Eigen::Matrix4d testMatrix = Eigen::Matrix4d::Zero();
    lahgi.SetType(HUREL::Compton::eMouduleType::QUAD);

    HUREL::Compton::SessionData *sessionData = nullptr;
    // Setup window

    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
    {
        return 1;
    }
        

    // GL 3.0 + GLSL 130
    // GL 3.2 + GLSL 150
    const char *glsl_version = "#version 150";
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    // glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only

    // Create window with graphics context
    GLFWwindow *window = glfwCreateWindow(1600, 900, "Dear ImGui GLFW+OpenGL3 example", NULL, NULL);
    glfwMaximizeWindow(window);
    if (window == NULL)
        return 1;

    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    ImGuiIO &io = ImGui::GetIO();

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();

    // Setup Korean font
    ImFont *font = io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/nanum/NanumGothic.ttf", 16.0f, nullptr, io.Fonts->GetGlyphRangesKorean());
    if (font == nullptr)
    {
        spdlog::error("Load nanum gothic fail. Please try after do \"sudo apt-install fonts-nanum*\"");
        return 1;
    }
    io.FontDefault = font;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImGuiStyle &style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImGui::StyleColorsLight();

    double *testX = new double[600];
    double *testY = new double[600];

    std::vector<HUREL::Compton::ListModeData> lmData;
    HUREL::Compton::RadiationImage radData(lmData);

    HUREL::MyVisualizer visualizer;

    auto pcdData = open3d::data::DemoICPPointClouds();

    visualizer.InitOpenGL();
    visualizer.CreateVisualizerWindow("Point Cloud Viewer", 500, 500, 50, 50, true);

    GLFWwindow *window2 = visualizer.GetWindow();

    open3d::geometry::PointCloud pc;
    open3d::io::ReadPointCloud("../202211130509Cs137_Right_30degree/20220706_DigitalLabScan_500uCi_30s_1.07_0.08_2.09_Pointcloud.ply", pc);
    auto pc_ptr = std::make_shared<open3d::geometry::PointCloud>(pc);

    visualizer.AddGeometry(pc_ptr);
    pc_ptr->points_.push_back(Eigen::Vector3d(0, 0, 0));
    pc_ptr->colors_.push_back(Eigen::Vector3d(0, 0, 255));

    bool initial = true;
    bool isSizeSet = false;

    std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsedSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(currentTime - lastTime);

        if (elapsedSeconds.count() <= 1.0 / FRAME_RATE) // calc for a milisecond sleep
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(1));
            continue;
        }
        lastTime = currentTime;

        // open3d update polls
        visualizer.UpdateGeometry();
        visualizer.PollEvents();

        glfwMakeContextCurrent(window);

        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();
        if (initial)
        {
            // ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, io.DisplaySize.y));
            // ImGui::SetNextWindowPos(ImVec2(0, 0));
        }

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Open3D window(always open)");
        ImGui::SetWindowSize(ImVec2(1000, 500), ImGuiCond_Once);
        HUREL::GUI::Reconstruction3D(visualizer);
        // ImGui::SetWindowPos(ImVec2(0, 500), ImGuiCond_Once);
        ImVec2 open3dWindowPos = ImGui::GetCursorScreenPos();
        ImVec2 open3dWindowSize = ImGui::GetContentRegionAvail();
        ImGui::End();

        HUREL::GUI::EnergySpectrumWindow(initial, sessionData);

        if (!HUREL::GUI::Reconstrcution2D(initial, sessionData))
        {
            return 1;
        }

        initial = false;
#pragma region rendering

        // Rendering imgui
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(1.0, 1.0, 1.0, 0);
        glClear(GL_COLOR_BUFFER_BIT);
        glfwMakeContextCurrent(window);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        // Update and Render additional Platform Windows
        // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
        //  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }

        glfwSwapBuffers(window);

        // if (!isSizeSet)
        // {

        //     int count;
        //     GLFWmonitor** monitors = glfwGetMonitors(&count);
        //     if (count > 0)
        //     {
        //         GLFWmonitor *primary = monitors[1];
        //     if (primary != nullptr)
        //     {
        //         const GLFWvidmode *mode = glfwGetVideoMode(primary);
        //         int screenWidth = mode->width;
        //         int screenHeight = mode->height;
        //         glfwSetWindowSize(window, screenWidth - screenHeight, screenHeight);
        //         int monitorXpos, monitorYpos;
        //         glfwGetMonitorPos(primary, &monitorXpos, &monitorYpos);
        //         glfwSetWindowPos(window, monitorXpos, monitorYpos);
        //         isSizeSet = true;
        //     }
        //     }

        // }

        glfwGetFramebufferSize(window, &display_w, &display_h);
        int windowXpos, windowYpos;
        glfwGetWindowPos(window, &windowXpos, &windowYpos);

        // rendering open3d
        glfwMakeContextCurrent(window2);
        if (open3dWindowSize.x <= 0 || open3dWindowSize.y <= 0)
        {
            visualizer.SetWinodwVisibility(false);
            visualizer.SetWindowSize(1, 1);
            visualizer.SetWindowPosition(windowXpos, windowYpos);
        }
        else
        {
            visualizer.SetWinodwVisibility(true);
            visualizer.SetWindowSize(open3dWindowSize.x, open3dWindowSize.y);
            visualizer.SetWindowPosition(open3dWindowPos.x, open3dWindowPos.y);
        }

        visualizer.UpdateRender();

        initial = false;

#pragma endregion
    }

    delete[] testX;
    delete[] testY;

    delete sessionData;

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();

    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    visualizer.DestroyVisualizerWindow();

    return 0;
}
