#include "nfd.h"
// #include "glew.h"
// #include "glad/glad.h"

#include <open3d/Open3D.h>

#include <iostream>
#include <unistd.h>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "implot.h"

#include "spdlog/sinks/stdout_color_sinks.h"

#include "MyVisualizer.h"
#include "LahgiControl.h"
#include "SessionData.h"
#include "RadiationImage.h"

#define FRAME_RATE (60)

static void glfw_error_callback(int error, const char *description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void overlayImage(const cv::Mat &background, const cv::Mat &foreground,
                  cv::Mat &output)
{
    background.copyTo(output);

    cv::Point2i location = cv::Point2i(0, 0);
    // start at the row indicated by location, or at row 0 if location.y is negative.
    for (int y = std::max(location.y, 0); y < background.rows; ++y)
    {
        int fY = y - location.y; // because of the translation

        // we are done of we have processed all rows of the foreground image.
        if (fY >= foreground.rows)
            break;

        // start at the column indicated by location,

        // or at column 0 if location.x is negative.
        for (int x = std::max(location.x, 0); x < background.cols; ++x)
        {
            int fX = x - location.x; // because of the translation.

            // we are done with this row if the column is outside of the foreground image.
            if (fX >= foreground.cols)
                break;

            // determine the opacity of the foregrond pixel, using its fourth (alpha) channel.
            double opacity =
                ((double)foreground.data[fY * foreground.step + fX * foreground.channels() + 3])

                / 255.;

            // and now combine the background and foreground pixel, using the opacity,

            // but only if opacity > 0.
            for (int c = 0; opacity > 0 && c < output.channels(); ++c)
            {
                unsigned char foregroundPx =
                    foreground.data[fY * foreground.step + fX * foreground.channels() + c];
                unsigned char backgroundPx =
                    background.data[y * background.step + x * background.channels() + c];
                output.data[y * output.step + output.channels() * x + c] =
                    backgroundPx * (1. - opacity) + foregroundPx * opacity;
            }
        }
    }
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
    //open3d::utility::SetVerbosityLevel(open3d::utility::VerbosityLevel::Debug);
    HUREL::Compton::LahgiControl &lahgi = HUREL::Compton::LahgiControl::instance();

    Eigen::Matrix4d testMatrix = Eigen::Matrix4d::Zero();
    lahgi.SetType(HUREL::Compton::eMouduleType::QUAD);

    HUREL::Compton::SessionData *sessionData = nullptr;
    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;
#
    // GL 3.0 + GLSL 130
    // GL 3.2 + GLSL 150
    const char *glsl_version = "#version 150";
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
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

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    ImGui::StyleColorsLight();

    double *testX = new double[600];
    double *testY = new double[600];

    std::vector<HUREL::Compton::ListModeData> lmData;
    HUREL::Compton::RadiationImage radData(lmData);
    bool isLogScale = false;
    bool isImageGet = false;
    float s2M = 2;
    float det_W = 0.312;
    float resImprov = 8;
    float m2D = 0.063;
    float hFov = 60;
    float wFov = 90;

    GLuint texture;
    GLuint texture2;
    cv::Mat image;
    cv::Mat radImage;
    cv::Mat radImageOveray;

    HUREL::MyVisualizer visualizer;

    auto pcdData = open3d::data::DemoICPPointClouds();

    visualizer.InitOpenGL();
    visualizer.CreateVisualizerWindow("Point Cloud Viewer", 500, 500, 50, 50, true);

    GLFWwindow *window2 = visualizer.window_;

    open3d::geometry::PointCloud pc;
    open3d::io::ReadPointCloud(pcdData.GetPaths(0), pc);
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
        // ImGui::SetWindowSize(ImVec2(1000, 500), ImGuiCond_Once);
        // ImGui::SetWindowPos(ImVec2(0, 500), ImGuiCond_Once);
        ImVec2 open3dWindowPos = ImGui::GetCursorScreenPos();
        ImVec2 open3dWindowSize = ImGui::GetContentRegionAvail();
        ImGui::End();

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to created a named window.
        {
            static float f = 0.0f;
            static int counter = 1;

            ImGui::Begin("안녕세상야", nullptr, ImGuiWindowFlags_NoBringToFrontOnFocus); // Create a window called "Hello, world!" and append into it.
            if (ImGui::Button("test"))
            {
                pc_ptr->points_.push_back(Eigen::Vector3d(0, 0, counter));
                pc_ptr->colors_.push_back(Eigen::Vector3d(0, 0, 255));

                ++counter;
            }
            if (initial)
            {
                // ImGui::SetWindowPos(ImVec2(0, 0), 0);
                // ImGui::SetWindowSize(ImVec2(1000, 500), 0);
            }

            if (sessionData != nullptr)
            {
                std::vector<HUREL::Compton::BinningEnergy> spect = sessionData->GetEnergySpectrum(nullptr, 0, 0).GetHistogramEnergy();
                for (int i = 0; i < spect.size(); ++i)
                {
                    testX[i] = spect.at(i).Energy;
                    testY[i] = spect.at(i).Count;
                }
            }

            ImPlot::PushStyleColor(0, ImVec4(0, 0, 0, 1));
            ImPlot::PushStyleColor(0, ImVec4(0, 0, 0, 1));
            ImGui::Checkbox("Log scale", &isLogScale);

            ImPlotFlags plotFlag = ImPlotFlags_None;
            if (ImPlot::BeginPlot("Energy Spectrum", ImVec2(500, 500), plotFlag))
            {
                ImPlot::SetupAxes("Count [#]", "Energy [keV]",
                                  ImPlotAxisFlags_::ImPlotAxisFlags_LockMax | ImPlotAxisFlags_::ImPlotAxisFlags_LockMin,
                                  ImPlotAxisFlags_::ImPlotAxisFlags_LockMin);

                ImPlot::SetupAxisLimits(ImAxis_::ImAxis_X1, 0, 3000, ImPlotCond_::ImPlotCond_Once);
                if (isLogScale)
                {
                    ImPlot::SetupAxisScale(ImAxis_::ImAxis_Y1, ImPlotScale_Log10);
                    ImPlot::SetupAxisLimits(ImAxis_::ImAxis_Y1, 1, 3000, ImPlotCond_::ImPlotCond_Once);
                }
                else
                {
                    ImPlot::SetupAxisScale(ImAxis_::ImAxis_Y1, ImPlotScale_Linear);
                    ImPlot::SetupAxisLimits(ImAxis_::ImAxis_Y1, 1, 3000, ImPlotCond_::ImPlotCond_Once);
                }

                ImPlot::PlotLine("module 1", testX, testY, 600);

                ImPlot::EndPlot();
            }

            ImGui::SliderFloat("float", &f, 0.0f, 1.0f); // Edit 1 float using a slider from 0.0f to 1.0f

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

            ImGui::End();
        }
        {
            ImGui::Begin("Recon Image");

            // ImGui::SetWindowPos(ImVec2(1000, 0), ImGuiCond_Once);

            nfdchar_t *fileDir = nullptr;

            if (ImGui::Button("load"))
            {
                char cwd[PATH_MAX];
                if (getcwd(cwd, sizeof(cwd)) == nullptr)
                {
                    perror("getdwd() error");
                    return 1;
                }
                nfdresult_t result = NFD_PickFolder(cwd, &fileDir);
                if (result == NFD_OKAY)
                {
                    puts("Success!");
                    printf(fileDir);
                    sessionData = new HUREL::Compton::SessionData(fileDir);
                }
                else if (result = NFD_CANCEL)
                {
                    puts("Cancel");
                }
                else
                {
                    printf("Error: %s\n", NFD_GetError());
                }
            }

            if (fileDir != nullptr)
            {
                ImGui::Text("%s", fileDir);
            }
            if (ImGui::SliderFloat("s2M", &s2M, 0.0f, 5.0f))
            {
                isImageGet = false;
            }
            if (ImGui::SliderFloat("det_W", &det_W, 0.0f, 0.5f))
            {
                isImageGet = false;
            }
            if (ImGui::SliderFloat("resImprov", &resImprov, 0.0f, 50.0f))
            {
                isImageGet = false;
            }
            if (ImGui::SliderFloat("m2D", &m2D, 0.0f, 0.2f))
            {
                isImageGet = false;
            }
            if (ImGui::SliderFloat("hFov", &hFov, 0.1f, 179.0f))
            {
                isImageGet = false;
            }
            if (ImGui::SliderFloat("wFov", &wFov, 0.1f, 179.0f))
            {
                isImageGet = false;
            }

            if (sessionData != nullptr)
            {

                if (!isImageGet)
                {
                    lmData = sessionData->GetListedListModeDataCount(10000);
                    spdlog::info("get listed list mode data done");
                    radData = HUREL::Compton::RadiationImage(lmData, s2M, det_W, resImprov, m2D, hFov, wFov);
                    spdlog::info("image recon done");
                    isImageGet = true;

                    image = sessionData->mRgbImage;
                    glGenTextures(1, &texture);
                    glBindTexture(GL_TEXTURE_2D, texture);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, image.data);
                    cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_BGR2BGRA);

                    radImage = HUREL::Compton::RadiationImage::GetCV_32SAsJet(radData.mHybridImage, image.rows, image.cols, 0.8);

                    overlayImage(image, radImage, radImageOveray);
                    glGenTextures(1, &texture2);
                    glBindTexture(GL_TEXTURE_2D, texture2);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
                    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, radImageOveray.cols, radImageOveray.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, radImageOveray.data);
                }
                // ImGui::Image(reinterpret_cast<void *>(static_cast<intptr_t>(texture)), ImVec2(image.cols, image.rows));

                ImGui::Image(reinterpret_cast<void *>(static_cast<intptr_t>(texture2)), ImVec2(radImage.cols, radImage.rows));
            }
            ImGui::End();
        }

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
            visualizer.SetWindowPosition(windowXpos + open3dWindowPos.x, windowYpos + open3dWindowPos.y);
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
