#include "ImWindows.h"

void HUREL::GUI::Reconstruction3D(HUREL::MyVisualizer &vis)
{
}

void HUREL::GUI::EnergySpectrumWindow(bool initial, Compton::SessionData *&sessionDataOrNull)
{
    static std::vector<double> testX;
    static std::vector<double> testY;
    static bool isLogScale;

    ImGui::Begin("에너지 스펙트럼", nullptr, ImGuiWindowFlags_NoBringToFrontOnFocus); // Create a window called "Hello, world!" and append into it.



    if (sessionDataOrNull != nullptr)
    {
        std::vector<HUREL::Compton::BinningEnergy> spect = sessionDataOrNull->GetEnergySpectrum(nullptr, 0, 0).GetHistogramEnergy();
        testX.resize(spect.size());
        testY.resize(spect.size());

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

    if (ImPlot::BeginPlot("Energy Spectrum", ImGui::GetContentRegionAvail(), plotFlag))
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

        ImPlot::PlotLine("module 1", testX.data(), testY.data(), testX.size());

        ImPlot::EndPlot();
    }

    ImGui::End();
}

void overlayImage(const cv::Mat &background, const cv::Mat &foreground, cv::Mat &output)
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
            for (int c = 0; opacity > 0 && c < output.channels() - 1; ++c)
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

bool HUREL::GUI::Reconstrcution2D(bool initial, Compton::SessionData *&sessionDataOrNUll)
{
    static bool isImageGet;
    static bool isGrayScale;
    static float s2M;
    static float det_W;
    static float resImprov;
    static float m2D;
    static float hFov;
    static float wFov;
    static float minPortion;
    static float opacity;

    static GLuint texture;
    static GLuint texture2;
    static cv::Mat image;
    static cv::Mat radImage;
    static cv::Mat radImageOveray;
    static std::vector<HUREL::Compton::ListModeData> lmData;
    static HUREL::Compton::RadiationImage radData;

    if (initial)
    {
        isImageGet = false;
        isGrayScale = true;
        s2M = 3.5;
        det_W = 0.312;
        resImprov = 15;
        m2D = 0.068;
        hFov = 60;
        wFov = 90;

        texture = 0;
        texture2 = 0;

        minPortion = 0.9;
        opacity = 0.5;
    }

    Compton::SessionData *&sessionData = sessionDataOrNUll;
    ImGui::Begin("Recon Image", nullptr, ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    // ImGui::SetWindowPos(ImVec2(1000, 0), ImGuiCond_Once);

    nfdchar_t *fileDir = nullptr;
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
            if (isGrayScale)
            {
                cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_BGR2GRAY);
                cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_GRAY2BGRA);
            }
            else
            {
                cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_BGR2BGRA);
            }

            

            radImage = HUREL::Compton::RadiationImage::GetCV_32SAsJet(radData.mHybridImage, image.rows, image.cols, minPortion, opacity);

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

    if (ImGui::Button("load"))
    {
        char cwd[PATH_MAX];
        if (getcwd(cwd, sizeof(cwd)) == nullptr)
        {
            perror("getdwd() error");
            return false;
        }
        nfdresult_t result = NFD_PickFolder(".", &fileDir);
        if (result == NFD_OKAY)
        {
            puts("Success!");
            printf(fileDir);
            sessionData = new HUREL::Compton::SessionData(fileDir);
            if (sessionData->GetSizeListedListModeData() == 0)
            {
                delete sessionData;
                sessionData = nullptr;
            }
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
    if (ImGui::Checkbox("Gray scale", &isGrayScale))
    {
        isImageGet = false;
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
    if (ImGui::SliderFloat("minPortion", &minPortion, 0.0f, 1.0f))
    {
        isImageGet = false;
    }
     if (ImGui::SliderFloat("opacity", &opacity, 0.0f, 1.0f))
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

    ImGui::End();

    return true;
}
