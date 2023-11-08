#include "ImWindows.h"

void HUREL::GUI::ChangeSettingValues(std::string key, std::string value)
{
    // chekc if key exists
    if (mSettingValues.find(key) == mSettingValues.end())
    {
        spdlog::error("setting key not found");
        return;
    }

    // check if value changed
    if (mSettingValues[key] == value)
    {
        return;
    }

    mSettingValues[key] = value;
    // write settings to file
    std::ofstream settingFile("setting.txt");
    if (settingFile.is_open())
    {
        for (auto &setting : mSettingValues)
        {
            settingFile << setting.first << " " << setting.second << std::endl;
        }
    }
    else
    {
        spdlog::error("setting.txt file not found");
    }
}

void HUREL::GUI::Init()
{
    if (mIsSettingValuesInit)
    {
        return;
    }
    // read setting file
    std::ifstream settingFile("setting.txt");
    if (settingFile.is_open())
    {
        std::string line;
        while (std::getline(settingFile, line))
        {
            std::istringstream iss(line);
            std::string key;
            std::string value;
            if (!(iss >> key >> value))
            {
                break;
            }
            mSettingValues[key] = value;
        }
    }
    else
    {
        spdlog::error("setting.txt file not found");
        // set settings to default
        mSettingValues.emplace("isLogScale", "1");
        mSettingValues.emplace("ref_fwhm", "50");
        mSettingValues.emplace("fwhm_at_0", "0.5");
        mSettingValues.emplace("min_snr", "5");
        mSettingValues.emplace("effectiveCount", "1000");
        mSettingValues.emplace("isGreyScale", "1");
        mSettingValues.emplace("s2M", "3.5");
        mSettingValues.emplace("det_W", "0.312");
        mSettingValues.emplace("resImprov", "15");
        mSettingValues.emplace("minPortion", "0.9");
        mSettingValues.emplace("opacity", "0.5");
        mSettingValues.emplace("m2D", "0.068");
        mSettingValues.emplace("hFov", "60.0");
        mSettingValues.emplace("wFov", "90.0");

        // write settings to file
        std::ofstream settingFile("setting.txt");
        if (settingFile.is_open())
        {
            for (auto &setting : mSettingValues)
            {
                settingFile << setting.first << " " << setting.second << std::endl;
            }
        }
        else
        {
            spdlog::error("setting.txt file not found");
        }
    }
    mIsSettingValuesInit = true;
}

void HUREL::GUI::Reconstruction3D(HUREL::MyVisualizer &vis)
{
    Init();
}

void HUREL::GUI::EnergySpectrumWindow(bool initial, Compton::SessionData *&sessionDataOrNull)
{
    Init();

    static std::vector<double> testX;
    static std::vector<double> testY;
    static bool isLogScale;
    static bool isScatterChecked;
    static bool isAbsorberChecked;
    static int channelNumber;
    static std::future<std::tuple<std::vector<double>, std::vector<double>, std::vector<HUREL::IsotopeData>>> calcPeaksFuture;
    static bool isCalcPeaksFutureReady;
    static std::vector<double> peaks;
    static std::vector<double> snrData;
    static HUREL::Compton::EnergySpectrum spectrum;

    static std::future<HUREL::Compton::EnergySpectrum> getEnergySpectrumFuture;
    static bool isGetEnergySpectrumFutureReady;

    static double currentDoseRate;
    static bool isCalibrationCheked;
    static double timeToAquire;

    static double min_snr;
    static double ref_x;
    static double ref_fwhm;
    static double fwhm_at_0;
    static double k40Peak = -1;
    static unsigned mColorBarTexture;
    static unsigned mReverseTriangleTexture;
    static cv::Mat colorBar;
    static cv::Mat reverseTriangle;
    static double minDoseRate = 0.01;
    // log 0.1 = -1
    static double greenZone = log10(0.1) - log10(minDoseRate); // = -1
    static double yellowZone = log10(2) - log10(minDoseRate);  // = 0.698970004336
    static double orangeZone = log10(40) - log10(minDoseRate); // = 1.39794000867
    static double redZone = log10(100) - log10(minDoseRate);   // = 2

    static bool isAutoTrackIsotope;

    if (initial)
    {
        isCalibrationCheked = false;
        timeToAquire = 10;
        mColorBarTexture = 0;
        mReverseTriangleTexture = 0;
        isScatterChecked = true;
        isAbsorberChecked = true;
        channelNumber = -1;
        isCalcPeaksFutureReady = true;
        isGetEnergySpectrumFutureReady = true;
        isLogScale = std::stoi(mSettingValues["isLogScale"]);
        ref_fwhm = std::stod(mSettingValues["ref_fwhm"]);
        fwhm_at_0 = std::stod(mSettingValues["fwhm_at_0"]);
        min_snr = std::stod(mSettingValues["min_snr"]);
        isAutoTrackIsotope = mSettingValues["isAutoTrackIsotope"] == "1";
        currentDoseRate = 1;
        // reverse equilateral triangle
        reverseTriangle = cv::Mat(15, 15, CV_8UC4);
        for (int j = 0; j < reverseTriangle.rows; j++)
        {
            for (int i = 0; i < reverseTriangle.cols; i++)
            {
                if (i < reverseTriangle.cols / 2 + reverseTriangle.rows - j && i > reverseTriangle.cols / 2 - (reverseTriangle.rows - j))
                {
                    reverseTriangle.at<cv::Vec4b>(j, i) = cv::Vec4b(0, 0, 0, 255);
                }
                else
                {
                    reverseTriangle.at<cv::Vec4b>(j, i) = cv::Vec4b(0, 0, 0, 0);
                }
            }
        }
    }

    ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
    ImGui::Begin("에너지스펙트럼", nullptr, ImGuiWindowFlags_NoBringToFrontOnFocus); // Create a window called "Hello, world!" and append into it.
    ImGui::Checkbox("전면부 검출기", &isScatterChecked);
    ImGui::SameLine();
    ImGui::Checkbox("후면부 검출기", &isAbsorberChecked);


    // input combo box
    if (!isScatterChecked && !isAbsorberChecked)
    {
        ImGui::Combo("채널", &channelNumber, "0\0"
                                             "1\0"
                                             "2\0"
                                             "3\0"
                                             "4\0"
                                             "5\0"
                                             "6\0"
                                             "7\0"
                                             "8\0"
                                             "9\0"
                                             "10\0"
                                             "11\0"
                                             "12\0"
                                             "13\0"
                                             "14\0"
                                             "15\0"
                                             "-1\0");
    }

    if (sessionDataOrNull != nullptr)
    {
        if (isGetEnergySpectrumFutureReady)
        {
            isGetEnergySpectrumFutureReady = false;
            getEnergySpectrumFuture = std::async(std::launch::async, [&]() -> HUREL::Compton::EnergySpectrum
                                                 {
                HUREL::Compton::EnergySpectrum totalSpectrum;
                HUREL::Compton::EnergySpectrum spectrum;
                


                int channelNumbers[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
                totalSpectrum = sessionDataOrNull->GetEnergySpectrum(channelNumbers, 16, timeToAquire * 1000);
                if (isScatterChecked & isAbsorberChecked)
                {
                    spectrum = totalSpectrum;
                }
                else if (isAbsorberChecked && !isScatterChecked)
                {
                    spectrum = sessionDataOrNull->GetAbsroberEnergySpectrum(timeToAquire * 1000);
                }
                else if (isScatterChecked && !isAbsorberChecked)
                {
                    spectrum = sessionDataOrNull->GetScatterEnergySpectrum(timeToAquire * 1000);
                }
                else
                {
                    if (channelNumber != -1)
                    {
                        int channelNumbers[1];
                        channelNumbers[0] = channelNumber;
                        spectrum = sessionDataOrNull->GetEnergySpectrum(channelNumbers, 1, timeToAquire * 1000);
                    }
                }
                currentDoseRate = HUREL::Compton::EnergySpectrum::GetDoseRate(spectrum, timeToAquire);
                sleep(1);
                return spectrum; });
        }
        if (getEnergySpectrumFuture.valid())
        {
            if (getEnergySpectrumFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                spectrum = getEnergySpectrumFuture.get();
                isGetEnergySpectrumFutureReady = true;
            }
        }

        std::vector<HUREL::Compton::BinningEnergy> spect = spectrum.GetHistogramEnergy();
        testX.resize(spect.size());
        testY.resize(spect.size());
        for (int i = 0; i < spect.size(); ++i)
        {
            testX[i] = spect.at(i).Energy;
            testY[i] = spect.at(i).Count;
        }

        if (calcPeaksFuture.valid())
        {
            if (calcPeaksFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                auto values = calcPeaksFuture.get();
                peaks = std::get<0>(values);
                snrData = std::get<1>(values);
                mIsotopeDataList = std::get<2>(values);

                if (mSelectedIsotopeData.GetIsotopeName() == "none")
                {
                }

                isCalcPeaksFutureReady = true;
            }
        }
        if (isCalcPeaksFutureReady)
        {
            isCalcPeaksFutureReady = false;

            calcPeaksFuture = std::async(std::launch::async, [=]() mutable -> std::tuple<std::vector<double>, std::vector<double>, std::vector<HUREL::IsotopeData>>
                                         {
                HUREL::Compton::EnergySpectrum& temp = spectrum;
                std::vector<double> snrPtr;
                std::vector<double> peaksTemp;
                //peaksTemp = HUREL::Compton::EnergySpectrum::GetSpectrumPeaks(temp, &snrPtr, 662, ref_fwhm, fwhm_at_0, min_snr);
                            peaksTemp = HUREL::Compton::EnergySpectrum::GetSpectrumPeaks(temp, &snrPtr);

                
                std::vector<HUREL::IsotopeData> tempIsotopeData = HUREL::IsotopeAnalysis::FindIsotope(peaks);
                bool flagIsIsotopeDataChanged = false;
                if (mIsotopeDataList.size() != tempIsotopeData.size())
                {
                    flagIsIsotopeDataChanged = true;
                }
                else
                {
                    for (int i = 0; i < mIsotopeDataList.size(); ++i)
                    {
                        if (!(mIsotopeDataList[i] == tempIsotopeData[i]))
                        {
                            flagIsIsotopeDataChanged = true;
                            break;
                        }
                    }
                }
                if (flagIsIsotopeDataChanged)
                {
                    mIsotopeDataList = tempIsotopeData;
                    std::vector<HUREL::Compton::sEnergyCheck> energyCheckData;

                    for (auto &isotope : mIsotopeDataList)
                    {
                        auto tempEchk = isotope.GetEnergyCheck();
                        energyCheckData.insert(energyCheckData.end(), tempEchk.begin(), tempEchk.end());
                    }
                    // Energy Echks 설정 부분.
                    HUREL::Compton::LahgiControl::instance().SetEchk(energyCheckData);
                }

                sleep(1);
                return std::make_tuple(peaksTemp, snrPtr, tempIsotopeData); });
        }
    }

    if (ImGui::Checkbox("로그 스케일", &isLogScale))
    {
        ChangeSettingValues("isLogScale", isLogScale ? "1" : "0");
    }

    ImPlot::PushStyleColor(0, ImVec4(0, 0, 0, 1));
    ImPlot::PushStyleColor(0, ImVec4(0, 0, 0, 1));
    ImPlotFlags plotFlag = ImPlotFlags_None | ImPlotFlags_Crosshairs;
    double xSize = ImGui::GetContentRegionAvail()[0];
    double ySize = ImGui::GetContentRegionAvail()[1];
    if (xSize > ySize)
    {
        xSize = ySize;
    }
    else
    {
        ySize = xSize;
    }
    ImVec2 plotSize = ImVec2(xSize - 20, ySize - 20);
    if (ImPlot::BeginPlot("에너지 스펙트럼", plotSize, plotFlag))
    {

        ImPlot::SetupAxes("에너지 [keV]", "계수치 [#]",
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

        // change legend position
        ImPlot::SetupLegend(ImPlotLocation_::ImPlotLocation_NorthEast, ImPlotLegendFlags_None);

        ImPlot::PlotLine("에너지 스펙트럼", testX.data(), testY.data(), testX.size());

        // plot peaks inifinite line on the graph
        // set color as red
        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0, 0, 1, 1));
        
        ImPlot::HideNextItem(true);
        ImPlot::PlotInfLines("피크",peaks.data(), peaks.size(), 0);
        ImPlot::PopStyleColor();


        std::vector<double> selectedPeak;

        std::vector<double> seleectIsotopeEnergy = mSelectedIsotopeData.GetEnergyPeaks();
        selectedPeak.insert(selectedPeak.end(), seleectIsotopeEnergy.begin(), seleectIsotopeEnergy.end());
        ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1, 0, 0, 1));

        ImPlot::PlotInfLines("선택된 핵종 피크", selectedPeak.data(), selectedPeak.size(), 0);
        ImPlot::PopStyleColor();

        // draw infLines with
        if (k40Peak > 0)
        {
            //color purple
            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(0.5, 0, 0.5, 1));
            double values[1];
            values[0] = k40Peak;
            ImPlot::PlotInfLines("K40", values, 1);
            ImPlot::PopStyleColor();
        }

        // plot snr as orange
        if (snrData.size() == testX.size())
        {
            // hide snr
            ImPlot::HideNextItem(true);
            ImPlot::PushStyleColor(ImPlotCol_Line, ImVec4(1, 0.5, 0, 1));
            ImPlot::PlotLine("SNR", testX.data(), snrData.data(), testX.size());
            // not show snr as default

            ImPlot::PopStyleColor();
        }
        // set text as bold
        // ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 2);
        ImPlotPoint mousePos = ImPlotPoint(-1, -1);
        ImPlotPoint mouseHoverPose = ImPlotPoint(-1, -1);
        // get click position
        if (ImPlot::IsPlotHovered())
        {
            mouseHoverPose = ImPlot::GetPlotMousePos();
            if (ImGui::IsMouseClicked(0))
            {
                mousePos = ImPlot::GetPlotMousePos();
            }
        }
        for (int i = 0; i < mIsotopeDataList.size(); ++i)
        {
            std::string isotopeName = mIsotopeDataList.at(i).GetIsotopeName();
            if (mIsotopeDataList.at(i) == mSelectedIsotopeData)
            {
                std::vector<double> isotopeEnergy = mIsotopeDataList.at(i).GetEnergyPeaks();
                for (int j = 0; j < isotopeEnergy.size(); ++j)
                {
                    int xIndexEnergy = std::upper_bound(testX.begin(), testX.end(), isotopeEnergy.at(j)) - testX.begin();
                    bool isSelected = false;
                    int drawPosX = isotopeEnergy.at(j) + 100;
                    int drawPosY = testY.at(xIndexEnergy) * 1.2;

                    if (mousePos.x > 0 && mousePos.y > 0)
                    {
                        if (mousePos.x > drawPosX * 0.9 && mousePos.x < drawPosX * 1.1)
                        {
                            if (mousePos.y > drawPosY * 0.9 && mousePos.y < drawPosY * 1.1)
                            {
                                isSelected = false;
                                spdlog::info("Isotope Name : {}", isotopeName);
                                isAutoTrackIsotope = false;
                                mSelectedIsotopeData = HUREL::IsotopeData();
                            }
                        }
                    }

                    // set text color as red
                    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 0, 0, 1.0));
                    ImPlot::PlotText(isotopeName.c_str(), drawPosX, drawPosY);
                    ImGui::PopStyleColor();
                }
            }
            else
            {
                std::vector<double> isotopeEnergy = mIsotopeDataList.at(i).GetEnergyPeaks();

                for (int j = 0; j < isotopeEnergy.size(); ++j)
                {
                    //std::cout << "checked isotope energy: " << isotopeEnergy.at << std::endl;
                    int xIndexEnergy = std::upper_bound(testX.begin(), testX.end(), isotopeEnergy.at(j)) - testX.begin();
                    bool isSelected = false;
                    int drawPosX = isotopeEnergy.at(j) + 100;
                    int drawPosY = testY.at(xIndexEnergy) * 1.2;
                    if (mousePos.x > 0 && mousePos.y > 0)
                    {
                        if (mousePos.x > drawPosX * 0.9 && mousePos.x < drawPosX * 1.1)
                        {
                            if (mousePos.y > drawPosY * 0.9 && mousePos.y < drawPosY * 1.1)
                            {
                                isSelected = true;
                                spdlog::info("Isotope Name : {}", isotopeName);
                                isAutoTrackIsotope = false;
                                mSelectedIsotopeData = mIsotopeDataList.at(i);
                            }
                        }
                    }

                    if (mouseHoverPose.x > 0 && mouseHoverPose.y > 0)
                    {
                        if (mouseHoverPose.x > drawPosX * 0.9 && mouseHoverPose.x < drawPosX * 1.1)
                        {
                            if (mouseHoverPose.y > drawPosY * 0.9 && mouseHoverPose.y < drawPosY * 1.1)
                            {
                                // set font color as grey
                                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.5, 0.5, 0.5, 1.0));
                            }
                        }
                    }

                    ImPlot::PlotText(isotopeName.c_str(), drawPosX, drawPosY);

                    if (mouseHoverPose.x > 0 && mouseHoverPose.y > 0)
                    {
                        if (mouseHoverPose.x > drawPosX * 0.9 && mouseHoverPose.x < drawPosX * 1.1)
                        {
                            if (mouseHoverPose.y > drawPosY * 0.9 && mouseHoverPose.y < drawPosY * 1.1)
                            {
                                ImGui::PopStyleColor();
                            }
                        }
                    }
                }
            }

            
        }

        if (isAutoTrackIsotope)
        {
            int priority = 99999999;
            for (auto iso : mIsotopeDataList)
            {
                if (iso.GetPriority() >= 0 && iso.GetPriority() < priority)
                {
                    priority = iso.GetPriority();
                    mSelectedIsotopeData = iso;
                }
            }
        }
        // ImPlot::PopStyleVar();
        // get texture id for cv::Mat colorBar

        ImPlot::EndPlot();
    }

    // show Isotope data as table
    if (ImGui::BeginTable("핵종 분석 결과", 3))
    {
        static int xRegionAvail = -1;
        if (xRegionAvail != ImGui::GetContentRegionAvail()[0])
        {
            xRegionAvail = ImGui::GetContentRegionAvail()[0];
            colorBar = cv::Mat(20, ImGui::GetContentRegionAvail()[0], CV_8UC3);
            int blackLineWidht = 2;
            for (int j = 0; j < colorBar.rows; j++)
            {
                for (int i = 0; i < colorBar.cols; i++)
                {

                    if (i < colorBar.cols * greenZone / redZone)
                    {
                        colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 255, 0);
                    }
                    else if (i < colorBar.cols * yellowZone / redZone)
                    {
                        colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(255, 255, 0);
                    }
                    else if (i < colorBar.cols * orangeZone / redZone)
                    {
                        colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(255, 165, 0);
                    }
                    else if (i < colorBar.cols * redZone / redZone)
                    {
                        colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(255, 0, 0);
                    }
                    else
                    {
                        colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
                    }
                    if (i > colorBar.cols * (log10(0.01) - log10(minDoseRate)) / redZone - blackLineWidht && i < colorBar.cols * (log10(0.01) - log10(minDoseRate)) / redZone + blackLineWidht)
                    {
                        colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
                    }
                    else if (i > colorBar.cols * (log10(0.1) - log10(minDoseRate)) / redZone - blackLineWidht && i < colorBar.cols * (log10(0.1) - log10(minDoseRate)) / redZone + blackLineWidht)
                    {
                        colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
                    }
                    else if (i > colorBar.cols * (log10(1) - log10(minDoseRate)) / redZone - blackLineWidht && i < colorBar.cols * (log10(1) - log10(minDoseRate)) / redZone + blackLineWidht)
                    {
                        colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
                    }
                    else if (i > colorBar.cols * (log10(10) - log10(minDoseRate)) / redZone - blackLineWidht && i < colorBar.cols * (log10(10) - log10(minDoseRate)) / redZone + blackLineWidht)
                    {
                        colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
                    }
                    else if (i > colorBar.cols * (log10(100) - log10(minDoseRate)) / redZone - blackLineWidht * 2 && i < colorBar.cols * (log10(100) - log10(minDoseRate)) / redZone)
                    {
                        colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
                    }
                    for (int drawIndex = 2; drawIndex <= 9; ++drawIndex)
                    {
                        int blackLineWidht2 = 1;
                        if (i > colorBar.cols * (log10(0.01 * drawIndex) - log10(minDoseRate)) / redZone - blackLineWidht2 && i < colorBar.cols * (log10(0.01 * drawIndex) - log10(minDoseRate)) / redZone)
                        {
                            colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
                        }
                        else if (i > colorBar.cols * (log10(0.1 * drawIndex) - log10(minDoseRate)) / redZone - blackLineWidht2 && i < colorBar.cols * (log10(0.1 * drawIndex) - log10(minDoseRate)) / redZone)
                        {
                            colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
                            break;
                        }
                        else if (i > colorBar.cols * (log10(1 * drawIndex) - log10(minDoseRate)) / redZone - blackLineWidht2 && i < colorBar.cols * (log10(1 * drawIndex) - log10(minDoseRate)) / redZone)
                        {
                            colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
                            break;
                        }
                        else if (i > colorBar.cols * (log10(10 * drawIndex) - log10(minDoseRate)) / redZone - blackLineWidht2 && i < colorBar.cols * (log10(10 * drawIndex) - log10(minDoseRate)) / redZone)
                        {
                            colorBar.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 0);
                            break;
                        }
                    }
                }
            }
        }

        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::Text("핵종 이름");
        ImGui::TableSetColumnIndex(1);
        ImGui::Text("에너지 [keV]");
                ImGui::TableSetColumnIndex(2);
        ImGui::Text("설명");
        for (int i = 0; i < mIsotopeDataList.size(); ++i)
        {
            if (mIsotopeDataList.at(i) == mSelectedIsotopeData)
            {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 0, 0, 1.0));
            }
            else
            {
                ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0, 0, 0, 1.0));
            }
            std::string isotopeName = mIsotopeDataList.at(i).GetIsotopeName();
            std::vector<double> isotopeEnergy = mIsotopeDataList.at(i).GetEnergyPeaks();
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text(isotopeName.c_str());
            ImGui::TableSetColumnIndex(1);

            for (int j = 0; j < isotopeEnergy.size(); ++j)
            {
                //add double precision 2
                ImGui::Text("%.2f", isotopeEnergy.at(j));
            }
            ImGui::TableSetColumnIndex(2);
            ImGui::Text(mIsotopeDataList.at(i).GetDescription().c_str());
            ImGui::PopStyleColor();
        }
        ImGui::EndTable();
    }

    ImGui::InputDouble("누적 시간", &timeToAquire, 1, 10, "%.0f");
    if (ImGui::Checkbox("핵종 자동 추적", &isAutoTrackIsotope))
    {
        ChangeSettingValues("isAutoTrackIsotope", std::to_string(isAutoTrackIsotope));
    }
    // color bar for get dose rate

    if (mColorBarTexture == 0)
    {
        glGenTextures(1, &mColorBarTexture);
        glBindTexture(GL_TEXTURE_2D, mColorBarTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    glBindTexture(GL_TEXTURE_2D, mColorBarTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, colorBar.cols, colorBar.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, colorBar.data);

    if (mReverseTriangleTexture == 0)
    {
        glGenTextures(1, &mReverseTriangleTexture);
        glBindTexture(GL_TEXTURE_2D, mReverseTriangleTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    glBindTexture(GL_TEXTURE_2D, mReverseTriangleTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, reverseTriangle.cols, reverseTriangle.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, reverseTriangle.data);
    
    // draw line
    ImGui::Separator();

    // make new widnow
    if (ImGui::Begin("선량률"))
    {
        // micor mm --> to greek letter: \u03BC
        // micro symbol: \u00B5
        ImGui::Text("선량률 [uSv/h]");
        ImGui::SameLine();

        // round froun 0.01
        std::stringstream stream;
        stream << std::fixed << std::setprecision(5) << currentDoseRate;
        ImGui::Text(stream.str().c_str());

        double arrowDoseRate = currentDoseRate;
        if (arrowDoseRate < minDoseRate)
        {
            arrowDoseRate = minDoseRate;
        }
        int x = ImGui::GetCursorPosX();
        int y = ImGui::GetCursorPosY();
        ImGui::SetCursorPosX(x - reverseTriangle.cols / 2 + (log10(arrowDoseRate) - log10(minDoseRate)) / 4 * colorBar.cols);
        ImGui::SetCursorPosY(y);
        ImGui::Image((void *)mReverseTriangleTexture, ImVec2(reverseTriangle.cols, reverseTriangle.rows));
        ImGui::SetCursorPosY(y + reverseTriangle.rows);
        ImGui::Image((void *)mColorBarTexture, ImVec2(colorBar.cols, colorBar.rows));
        // add text at color bar
        x = ImGui::GetCursorPosX();
        y = ImGui::GetCursorPosY();
        ImGui::SetCursorPosX(x);
        ImGui::SetCursorPosY(y);
        ImGui::Text("0.01");

        ImGui::SetCursorPosX(x + 1.0 / 4 * colorBar.cols - 5);
        ImGui::SetCursorPosY(y);
        ImGui::Text("0.1");

        ImGui::SetCursorPosX(x + 2.0 / 4 * colorBar.cols);
        ImGui::SetCursorPosY(y);
        ImGui::Text("1");
        ImGui::SetCursorPosX(x + 3.0 / 4 * colorBar.cols - 5);
        ImGui::SetCursorPosY(y);
        ImGui::Text("10");

        ImGui::SetCursorPosX(x + 4.0 / 4 * colorBar.cols - 30);
        ImGui::SetCursorPosY(y);
        ImGui::Text("100");

        ImGui::End();
    }

    ImGui::Checkbox("교정", &isCalibrationCheked);
    if (isCalibrationCheked && !isScatterChecked && !isAbsorberChecked)
    {

        if (k40Peak < 0 && ImGui::Button("재교정 (Find K40)"))
        {
            if (peaks.size() == 0)
            {
                spdlog::error("재교정할 에너지가 없습니다.");
            }
            else
            {
                // K40 Energy
                double refEnergy = 1460.82;

                for (auto &peak : peaks)
                {
                    if (k40Peak < 0)
                    {
                        if (peak > refEnergy - 300 & peak < refEnergy + 300)
                        {
                            k40Peak = peak;
                        }
                    }
                    else
                    {
                        if (abs(peak - k40Peak) < abs(k40Peak - refEnergy))
                        {
                            k40Peak = peak;
                        }
                    }
                }
            }
        }
        if (k40Peak > 0)
        {
            //ImGui::Button("재교정");
            if (ImGui::Button("재교정"))
            {
                auto ecalValues = HUREL::Compton::LahgiControl::instance().GetEcalValue(channelNumber);

                double diffPortion =  1460.82 / k40Peak;
                // ax^2 + bx + c = energy
                double a = std::get<0>(ecalValues);
                double b = std::get<1>(ecalValues);
                double c = std::get<2>(ecalValues);

                double newA = a * diffPortion * diffPortion;
                double newB = b * diffPortion;
                double newC = c;
                
                HUREL::Compton::LahgiControl::instance().SetEcalValue(channelNumber, std::make_tuple(newA, newB, newC));
                k40Peak = -1;
            }

            ImGui::SameLine();

            if (ImGui::Button("교정 값 리셋"))
            {
                HUREL::Compton::LahgiControl::instance().SetEcalValue(channelNumber, std::make_tuple(0, 1, 0));
                k40Peak = -1;
            }

            ImGui::Text("K40 피크 위치 변경");
            ImGui::SameLine();
            if (ImGui::ArrowButton("##Left", ImGuiDir_Left))
            {
                for (int i = 0; i  < peaks.size(); i++)
                {
                    if (peaks[i] == k40Peak)
                    {
                        if (i > 0)
                        {
                            k40Peak = peaks[i - 1];
                        }
                        break;
                    }
                }
            }
            ImGui::SameLine();
            if (ImGui::ArrowButton("##Right", ImGuiDir_Right))
            {
                for (int i = 0; i  < peaks.size(); i++)
                {
                    if (peaks[i] == k40Peak)
                    {
                        if (i < peaks.size() - 1)
                        {
                            k40Peak = peaks[i + 1];
                        }
                        break;
                    }
                }
            }
        }
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
    Init();
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
    static cv::Point2d radImageMaxLoc;

    static int effectiveCount;
    static int trueCount = 0;

    static bool recon2dFutreReady;
    static std::future<cv::Mat> recon2dFuture;
    static bool recon2dFutreReadyRealtime;
    static std::future<cv::Mat> recon2dFutureRealtime;
    static int reconType;
    static IsotopeData tempIsotopeData;
    static bool isAutoDistance;

    if (initial)
    {
        radImageMaxLoc = cv::Point2d(-1, -1);
        isImageGet = false;
        isGrayScale = true;
        effectiveCount = stoi(mSettingValues["effectiveCount"]);
        s2M = stof(mSettingValues["s2M"]);
        det_W = stof(mSettingValues["det_W"]);
        resImprov = stof(mSettingValues["resImprov"]);
        m2D = stof(mSettingValues["m2D"]);
        hFov = stof(mSettingValues["hFov"]);
        wFov = stof(mSettingValues["wFov"]);
        minPortion = stof(mSettingValues["minPortion"]);
        opacity = stof(mSettingValues["opacity"]);
        isAutoDistance = mSettingValues["isAutoDistance"] == "1" ? true : false;
        recon2dFutreReadyRealtime = true;
        recon2dFutreReady = true;
        glGenTextures(1, &texture2);
        tempIsotopeData = tempIsotopeData = mSelectedIsotopeData;
        reconType = 1;
    }

    if (tempIsotopeData.GetIsotopeName() != mSelectedIsotopeData.GetIsotopeName())
    {
        isImageGet = false;
    }
    tempIsotopeData = mSelectedIsotopeData;

    Compton::SessionData *&sessionData = sessionDataOrNUll;
    ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);

    ImGui::Begin("영상화 창", nullptr, ImGuiWindowFlags_NoBringToFrontOnFocus);

    // progressBar /  effectiveCount
    {
        ImGui::Text("진행도");
        ImGui::SameLine();
        //set progress bar color
        float progress = (float)trueCount / (float)effectiveCount;
        if (progress < 0.3)
        {
            //set color red
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
        }
        else if (progress < 0.7)
        {
            //set color yellow
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(1.0f, 1.0f, 0.0f, 1.0f));
        }
        else
        {
            //set color green
            ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImVec4(0.0f, 1.0f, 0.0f, 1.0f));
        }
        
        ImGui::ProgressBar((float)trueCount / (float)effectiveCount);
        ImGui::PopStyleColor();
    }
    if (mIsRealtimeRun)
    {
        if (recon2dFutreReadyRealtime)
        {
            recon2dFutreReadyRealtime = false;
            recon2dFutureRealtime = std::async(std::launch::async, [&]() -> cv::Mat
                                               {
                image = HUREL::Compton::RtabmapSlamControl::instance().GetCurrentVideoFrame();
                
                
                if (!image.empty())
                {
                    if (isGrayScale)
                    {
                        cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_BGR2GRAY);
                        cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_GRAY2BGRA);
                    }
                    else
                    {
                        cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_BGR2BGRA);
                    }
                }
                cv::Mat tempRadImage = radImage.clone();
                cv::Mat resultOverlay;
                overlayImage(image, tempRadImage, radImageOveray);

                if (isAutoDistance)
                {
                    cv::Mat depth = HUREL::Compton::RtabmapSlamControl::instance().GetCurrentDepthFrame();

                    // cv::Mat depthColor = depth.clone();
                    // cv::normalize(depthColor, depthColor, 0, 256, cv::NORM_MINMAX, CV_8UC1);
                    // cv::applyColorMap(depthColor, depthColor, cv::COLORMAP_JET);
                    // cv::cvtColor(depthColor, depthColor, cv::ColorConversionCodes::COLOR_BGR2BGRA);
                    // //alpha blending
                    // cv::addWeighted(depthColor, opacity, radImageOveray, 1 - opacity, 0, radImageOveray);

                    if (radImageMaxLoc.x == -1)
                    {
                        double avgDepth = 0;
                        int count = 0;
                        for (int i = depth.rows * 0.45; i < depth.rows * 0.55; i++)
                        {
                            for (int j = depth.cols * 0.45; j < depth.cols * 0.55; j++)
                            {
                                if (depth.at<float>(i, j) > 0)
                                {
                                    // check isnan
                                    if (isnan(depth.at<float>(i, j)))
                                    {
                                        continue;
                                    }
                                    avgDepth += depth.at<float>(i, j);
                                    count++;
                                }
                            }
                        }
                        if (count == 0)
                        {
                            s2M = s2M;
                        }
                        else
                        {
                            s2M = avgDepth / count;
                        }
                    }
                    else if (radImageMaxLoc.x != 0 || radImageMaxLoc.y != 0)
                    {

                        double avgDepth = 0;
                        int count = 0;
                        int startRow = 0;
                        int rowSize = 0.05  * depth.rows;
                        int colSize = 0.05  * depth.cols;
                        radImageMaxLoc.x = (double)radImageMaxLoc.x * (double)depth.cols;
                        radImageMaxLoc.y = (double)radImageMaxLoc.y * (double)depth.rows;
                        if (radImageMaxLoc.y - rowSize > 0 && radImageMaxLoc.y - rowSize < depth.rows)
                        {
                            startRow = radImageMaxLoc.y - rowSize;
                        }
                        int endRow = depth.rows;
                        if (radImageMaxLoc.y + rowSize < depth.rows && radImageMaxLoc.y + rowSize > 0)
                        {
                            endRow = radImageMaxLoc.y + rowSize;
                        }

                        int startCol = 0;
                        if (radImageMaxLoc.x - colSize > 0 && radImageMaxLoc.x - colSize < depth.cols)
                        {
                            startCol = radImageMaxLoc.x - colSize;
                        }
                        int endCol = depth.cols;
                        if (radImageMaxLoc.x + colSize < depth.cols && radImageMaxLoc.x + colSize > 0)
                        {
                            endCol = radImageMaxLoc.x + colSize;
                        }
                     
                        for (int i = startRow; i < endRow; i++)
                        {
                            for (int j = startCol; j <  endCol; j++)
                            {
                                //check isnan
                                if (isnan(depth.at<float>(i, j)))
                                {
                                    continue;
                                }
                                
                                if (depth.at<float>(i, j) > 0)
                                {
                                    avgDepth += depth.at<float>(i, j);
                                    count++;
                                }
                            }
                        }
                        if (count == 0 || startRow == 0 || startCol == 0 || endRow == depth.rows || endCol == depth.cols)
                        {
                            s2M = s2M;
                        }
                        else
                        {
                            s2M = avgDepth / count;
                            cv::rectangle(radImageOveray, cv::Point(radImageMaxLoc.x - 10, radImageMaxLoc.y - 10), cv::Point(radImageMaxLoc.x + 10, radImageMaxLoc.y + 10), cv::Scalar(0, 0, 255, 255), 2, 8, 0);
                        }
                        
                        //radImageMaxLoc.x = (double)radImageMaxLoc.x / (double)depth.cols * (double)radImageOveray.cols;
                        //radImageMaxLoc.y = (double)radImageMaxLoc.y / (double)depth.rows * (double)radImageOveray.rows;

                        //add radImageOveray a red rectangle at the max location
                    }
                }
                return radImageOveray; });
        }
        if (recon2dFutureRealtime.valid())
        {
            if (recon2dFutureRealtime.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
            {
                radImageOveray = recon2dFutureRealtime.get();
                recon2dFutreReadyRealtime = true;

                glBindTexture(GL_TEXTURE_2D, texture2);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, radImageOveray.cols, radImageOveray.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, radImageOveray.data);
            }
        }

        if (sessionData != nullptr && recon2dFutreReady)
        {
            recon2dFutreReady = false;
            //spdlog::info("Se;ected Isotope Name : {}", mSelectedIsotopeData.GetIsotopeName());
            recon2dFuture = std::async(std::launch::async, [&]() -> cv::Mat
            {
                auto interData = sessionData->GetInteractionData(mSelectedIsotopeData, effectiveCount);
                
                trueCount = 0;
                if (interData.size() == 0)
                {
                    //spdlog::info("no interaction data");
                    return cv::Mat();
                }
                for (int i = 0; i < interData.size(); i++)
                {
                    trueCount += interData[i].interactionCount;
                }
                
                open3d::geometry::PointCloud pcd = sessionData->GetPointCloud();
                auto transmatrix = sessionData->GetTransMatrix();
                //std::cout << "Get pointcloud!!! " << pcd.points_.size()<< std::endl; 
                if (pcd.points_.size() == 0)
                {
                    spdlog::info("no point cloud data");
                    std::cout << "Empty Get pointcloud!!!" << std::endl; 
                }
				
                double maxValue = 0; double maxLocx = 0; double maxLocy = 0; double maxLocz = 0;
                //auto radData = HUREL::Compton::RadiationImage(interData, s2M, resImprov, pcd, &maxValue, &maxLocx, &maxLocy, &maxLocz); //pcl에 compton 정지 영상화 구현 완료, 최대값저장은 미완
                //auto radData = HUREL::Compton::RadiationImage(interData, s2M, resImprov, pcd, transmatrix, &maxValue, &maxLocx, &maxLocy, &maxLocz); //pcl에 compton 움직이면서 영상화 완료, 최대값저장은 미완
                //auto radData = HUREL::Compton::RadiationImage(interData, s2M, resImprov, 0.073, 60, 90);  //직교 좌표계에 영상화 hybrid까지 구현 완료
                auto radData = HUREL::Compton::RadiationImage(interData, s2M, resImprov, hFov, wFov);  //직교 좌표계에 영상화 hybrid까지 구현 완료

                //spdlog::info("image recon done");
                cv::Point2i tempPoint;
                if (reconType == 0)
                {
                    //get max pixel radData.mHybridImage
                    double maxVal = 0;
                    cv::minMaxLoc(radData.mHybridImage, NULL, &maxVal, NULL, &tempPoint);
                    radImageMaxLoc.x = (double)tempPoint.x/ (double)radData.mHybridImage.cols;
                    radImageMaxLoc.y = (double)tempPoint.y/ (double)radData.mHybridImage.rows;
                    return HUREL::Compton::RadiationImage::GetCV_32SAsJet(radData.mHybridImage, image.rows, image.cols, minPortion, opacity);
                }
                else if (reconType == 1)
                {
                    double maxVal = 0;
                    cv::minMaxLoc(radData.mComptonImage, NULL, &maxVal, NULL, &tempPoint);

                    radImageMaxLoc.x = (double)tempPoint.x/ (double)radData.mComptonImage.cols;
                    radImageMaxLoc.y = (double)tempPoint.y/ (double)radData.mComptonImage.rows;
                    return HUREL::Compton::RadiationImage::GetCV_32SAsJet(radData.mComptonImage, image.rows, image.cols, minPortion, opacity);

                }
                else if (reconType == 2)
                {
                    double maxVal = 0;
                    cv::minMaxLoc(radData.mCodedImage, NULL, &maxVal, NULL, &tempPoint);
                    radImageMaxLoc.x = (double)tempPoint.x/ (double)radData.mComptonImage.cols;
                    radImageMaxLoc.y = (double)tempPoint.y/ (double)radData.mComptonImage.rows;
                    return HUREL::Compton::RadiationImage::GetCV_32SAsJet(radData.mCodedImage, image.rows, image.cols, minPortion, opacity);

                }
                else
                {
                    return cv::Mat();
                } 

                //핵종 이름, maximum location이랑, value를 push back
                //HUREL::Compton::RtabmapSlamControl::instance().PushBackIsotopeData(mSelectedIsotopeData.GetIsotopeName(), maxLocx, maxLocy, maxLocz, maxValue);
            }

            );
        }

        if (recon2dFuture.valid())
        {
            // ImGui::Text("Reconstrcution 2D...........");
            if (recon2dFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                radImage = recon2dFuture.get();
                isImageGet = true;
                recon2dFutreReady = true;
            }
        }
        else
        {
            // ImGui::Text("Done Reconstrcution 2D");
        }
    }
    else
    {
        if (sessionData != nullptr && !isImageGet && recon2dFutreReady)
        {
            recon2dFutreReady = false;

            recon2dFuture = std::async(std::launch::async, [&]() -> cv::Mat
                                       {
                //lmData = sessionData->GetListedListModeData(effectiveCount);
                auto interData = sessionData->GetInteractionData(mSelectedIsotopeData, effectiveCount);
                spdlog::info("get listed list mode data done");
               
                trueCount = 0;
                //radData = HUREL::Compton::RadiationImage(lmData, s2M, det_W, resImprov, m2D, hFov, wFov);
                if (interData.size() == 0)
                {
                    //spdlog::info("no interaction data");
                    return cv::Mat();
                }
                for (int i = 0; i < interData.size(); i++)
                {
                    trueCount += interData[i].interactionCount;
                }
                //radData = HUREL::Compton::RadiationImage(interData, s2M, resImprov, m2D, hFov, wFov);
                radData = HUREL::Compton::RadiationImage(interData, s2M, resImprov, 0.073, 60, 90);
                //radData = HUREL::Compton::RadiationImage(interData, s2M, resImprov, pcd, transmatrix, &maxValue, &maxLocx, &maxLocy, &maxLocz); //pcl에 compton 움직이면서 영상화 완료, 최대값저장은 미완
                //spdlog::info("image recon done");
                
                image = sessionData->mRgbImage;
                
                if (!image.empty())
                {
                    if (isGrayScale)
                    {
                        cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_BGR2GRAY);
                        cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_GRAY2BGRA);
                    }
                    else
                    {
                        cv::cvtColor(image, image, cv::ColorConversionCodes::COLOR_BGR2BGRA);
                    }
                    if (reconType == 0)
                    {
                        radImage = HUREL::Compton::RadiationImage::GetCV_32SAsJet(radData.mHybridImage, image.rows, image.cols, minPortion, opacity);
                    }
                    else if (reconType == 1)
                    {
                        radImage = HUREL::Compton::RadiationImage::GetCV_32SAsJet(radData.mComptonImage, image.rows, image.cols, minPortion, opacity);
                    }
                    else if (reconType == 2)
                    {
                        radImage = HUREL::Compton::RadiationImage::GetCV_32SAsJet(radData.mCodedImage, image.rows, image.cols, minPortion, opacity);
                    }
                    else
                    {
                        radImage = cv::Mat();
                    }
                    overlayImage(image, radImage, radImageOveray);
                }
                return radImageOveray; 
                }
                );
        }

        if (recon2dFuture.valid())
        {
            // ImGui::Text("Reconstrcution 2D...........");
            if (recon2dFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                radImageOveray = recon2dFuture.get();
                isImageGet = true;
                recon2dFutreReady = true;

                glBindTexture(GL_TEXTURE_2D, texture2);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
                glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, radImageOveray.cols, radImageOveray.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, radImageOveray.data);
            }
        }
        else
        {
            // ImGui::Text("Done Reconstrcution 2D");
        }
    }
    // ImGui::SameLine();

    // combo box hybrid, compton, coded aperture

    ImGui::Image(reinterpret_cast<void *>(static_cast<intptr_t>(texture2)), ImVec2(radImageOveray.cols, radImageOveray.rows));
    ImGui::Combo("영상화 종류", &reconType, "하이브리드\0콤프턴\0부호화구경\0");

    // set counts
    if (ImGui::InputInt("유효 이벤트 수", &effectiveCount, 10, 100))
    {
        if (effectiveCount < 0)
        {
            effectiveCount = 0;
        }

        isImageGet = false;
        ChangeSettingValues("effectiveCount", std::to_string(effectiveCount));
    }

    if (ImGui::Checkbox("흑백", &isGrayScale))
    {
        isImageGet = false;
        ChangeSettingValues("isGrayScale", std::to_string(isGrayScale ? 1 : 0));
    }
    if (ImGui::SliderFloat("선원 검출기간 거리", &s2M, 0.0f, 5.0f))
    {
        isImageGet = false;
        ChangeSettingValues("s2M", std::to_string(s2M));
    }
    ImGui::SameLine();
    if (ImGui::Checkbox("거리 자동측정", &isAutoDistance))
    {
        isImageGet = false;
        ChangeSettingValues("isAutoDistance", std::to_string(isAutoDistance ? 1 : 0));
    }
    if (ImGui::SliderFloat("검출기 넓이", &det_W, 0.0f, 0.5f))
    {
        isImageGet = false;
        ChangeSettingValues("det_W", std::to_string(det_W));
    }
    if (ImGui::SliderFloat("해상도 보정", &resImprov, 0.0f, 50.0f))
    {
        isImageGet = false;
        ChangeSettingValues("resImprov", std::to_string(resImprov));
    }
    if (ImGui::SliderFloat("영상화 최소 비율", &minPortion, 0.0f, 1.0f))
    {
        isImageGet = false;
        ChangeSettingValues("minPortion", std::to_string(minPortion));
    }
    if (ImGui::SliderFloat("투명도", &opacity, 0.0f, 1.0f))
    {
        isImageGet = false;
        ChangeSettingValues("opacity", std::to_string(opacity));
    }

    if (ImGui::SliderFloat("검출기 마스크 간 거리", &m2D, 0.0f, 0.2f))
    {
        isImageGet = false;
        ChangeSettingValues("m2D", std::to_string(m2D));
    }
    if (ImGui::SliderFloat("수직 Fov", &hFov, 0.1f, 179.0f))
    {
        isImageGet = false;
        ChangeSettingValues("hFov", std::to_string(hFov));
    }
    if (ImGui::SliderFloat("수평 Fov", &wFov, 0.1f, 179.0f))
    {
        isImageGet = false;
        ChangeSettingValues("wFov", std::to_string(wFov));
    }

    ImGui::End();

    return true;
}

void HUREL::GUI::InformationWindow(bool initial, Compton::SessionData *&sessionDataOrNull)
{
    Init();
    static int countRateTime;
    static bool isShowCountRateRealtime;
    if (initial)
    {
        isShowCountRateRealtime = true;
        countRateTime = 60;
    }
    static std::chrono::system_clock::time_point start;

    if (initial)
    {
        start = std::chrono::system_clock::now();
    }
    ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
    ImGui::Begin("정보창", nullptr, ImGuiWindowFlags_NoBringToFrontOnFocus);
    // set time ui
    ImGui::Checkbox("실시간 업데이트", &isShowCountRateRealtime);
    ImGui::InputInt("시간 범위", &countRateTime, 1, 10);
    // draw line graph with implot as x label time and y label count rate of session
    if (ImPlot::BeginPlot("계수율"))
    {
        ImPlot::SetupAxes("시간 (s)", "계수율 (kcps)",
                          ImPlotAxisFlags_::ImPlotAxisFlags_LockMin,
                          ImPlotAxisFlags_::ImPlotAxisFlags_LockMin);
        if (initial)
        {
            ImPlot::SetupAxesLimits(0, 10, 0, 10, ImGuiCond_::ImGuiCond_Always);
        }

        if (sessionDataOrNull != nullptr)
        {
            std::vector<double> countrate = sessionDataOrNull->GetCountRateEvery500ms();
            std::vector<double> timeForCountrate;
            timeForCountrate.reserve(countrate.size());

            for (int i = 0; i < countrate.size(); i++)
            {
                timeForCountrate.push_back(i);
            }
            if (isShowCountRateRealtime && countrate.size() > 0)
            {
                double minVal = timeForCountrate[timeForCountrate.size() - 1] - countRateTime;
                double maxVal = timeForCountrate[timeForCountrate.size() - 1];
                if (minVal < 0)
                {
                    minVal = 0;
                }
                // Set x axis limit
                ImPlot::SetupAxisLimits(0, minVal, maxVal, ImGuiCond_Always);
            }

            ImPlot::PlotLine("계수율", timeForCountrate.data(), countrate.data(), countrate.size());
            std::chrono::milliseconds startTime = sessionDataOrNull->GetStartTime();
        }

        ImPlot::EndPlot();
    }
    ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
    std::chrono::duration<double> time = std::chrono::system_clock::now() - start;

    // print time as hour:minute:second
    int hour = time.count() / 3600;
    int minute = (time.count() - hour * 3600) / 60;
    int second = time.count() - hour * 3600 - minute * 60;
    ImGui::Text("프로그램 동작 시간: %02d:%02d:%02d", hour, minute, second);

    if (sessionDataOrNull != nullptr)
    {

        ImGui::Text("세션 폴더 경로: %s", sessionDataOrNull->GetLoadFileDir().c_str());
        ImGui::Text("세션 리스트 모드 데이터 크기: %lu", sessionDataOrNull->GetSizeListedListModeData());
        long long duration = sessionDataOrNull->GetEndTime().count() - sessionDataOrNull->GetStartTime().count();
        // print time as hour:minute:second:milisecond
        hour = duration / 3600 / 1000;
        minute = (duration - hour * 3600 * 1000) / 60 / 1000;
        second = (duration - hour * 3600 * 1000 - minute * 60 * 1000) / 1000;
        int milisecond = (duration - hour * 3600 * 10000 - minute * 60 * 1000 - second * 1000);
        ImGui::Text("세션 동작 시간: %02d:%02d:%02d:%03d", hour, minute, second, milisecond);
    }

    ImGui::End();
}

void HUREL::GUI::ControlWindow(bool initial, Compton::SessionData *&sessionDataOrNull)
{
    Init();
    static std::future<bool> startSessionFuture;
    static bool isStartSession;
    static bool isStartingSession;
    static bool isStopingSession;
    static std::string sessionName;
    static char sessionNameChar[256];

    if (initial)
    {
        isStartSession = false;
        isStartingSession = false;
        isStopingSession = false;
        sessionName = "session";
        strcpy(sessionNameChar, sessionName.c_str());
    }
    ImGui::SetNextWindowViewport(ImGui::GetMainViewport()->ID);
    ImGui::Begin("동작창", nullptr, ImGuiWindowFlags_NoBringToFrontOnFocus);

    ImGui::Checkbox("실시간 동작", &HUREL::GUI::mIsRealtimeRun);
    if (HUREL::GUI::mIsRealtimeRun)
    {
        if (sessionDataOrNull != nullptr && sessionDataOrNull != &HUREL::Compton::LahgiControl::instance().GetLiveSessionData())
        {
            delete sessionDataOrNull;
        }
        sessionDataOrNull = &HUREL::Compton::LahgiControl::instance().GetLiveSessionData();

        if (isStartSession)
        {
            ImGui::Text("세션이 동작 중 입니다.");
        }
        else if (isStartingSession)
        {
            ImGui::Text("세션이 시작 중 입니다.");
        }
        else if (isStopingSession)
        {
            ImGui::Text("세션이 종료 중 입니다.");
        }
        else
        {
            ImGui::Text("세션이 동작하고 있지 않습니다.");
        }

        // session name input
        ImGui::InputText("세션 이름", sessionNameChar, 256);

        if (!isStartSession && (isStartingSession || isStopingSession))
        {
            // set button to disabled and grey
            ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0.0f, 0.0f, 0.4f));
            ImGui::Button("시작");
            ImGui::PopStyleColor();
        }
        if (!isStartSession && !(isStartingSession || isStopingSession))
        {
            if (ImGui::Button("시작"))
            {
                // start session asynchronusly
                startSessionFuture = std::async(std::launch::async,
                                                []() -> bool
                                                { return HUREL::Compton::LahgiControl::instance().StartSession(); });
                isStartingSession = true;
            }
        }
        if (isStartSession && !isStartingSession)
        {
            if (ImGui::Button("Stop"))
            {
                // stop session
                startSessionFuture = std::async(std::launch::async,
                                                []() -> bool
                                                { HUREL::Compton::LahgiControl::instance().StopSession(sessionNameChar);
                                                    return true; });
                isStartSession = false;
                isStopingSession = true;
            }
        }
        if (isStartSession && isStartingSession)
        {
            ImGui::Button("Error");
        }

        if (startSessionFuture.valid())
        {
            if (startSessionFuture.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
            {
                if (startSessionFuture.get())
                {
                    if (!isStopingSession)
                    {
                        mSelectedIsotopeData = HUREL::IsotopeData();
                        isStartSession = true;
                    }
                    isStartingSession = false;
                    isStopingSession = false;
                }
                else
                {
                    isStartingSession = false;

                    isStopingSession = false;
                }
            }
        }

        // show status as green or red

        // hivoltage
        ImGui::Text("검출기 상태: ");
        ImGui::SameLine();
        if (HUREL::Compton::LahgiControl::instance().GetHvStatus())
        {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "정상");
        }
        else
        {
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "오류");
        }

        // fgpa
        ImGui::Text("FPGA 상태: ");
        ImGui::SameLine();
        if (HUREL::Compton::LahgiControl::instance().GetFPGAStatus())
        {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "정상");
        }
        else
        {
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "오류");
        }

        // camera
        ImGui::Text("카메라 상태: ");
        ImGui::SameLine();
        if (HUREL::Compton::LahgiControl::instance().GetCameraStatus())
        {
            ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "정상");
        }
        else
        {
            ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "오류");
        }
    }
    else
    {
        nfdchar_t *fileDir = nullptr;
        if (ImGui::Button("불러오기"))
        {

            char cwd[PATH_MAX];
            if (getcwd(cwd, sizeof(cwd)) == nullptr)
            {
                perror("getdwd() error");
                ImGui::End();
                return;
            }
            nfdresult_t result = NFD_PickFolder(".", &fileDir);
            if (result == NFD_OKAY)
            {
                if (sessionDataOrNull != nullptr && sessionDataOrNull != &HUREL::Compton::LahgiControl::instance().GetLiveSessionData())
                {
                    delete sessionDataOrNull;
                }
                sessionDataOrNull = new HUREL::Compton::SessionData(fileDir);
                if (sessionDataOrNull->GetSizeListedListModeData() == 0)
                {
                    delete sessionDataOrNull;
                    sessionDataOrNull = nullptr;
                }
                else
                {
                    isImageGet = false;
                }
            }
            else if (result == NFD_CANCEL)
            {
                puts("Cancel");
            }
            else
            {
                printf("Error: %s\n", NFD_GetError());
            }
        }
        if (sessionDataOrNull != nullptr)
        {
            ImGui::Text("경로: %s", sessionDataOrNull->GetLoadFileDir().c_str());
        }
    }

    ImGui::End();
}

void HUREL::GUI::SettingWindow(bool initial, Compton::SessionData *&sessionDataOrNull)
{
}

void HUREL::GUI::MakeBeep(double timeItervalInSecond)
{
    static double setBeepTime = 0.0;

    static std::chrono::time_point<std::chrono::system_clock> beforeBeepTime = std::chrono::system_clock::now();
    
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    if (setBeepTime != timeItervalInSecond)
    {
        //start timer
        setBeepTime = timeItervalInSecond;
        beforeBeepTime = std::chrono::system_clock::now();
    }
    else
    {
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - beforeBeepTime).count() > setBeepTime * 1000)
        {
            // beep
            std::cout << "beep" << std::endl;
            beforeBeepTime = std::chrono::system_clock::now();
        }
    }
}
