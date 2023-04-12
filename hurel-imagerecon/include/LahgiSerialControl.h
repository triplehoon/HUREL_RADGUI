#pragma once

// C library headers
#include <string>
#include <iostream>
#include <vector>
#include <sstream>

// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

#include "spdlog/spdlog.h"

namespace HUREL
{
    namespace Compton
    {

        class LahgiSerialControl
        {
        private:
            int mSerialPort;
            std::string mSerialPortName;
            struct termios mSerialPortSettings;
            bool mIsConnected = false;

            std::vector<std::string> mSerialPortList;
                
            LahgiSerialControl();
            ~LahgiSerialControl();

        public:
            static LahgiSerialControl &GetInstance()
            {
                static LahgiSerialControl instance;
                return instance;
            }

            double mHvVoltage;
            double mHvCurrent;

            double mBatteryVoltage;

            bool mIsFpgaOn;
            bool mIsSwitchOn[6];

            void OpenSerialPort(std::string portName);
            void CloseSerialPort();

            bool IsSerialPortOpen();

            void GetSerialPortList();

            void SendCommand(std::string command);
            void ReadCommand(std::string &command);
            void ReadLine(std::string &command);

            bool CheckConnection();

            void CheckValues(bool isSendCheckCommand = false);

            bool SetFpga(bool isFpgaOn);
            bool SetHv(bool isHvOn);

            bool SetSwtich(int switchNumber, bool isSwitchOn);

            static void TestLahgiSerialControl();
        };

    };
};