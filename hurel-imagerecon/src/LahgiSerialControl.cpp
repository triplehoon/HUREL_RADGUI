#include "LahgiSerialControl.h"

HUREL::Compton::LahgiSerialControl::LahgiSerialControl()
{
    mSerialPort = -1;
    mSerialPortName = "";


    GetSerialPortList();

    if (mSerialPortList.size() == 0)
    {
        spdlog::error("LahgiSerialControl::LahgiSerialControl - No serial port found");

        spdlog::error("Bind bluetooth machine with laptop. Example below");
        spdlog::error("sudo rfcomm bind 0 98:D3:71:FD:C6:77 1");
        spdlog::error("sudo rfcomm bind 0 (--MAC address--) 1");
        return;
    } 
    CheckConnection();

    CheckValues(true);
    if (mIsFpgaOn == false)
    {
        SetFpga(true);
    }
    if (mHvVoltage < 500)
    {
        SetHv(false);
        SetHv(true);
    }

}

HUREL::Compton::LahgiSerialControl::~LahgiSerialControl()
{
    CloseSerialPort();
}

void HUREL::Compton::LahgiSerialControl::OpenSerialPort(std::string portName)
{
    spdlog::info("LahgiSerialControl::OpenSerialPort - {}", portName);
    mSerialPort = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (mSerialPort == -1)
    {
        spdlog::error("LahgiSerialControl::OpenSerialPort - Unable to open port {}", portName);
    }
    else
    {
        fcntl(mSerialPort, F_SETFL, 0);
        tcgetattr(mSerialPort, &mSerialPortSettings);
        cfsetispeed(&mSerialPortSettings, B9600);
        cfsetospeed(&mSerialPortSettings, B9600);
        mSerialPortSettings.c_cflag &= ~PARENB;
        mSerialPortSettings.c_cflag &= ~CSTOPB;
        mSerialPortSettings.c_cflag &= ~CSIZE;
        mSerialPortSettings.c_cflag |= CS8;
        mSerialPortSettings.c_cflag &= ~CRTSCTS;
        mSerialPortSettings.c_cflag |= CREAD | CLOCAL;
        mSerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);
        mSerialPortSettings.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        mSerialPortSettings.c_oflag &= ~OPOST;
        mSerialPortSettings.c_cc[VMIN] = 0;
        mSerialPortSettings.c_cc[VTIME] = 10;
        tcsetattr(mSerialPort, TCSANOW, &mSerialPortSettings);

        if (tcsetattr(mSerialPort, TCSANOW, &mSerialPortSettings) != 0)
        {
            printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        }
    }
}

void HUREL::Compton::LahgiSerialControl::CloseSerialPort()
{
    spdlog::info("LahgiSerialControl::CloseSerialPort");
    if (mSerialPort != -1)
    {
        close(mSerialPort);
        mSerialPort = -1;
        mSerialPortName = "";
        mIsConnected = false;
    }
}

bool HUREL::Compton::LahgiSerialControl::IsSerialPortOpen()
{
    return mSerialPort != -1;
}

void HUREL::Compton::LahgiSerialControl::GetSerialPortList()
{
    spdlog::info("LahgiSerialControl::GetSerialPortList");
    if (mSerialPortList.size() > 0)
    {
        return;
    }
    std::vector<std::string> portList;
    for (int i = 0; i < 10; ++i)
    {
        std::string portName = "/dev/rfcomm" + std::to_string(i);
        int fd = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

        // Check for errors
        if (fd != -1)
        {
            portList.push_back(portName);
            close(fd);
        }
    }
    mSerialPortList = portList;
}

void HUREL::Compton::LahgiSerialControl::SendCommand(std::string command)
{
    if (mSerialPort != -1)
    {
        write(mSerialPort, command.c_str(), command.length());
    }
    // delete last character
    if (command.length() > 0)
    {
        command = command.substr(0, command.length() - 1);
    }
    spdlog::info("LahgiSerialControl::SendCommand - {}", command);
}

static char *readCommandRemainder = NULL;
void HUREL::Compton::LahgiSerialControl::ReadCommand(std::string &command)
{

    spdlog::info("LahgiSerialControl::ReadCommand");
    // set timeout
    double timeOut = 10;

    // current time
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();

    if (mSerialPort != -1)
    {
        char buffer[1024];
        if (readCommandRemainder != NULL)
        {
            command = readCommandRemainder;
            


            delete[] readCommandRemainder;
            readCommandRemainder = NULL;
        }
        while (true)
        {
            if (command.find("\n") != std::string::npos)
            {
               
                break;
                
            }
            end = std::chrono::system_clock::now();
            /* code */
            int n = read(mSerialPort, (void *)buffer, 1024);
            if (n > 0)
            {
                buffer[n] = 0;
                command += buffer;
            }
            

            if (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() > timeOut * 1000)
            {
                spdlog::info("LahgiSerialControl::ReadCommand - Timeout");
                break;
            }
        }

        // save command afer \n to readCommandRemainder
        if (command.find("\n") != std::string::npos)
        {
            int index = command.find("\n");
            std::string remainder = command.substr(index + 1);
            readCommandRemainder = new char[remainder.length() + 1];
            strcpy(readCommandRemainder, remainder.c_str());
            command = command.substr(0, index);
            if (command.length() == 0)
            {
                ReadCommand(command);
            }
        }

        // delete last character
        if (command.length() > 0)
        {
            command = command.substr(0, command.length() - 1);
        }

        spdlog::info("LahgiSerialControl::ReadCommand - {}", command);
    }
}

static int CheckConnectionCount = 0;
bool HUREL::Compton::LahgiSerialControl::CheckConnection()
{
    if (mIsConnected)
    {
        return true;
    }

    GetSerialPortList();


    CheckConnectionCount++;
    if (CheckConnectionCount > 10)
    {
        CheckConnectionCount = 0;
        mIsConnected = false;
        CloseSerialPort();
        spdlog::error("LahgiSerialControl::CheckConnection - Lahgi is not connected. Try {} connections", CheckConnectionCount);
        return false;
    }
    spdlog::info("LahgiSerialControl::CheckConnection");
    if (mSerialPort == -1)
    {
        std::vector<std::string> ports = mSerialPortList;
        for (int i = 0; i < ports.size(); i++)
        {
            OpenSerialPort(ports[i]);
            if (IsSerialPortOpen())
            {
                std::string response;
                SendCommand("\n");
                //dump command response
                //ReadCommand(response);
                SendCommand("\n");
                SendCommand("\n");
                SendCommand("\n");
                SendCommand("check\n");
                ReadCommand(response);
                if (response.length() > 0)
                {
                    spdlog::info("LahgiSerialControl::CheckConnection - Lahgi is connected to {}", ports[i]);
                    mIsConnected = true;
                    return true;
                }
                else
                {
                    spdlog::info("LahgiSerialControl::CheckConnection - Lahgi is not connected to {}", ports[i]);
                    CloseSerialPort();
                    GetSerialPortList();
                    return CheckConnection();
                }
            }
        }
    }
    return true;
}

void HUREL::Compton::LahgiSerialControl::CheckValues(bool isSendCheckCommand)
{
    CheckConnection();
    spdlog::info("LahgiSerialControl::CheckValues");
    if (mSerialPort != -1)
    {
        if (isSendCheckCommand)
        {
            SendCommand("\n");
            SendCommand("check\n");
        }
        std::string response;
        ReadCommand(response);

        if (response.length() > 0)
        {
            std::stringstream ss(response);
            std::string item;
            while (std::getline(ss, item, ','))
            {
                std::stringstream ss2(item);
                std::string key;
                std::string value;
                std::getline(ss2, key, ':');
                std::getline(ss2, value, '\0');

                if (key == "hvvolt")
                {
                    mHvVoltage = std::stof(value);
                }
                if (key == "hvcurr")
                {
                    mHvCurrent = std::stof(value);
                }
                if (key == "batvolt")
                {
                    mBatteryVoltage = std::stof(value);
                }
                if (key == "fpga")
                {
                    if (value == "on")
                    {
                        mIsFpgaOn = true;
                    }
                    else
                    {
                        mIsFpgaOn = false;
                    }
                }
                for (int i = 0; i < 6; ++i)
                {
                    if (key == "switch" + std::to_string(i))
                    {
                        if (value == "on")
                        {
                            mIsSwitchOn[i] = true;
                        }
                        else
                        {
                            mIsSwitchOn[i] = false;
                        }
                    }
                }
            }
        }
        else
        {
            mHvVoltage = 0;
            mHvCurrent = 0;
            mBatteryVoltage = 0;
            mIsFpgaOn = false;
            for (int i = 0; i < 6; ++i)
            {
                mIsSwitchOn[i] = false;
            }
            CloseSerialPort();
        }
    }
    else
    {
        mHvVoltage = 0;
        mHvCurrent = 0;
        mBatteryVoltage = 0;
        mIsFpgaOn = false;
        for (int i = 0; i < 6; ++i)
        {
            mIsSwitchOn[i] = false;
        }
    }
}

bool HUREL::Compton::LahgiSerialControl::SetFpga(bool isFpgaOn)
{
    spdlog::info("LahgiSerialControl::SetFpga - {}", isFpgaOn);
    if (mSerialPort != -1)
    {
        if (isFpgaOn)
        {
            SendCommand("setfpga:on\n");
        }
        else
        {
            SendCommand("setfpga:off\n");
        }
        CheckValues(false);
    }
    return mIsFpgaOn;
}

bool HUREL::Compton::LahgiSerialControl::SetHv(bool isHvOn)
{
    std::chrono::time_point<std::chrono::system_clock> start, end;
    start = std::chrono::system_clock::now();
    int timeOutSeconds = 30;
    spdlog::info("LahgiSerialControl::SetHv - {}", isHvOn);
    if (mSerialPort != -1)
    {
        end = std::chrono::system_clock::now();
        if (isHvOn)
        {
            SendCommand("sethv:on\n");
        }
        else
        {
            SendCommand("sethv:off\n");
        }
        // Serial read line
        std::string response;
        ReadCommand(response);
        while (response != "done")
        {

            end = std::chrono::system_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(end - start).count() > timeOutSeconds)
            {
                spdlog::error("LahgiSerialControl::SetHv - Timeout");
                CloseSerialPort();
                return false;
            }
            // check string is float
            if (response.find_first_not_of("0123456789.") == std::string::npos)
            {
                mHvVoltage = std::stof(response);
                spdlog::info("LahgiSerialControl::SetHv - {}", mHvVoltage);
            }
            ReadCommand(response);
        }
        CheckValues(false);
    }
    if (mHvVoltage > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool HUREL::Compton::LahgiSerialControl::SetSwtich(int switchNumber, bool isSwitchOn)
{
    spdlog::info("LahgiSerialControl::SetSwtich - {} - {}", switchNumber, isSwitchOn);
    if (mSerialPort != -1)
    {
        if (isSwitchOn)
        {
            SendCommand("setswitch" + std::to_string(switchNumber) + ":on\n");
        }
        else
        {
            SendCommand("setswitch" + std::to_string(switchNumber) + ":off\n");
        }
        CheckValues(false);
    }
    return mIsSwitchOn[switchNumber];
}

void HUREL::Compton::LahgiSerialControl::TestLahgiSerialControl()
{
    // get instance
    LahgiSerialControl &control = LahgiSerialControl::GetInstance();

    // check connection
    //control.GetSerialPortList();

    auto deviceList = control.mSerialPortList;
    for (int i = 0; i < deviceList.size(); i++)
    {
        spdlog::info("LahgiSerialControl::TestLahgiSerialControl - {}", deviceList[i]);
    }
    if (deviceList.size() == 0)
    {
        spdlog::debug("LahgiSerialControl::TestLahgiSerialControl - No device found");
        return;
    }
    control.CheckConnection();

    //control.CheckValues(true);

    //control.SetFpga(true);

    sleep(5);
    
    //control.SetFpga(true);

    //control.SetHv(true);
}
