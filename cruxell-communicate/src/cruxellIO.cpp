#include "cruxellIO.h"

using namespace HUREL::CRUXELL;

HUREL::CRUXELL::CruxellIO &HUREL::CRUXELL::CruxellIO::instance()
{
    static CruxellIO &instance = *(new CruxellIO());
    return instance;
}

HUREL::CRUXELL::CruxellIO::CruxellIO() : h1(NULL)
{
    spdlog::info("hello cruxell IO");

    Connect();
    SetTestSettingValues();

    if (mIsConnect)
    {
        spdlog::info("Start reset connection with fpga");
        usb_setting(2);
        usb_setting(3);
        spdlog::info("Done reset connection with fpga");
    }
}

void HUREL::CRUXELL::CruxellIO::tryparse_send(unsigned char *dataInput, int data_size, int data, int add, int sub_add)
{
    if (dataInput == nullptr)
    {
        return;
    }

    int temp_send_buffer[1024];
    memset(temp_send_buffer, 0, 1024 * sizeof(int));
    int transfered = 0;
    int get_data = data;

    for (int i = 0; i < 700; ++i)
    {
        bool is_sub_add_minus_one = sub_add == -1;
        if (is_sub_add_minus_one)
        {
            temp_send_buffer[i] = (add << 24) | (get_data & 0xFFFFFF);
        }
        else
        {
            temp_send_buffer[i] = (add << 24) | (sub_add << 16) | (get_data & 0xFFFF);
        }
    }

    memcpy(dataInput, temp_send_buffer, 1024 * sizeof(int));
    int ecount = 0;
    for (int i = 0; i < 1000; ++i)
    {
        int r = cyusb_bulk_transfer(h1, mOutEnpointAddress, dataInput, 4096, &transfered, 10);
        if (r != 0)
        {
            ++ecount;
        }
        else
        {
            break;
        }
    }
    if (ecount > 0)
    {
        spdlog::warn("try parse ecount: {0}", ecount);
    }
}

HUREL::CRUXELL::CruxellIO::~CruxellIO()
{
    cyusb_close();
}

void HUREL::CRUXELL::CruxellIO::Connect()
{
    int r = cyusb_open();

    if (r < 0)
    {
        spdlog::error("Error opening library\n");
        return;
    }
    else if (r == 0)
    {
        spdlog::error("No device found");
        return;
    }
    if (r > 1)
    {
        spdlog::error("More than 1 devices of interest found. Disconnect unwanted devices");
        return;
    }
    h1 = cyusb_gethandle(0);
    if (cyusb_getvendor(h1) != 0x04b4)
    {
        spdlog::error("Cypress chipset not detected");
        cyusb_close();
        return;
    }

    // r = cyusb_kernel_driver_active(h1, 0);
    // if ( r != 0 ) {
    //     spdlog::error("kernel driver active. Exitting\n");
    //    cyusb_close();
    //    return;
    // }
    r = cyusb_claim_interface(h1, 0);
    if (r != 0)
    {
        spdlog::error("Error in claiming interface");
        cyusb_error(r);
        if (r == LIBUSB_ERROR_BUSY)
        {
            spdlog::warn("Detaching kernel driver. please retry connection");
            r = cyusb_detach_kernel_driver(h1, 0);
            cyusb_error(r);
        }
        cyusb_close();
        return;
    }

    libusb_config_descriptor *configPointer = NULL;

    int ecode = cyusb_get_active_config_descriptor(h1, &configPointer);
    libusb_config_descriptor &config = *configPointer;
    if (ecode == 0)
    {
        for (int i = 0; i < config.bNumInterfaces; ++i)
        {
            libusb_interface interface = config.interface[i];
            if (interface.num_altsetting == 0)
            {
                spdlog::error("No altsetting");
                break;
            }

            libusb_interface_descriptor descpriptor = *interface.altsetting;

            for (int j = 0; j < descpriptor.bNumEndpoints; ++j)
            {
                libusb_endpoint_descriptor ep = descpriptor.endpoint[j];
                if (ep.bEndpointAddress & libusb_endpoint_direction::LIBUSB_ENDPOINT_IN)
                {
                    spdlog::info("cypress endpoint-in 0x{0:x}", ep.bEndpointAddress);
                    mInEnpointAddress = ep.bEndpointAddress;
                }
                else
                {
                    spdlog::info("cypress endpoint-out 0x{0:x}", ep.bEndpointAddress);
                    mOutEnpointAddress = ep.bEndpointAddress;
                }
            }
        }
    }

    mIsConnect = true;

    spdlog::info("cypress connect complete");
}

bool HUREL::CRUXELL::CruxellIO::IsConnect()
{
    return mIsConnect;
}

void HUREL::CRUXELL::CruxellIO::SetTestSettingValues()
{
    tb_0x11 = 2;

    tb_0x12 = 35; // // T.W for Global Trig
    tb_0x13 = 3;//
    tb_0x14 = 2;
    tb_0x15 = 85;
    tb_0x16 = 1;
    tb_0x17 = 100;
    tb_0x18 = 80;

    tb_0x19_0 = 4000;
    tb_0x19_1 = 4000;
    tb_0x19_2 = 4000;
    tb_0x19_3 = 4000;
    tb_0x19_4 = 10;
    tb_0x19_5 = 10;
    tb_0x19_6 = 10;
    tb_0x19_7 = 10;
    tb_0x19_8 = 4000;
    tb_0x19_9 = 4000;
    tb_0x19_10 = 4000;
    tb_0x19_11 = 4000;
    tb_0x19_12 = 10;
    tb_0x19_13 = 10;
    tb_0x19_14 = 10;
    tb_0x19_15 = 10;
    tb_0x1a = 1;
    tb_0x1b = 1;
    tb_0x0a = 0;
    tb_0x0b = 0;
    tb_0x1c = 0;

    if (tb_0x0a * tb_0x0b == 0)
    {
        tb_0x0a = 0;
        tb_0x0b = 0;
    }
}

void HUREL::CRUXELL::CruxellIO::SetByteDataAddingFunction(void (*const func)(const unsigned short *))
{
    mFuncAdder = func;
}

bool HUREL::CRUXELL::CruxellIO::Run()
{
    if (mIsRunning)
    {
        spdlog::warn("CruxellIo: Already the FPGA is running.");
        return true;
    }

    if (!mIsConnect)
    {
        spdlog::warn("CruxellIo: FPGA is not connected");
        return false;
    }

    usb_setting(1);
    mIsRunning = true;
    int result1 = pthread_create(&tidParsingThread, NULL, parserLoop, this) ;
    int result2 = pthread_create(&tidParsingThread2, NULL, parserLoop2, this) ;
    
    if (result1 < 0 || result2 < 0)
    {
        spdlog::error("CRUXELLIO parserLoop thread creation fail!!!");
        mIsRunning = false;

        usb_setting(3);
        return false;
    }
\
    return true;
}

bool HUREL::CRUXELL::CruxellIO::Stop()
{
    if (!mIsRunning)
    {
        spdlog::info("FPGA is not running. Stop procedure is finished");
        return true;
    }
    mIsRunning = false;

    usb_setting(2);
    usb_setting(3);
    pthread_join(tidParsingThread, nullptr);
    pthread_join(tidParsingThread2, nullptr);

    //cyusb_close();

    spdlog::info("FPGA stop procedure is finished");

    return true;
}

bool HUREL::CRUXELL::CruxellIO::IsRunning()
{
    return mIsRunning;
}

void HUREL::CRUXELL::CruxellIO::usb_setting(int flag)
{
    unsigned char outData[4096];
    int transfered[4096];
    memset(outData, '\0', 4096);
    int temp_send_buffer[1024];
    memset(temp_send_buffer, 0, 1024 * sizeof(int));

    if (flag == 1)
    {
        // 최초 셋팅값 전송
        tryparse_send(outData, 4096, tb_0x0a, 0x0a, -1);      // Sec
        tryparse_send(outData, 4096, tb_0x0b, 0x0b, -1);      // Count
        tryparse_send(outData, 4096, tb_0x11, 0x11, -1);      // Single / Coincidence Mode
        tryparse_send(outData, 4096, tb_0x12, 0x12, -1);      // T.W for Global Trig
        tryparse_send(outData, 4096, tb_0x13, 0x13, -1);      // Smooth Window
        tryparse_send(outData, 4096, tb_0x14, 0x14, -1);      // BL.Meas.Interval
        tryparse_send(outData, 4096, tb_0x15, 0x15, -1);      // BL.Offset
        tryparse_send(outData, 4096, tb_0x16, 0x16, -1);      // Max Offset
        tryparse_send(outData, 4096, tb_0x17, 0x17, -1);      // Max Meas.Interval
        tryparse_send(outData, 4096, tb_0x18, 0x18, -1);      // Trig.Slope Points
        tryparse_send(outData, 4096, tb_0x19_0, 0x19, 0x01);  // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_1, 0x19, 0x02);  // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_2, 0x19, 0x03);  // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_3, 0x19, 0x04);  // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_4, 0x19, 0x05);  // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_5, 0x19, 0x06);  // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_6, 0x19, 0x07);  // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_7, 0x19, 0x08);  // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_8, 0x19, 0x09);  // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_9, 0x19, 0x0a);  // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_10, 0x19, 0x0b); // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_11, 0x19, 0x0c); // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_12, 0x19, 0x0d); // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_13, 0x19, 0x0e); // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_14, 0x19, 0x0f); // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x19_15, 0x19, 0x10); // Local Trig.Threshold
        tryparse_send(outData, 4096, tb_0x1a, 0x1a, -1);      // No.Trig.for Det Trig
        tryparse_send(outData, 4096, tb_0x1b, 0x1b, -1);      // T.W for Det.Trig

        tryparse_send(outData, 4096, tb_0x1c, 0x1c, -1); // For DEBUG
        tryparse_send(outData, 4096, 1, 0x1d, -1);       // FIFO CLR Always 1

        // 런신호 (768 / 770)
        temp_send_buffer[768] = 1;
        temp_send_buffer[770] = 1;
        memcpy(outData, temp_send_buffer, 4096);
        int ecount = 0;

        for (int i = 0; i < 100; ++i)
        {
            int r = cyusb_bulk_transfer(h1, mOutEnpointAddress, outData, 4096, transfered, 10);
            if (r == 0)
            {
                break;
                ;
            }
            else
            {
                cyusb_clear_halt(h1, mOutEnpointAddress);
                ++ecount;
            }
        }
        if (ecount > 0)
        {
            spdlog::warn("usb setting 1 ecount: {0}", ecount);
        }
    }
    else if (flag == 2)
    {
        // Final call (768 / 770)
        temp_send_buffer[768] = 1;
        temp_send_buffer[769] = 1;
        temp_send_buffer[770] = 1;

        memcpy(outData, temp_send_buffer, 4096);
        int ecount = 0;
        for (int i = 0; i < 1000; ++i)
        {
            int r = cyusb_bulk_transfer(h1, mOutEnpointAddress, outData, 4096, transfered, 10);
            if (r == 0)
            {
                break;
            }
            else
            {
                cyusb_clear_halt(h1, mOutEnpointAddress);
                ++ecount;
            }
        }
        if (ecount > 0)
        {
            spdlog::warn("usb setting 2 ecount: {0}", ecount);
        }
    }
    else if (flag == 3)
    {
        memcpy(outData, temp_send_buffer, 4096);
        int ecount = 0;
        for (int i = 0; i < 1000; ++i)
        {
            int r = cyusb_bulk_transfer(h1, mOutEnpointAddress, outData, 4096, transfered, 10);
            if (r == 0)
            {
                break;
            }
            else
            {
                cyusb_clear_halt(h1, mOutEnpointAddress);
                ++ecount;
            }
        }
        if (ecount > 0)
        {
            spdlog::warn("usb setting 3 ecount: {0}", ecount);
        }
    }
}

static int packetCount = 0;

void HUREL::CRUXELL::CruxellIO::tranferCallback(libusb_transfer *transfer)
{
    CruxellIO &io = CruxellIO::instance();

    switch (transfer->status)
    {
    case LIBUSB_TRANSFER_COMPLETED:
        spdlog::info("Call back sucess");
        for (int i = 0; i < transfer->actual_length; ++i)
        {
            if (transfer->buffer[i] == 0xfe)
            {
                // spdlog::info("fefe in");
                ++packetCount;
                if (packetCount > 0 && packetCount % 100 == 0)
                {
                    spdlog::info("{0} 0xfe packet count", packetCount);
                }
            }
        }
        break;

    default:
        spdlog::info("Call back fail");
        break;
    }
}

void *HUREL::CRUXELL::CruxellIO::parserLoop(void *arg1)
{
    CruxellIO &io = CruxellIO::instance();

    int transferResult;
    constexpr size_t buffsize = 0x40000;
    int transferStackCount = 20;
    unsigned char** buf = new unsigned char *[transferStackCount];
    for (int i = 0; i < transferStackCount; ++i)
    {
        buf[i] = new unsigned char[buffsize];
        memset(buf[i], 0xff, buffsize);
    }
    int transferred;
    size_t packetCount = 0;

    unsigned short testBuff[144];

  
    int timeOutCount = 0;

    //time measure
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();

    size_t transferStackIndex = 0;

    while (io.mIsRunning)
    {
        start = std::chrono::high_resolution_clock::now();

        transferResult = cyusb_bulk_transfer(io.h1, io.mInEnpointAddress, buf[transferStackIndex], buffsize, &transferred, 1000);

        if (transferResult == 0)
        {
            while (io.mDataQueue.unsafe_size() == 20)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                spdlog::warn("wait until queue is empty");
            }
            io.mDataQueue.push(std::make_pair(transferred, buf[transferStackIndex]));
            transferStackIndex = (transferStackIndex + 1);
            if (transferStackIndex >= transferStackCount)
            {
                transferStackIndex = 0;
            }
            timeOutCount =  0;
        }
        else
        {

            if (transferResult == LIBUSB_ERROR_TIMEOUT)
            {
                spdlog::warn("There are no data to process (time out)");
                ++timeOutCount;
                if (timeOutCount > 5)
                {
                    spdlog::warn("time out count is over 5, reset usb");
                    CRUXELL::CruxellIO::instance().usb_setting(2);
                    CRUXELL::CruxellIO::instance().usb_setting(3);
                    CRUXELL::CruxellIO::instance().usb_setting(1);
                    timeOutCount = 0;
                }
            }
            else if (transferResult == LIBUSB_ERROR_NO_DEVICE)
            {
                spdlog::error("Device is deiconnected break the reader loop (no device");
                break;
            }
            else
            {
                spdlog::error("ohter libusb erro: {0}", transferResult);
                break;
            }
        }
    }

    // empty io buffer    
    while (transferResult == 0)
    {
        transferResult = cyusb_bulk_transfer(io.h1, io.mInEnpointAddress, buf[transferStackIndex], buffsize, &transferred, 1000);
    }

    delete[] buf;

    return nullptr;
}

void *HUREL::CRUXELL::CruxellIO::parserLoop2(void *arg1)
{
    CruxellIO &io = CruxellIO::instance();
    //start time
    auto start = std::chrono::high_resolution_clock::now();
    int flag = 0; // nothing 1 is find FE, 2 find seond FE
    int countFlag = 0;


    unsigned char dataBuffer[296];
    memset(dataBuffer, 0, 296);
    while (io.mIsRunning)
    {
        std::pair<int, unsigned char *> data;
        while(!io.mDataQueue.try_pop(data))
        {
            //sleep thread
            if (!io.mIsRunning)
            {
                return nullptr;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        start = std::chrono::high_resolution_clock::now();
        int transferred = data.first;
        unsigned char *buf = data.second;
        for (int i = 0; i < transferred; ++i)
        {
            if (flag == 2)
            {
                dataBuffer[countFlag] = buf[i];
                ++countFlag;
                if (countFlag == 296 && dataBuffer[294] == 0xFE && dataBuffer[295] == 0xFE)
                {
                    ++packetCount;

                    io.mFuncAdder((unsigned short *)dataBuffer);
                    countFlag = 0;
                }
                else if (countFlag == 296)
                {
                    spdlog::warn("reset flag as 0");
                    countFlag = 0;
                    flag = 0;
                }
            }
            else
            {
                if (buf[i] == 0xFE && flag == 0)
                {
                    spdlog::info("fpga flag is 1");
                    flag = 1;
                }
                else if (buf[i] == 0xFE && flag == 1)
                {
                    spdlog::info("fpga flag is 2");
                    flag = 2;
                }
                else
                {
                    // spdlog::info("fpga flag is 0");
                    flag = 0;
                }
            }
        }
        //spdlog::warn("data compute transferCount: {0}", transferred);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = (end - start);
        //spdlog::warn("data compute elapsed time: {0} ms", elapsed.count() * 1000);
    }

    return nullptr;
}
