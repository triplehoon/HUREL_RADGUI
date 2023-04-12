#pragma once

#include <stdio.h>
#include <pthread.h>
#include <memory.h>
#include <future>
#include <queue>

#include <tbb/concurrent_queue.h>
#include "cyusb.h"

#include "spdlog/spdlog.h"

namespace HUREL
{
    namespace CRUXELL {

        class CruxellIO
        {
        private:      
            CruxellIO();        

            uint8_t mInEnpointAddress = 0;
            uint8_t mOutEnpointAddress = 0;

            int tb_0x11, tb_0x12, tb_0x13, tb_0x14, tb_0x15, tb_0x16, tb_0x17, tb_0x18, tb_0x1a, tb_0x1b, tb_0x0a, tb_0x0b, tb_0x1c;
            int tb_0x19_0, tb_0x19_1, tb_0x19_2, tb_0x19_3, tb_0x19_4, tb_0x19_5, tb_0x19_6, tb_0x19_7, tb_0x19_8, tb_0x19_9, tb_0x19_10, tb_0x19_11, tb_0x19_12, tb_0x19_13, tb_0x19_14, tb_0x19_15;

            
            bool mIsConnect = false;
            bool mIsRunning = false;

            void (*mFuncAdder)(const unsigned short*);

            

            pthread_t tidParsingThread;
            pthread_t tidParsingThread2;

            void tryparse_send(unsigned char* dataInput, int data_size, int data, int add, int sub_add );
            void usb_setting(int flag);

            tbb::concurrent_queue<std::pair<int, unsigned char*>> mDataQueue = tbb::concurrent_queue<std::pair<int, unsigned char*>>();

            static void tranferCallback(libusb_transfer* transfer);
            static void* parserLoop(void* arg1);
            static void* parserLoop2(void* arg1);

        public:
            static CruxellIO& instance();
            cyusb_handle *h1;

            ~CruxellIO();

            void Connect();
            bool IsConnect();
            void SetTestSettingValues();
            
            void SetByteDataAddingFunction(void (*const func)(const unsigned short*));

            bool Run();
            bool Stop();

            bool IsRunning();


        };
        
    

    }
    
} // namespace HUREL