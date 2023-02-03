#include <stdio.h>

#include <iostream>


#include "LahgiControl.h"


int main(int argc, char* argv[])
{  
    

    
    spdlog::info("Current program: {0}", argv[0]);
    HUREL::Compton::LahgiControl::instance();
    return 0;
}