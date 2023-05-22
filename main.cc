// #include "df2_filter.h"
#include <cstring>
#include <memory>
#include <string>
#include <iostream>

#include <stdexcept>
#include <future>
#include <chrono>
#include <thread>

#include <chrono>
#include <ctime>

#include "config.h"
#include "PaddleController.h"
#include "KincoDriver.h"
#include "TerminalInterface.h"

const char driverSerialPath[] = "/dev/ttyUSB0";
const char paddleSerialPath[] = "/dev/ttyACM2";


int main()
{
    PaddleController *paddleCtrlObj = new PaddleController(paddleSerialPath, 9600);

#if defined(LFAST_TERMINAL)
    TerminalInterface *terminal = new TerminalInterface("SLEW DRIVE TEST INTERFACE");
    paddleCtrlObj->setupTerminal(terminal);
#endif
    // bool connected = paddleCtrlObj->connectToDrivers(driverSerialPath);
    bool connected = true;
    // paddleCtrlObj->configureDrivers();

    bool keepGoing = true;

    if (connected)
    {
        static long int n = 0;
        try
        {
            while (keepGoing)
            {
                // paddleCtrlObj->testUpdate();
                constexpr double T_s = 1.0 / UPDATE_RATE_HZ;
                constexpr int32_t slpPrd_us = (int32_t)T_s * 1000000;
                std::thread tu = paddleCtrlObj->commandUpdate();
                paddleCtrlObj->readSerialBuff();
                paddleCtrlObj->processReceived();
                std::this_thread::sleep_for(std::chrono::microseconds(slpPrd_us));
                tu.join();
            }
        }
        catch (const std::exception &e)
        {
            #if defined(LFAST_TERMINAL)
            terminal->addDebugMessage(e.what());
            #else
            std::cout << e.what();
            #endif
        }
    }

    paddleCtrlObj->shutdown();

    return 0;
}