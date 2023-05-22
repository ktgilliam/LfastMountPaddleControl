#include "PaddleController.h"

#include <cmath>
#include <iostream>
#include <cstring> // string function definitions
#include <chrono>
#include <thread>
#include <regex>

#include "config.h"
using namespace mn::CppLinuxSerial;

SerialPort *paddleSerial;
using namespace std::chrono_literals;
// int drvAID = 1;
// int drvBID = 2;
PaddleController::PaddleController(const char *devPath, uint32_t baud)
{
    // configureSerialComms(devPath);
    // size = RX_BUFF_SIZE;
    paddleSerial = new SerialPort(devPath, BaudRate::B_9600, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    paddleSerial->SetTimeout(100); // Block for up to 100ms to receive data
    paddleSerial->Open();
    std::this_thread::sleep_for(100ms);
    pAzDriveA = new KincoDriver(AZ_DRIVER_ID_A);
    pAzDriveB = new KincoDriver(AZ_DRIVER_ID_B);
    pElDriveA = new KincoDriver(EL_DRIVER_ID_A);
    pElDriveB = new KincoDriver(EL_DRIVER_ID_B);
    ElMotorACommand = 0.0;
    ElMotorBCommand = 0.0;
    AzMotorACommand = 0.0;
    AzMotorBCommand = 0.0;
    connected = false;
}
void PaddleController::readSerialBuff()
{
    // char data[RX_BUFF_SIZE];
    // std::memset(data, '\0', sizeof(data));
    std::string rxData;
    readSerialBuff(rxData);
    receivedStrings.push_back(rxData);
}
void PaddleController::readSerialBuff(std::string &data)
{
    paddleSerial->Read(data);
}

void PaddleController::processReceived()
{
    // size_t pos = 0;
    // uint32_t firstFour = *buf;
    try
    {
        std::regex rgx("\\{(EL|AZ):([-+]?[0-9]*\\.[0-9]+|[0-9]+)\\}");

        // std::regex rgx("\\{(AZ)");
        // std::regex rgx(R"({(EL|AZ):([-+]?[0-9]*\.[0-9]+|[0-9]+)})");
        // std::regex rgx(R"('{(EL|AZ):')");
        for (auto &data : receivedStrings)
        {
            std::smatch matches;
            if (std::regex_search(data, matches, rgx))
            {
                // std::cout << "Match found\n";
                if (matches.size() == 3)
                {
                    std::string axisStr = matches[1];
                    std::string valStr = matches[2];
                    double val = std::stof(valStr);
                    int b = 4;
                }
                for (size_t ii = 0; ii < matches.size(); ++ii)
                {
                    std::cout << ii << ": '" << matches[ii].str() << "'\n";
                }
            }
            else
            {
                // std::cout << "Match not found\n";
            }

            // std::cout << "Read data = \"" << data << "\"" << std::endl;
        }
        receivedStrings.clear();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        int a = 5;
    }
}

bool PaddleController::connectToDrivers(const char *devPath)
{
    bool result = true;
    try
    {
        KincoDriver::initializeRTU(devPath, 19200, 'N', 8, 1);
        result &= pAzDriveA->driverHandshake();
        result &= pAzDriveB->driverHandshake();
        result &= pElDriveA->driverHandshake();
        result &= pElDriveB->driverHandshake();
    }
    catch (const std::exception &e)
    {
#if defined(LFAST_TERMINAL)
        terminal->addDebugMessage(e.what(), TERM::ERROR);
#else
        std::cout << e.what();
#endif
    }
    connected = result;
    return connected;
}

bool PaddleController::configureDrivers()
{
    try
    {
        pAzDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
        pAzDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);
        pElDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
        pElDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);

        pAzDriveA->setDirectionMode(KINCO::CCW_IS_POSITIVE);
        pAzDriveB->setDirectionMode(KINCO::CW_IS_POSITIVE);
        pElDriveA->setDirectionMode(KINCO::CCW_IS_POSITIVE);
        pElDriveB->setDirectionMode(KINCO::CW_IS_POSITIVE);

        pAzDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pAzDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pElDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pElDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);

        pAzDriveA->zeroPositionOffset();
        pAzDriveB->zeroPositionOffset();
        pElDriveA->zeroPositionOffset();
        pElDriveB->zeroPositionOffset();

        pAzDriveA->updateVelocityCommand(0.0);
        pAzDriveB->updateVelocityCommand(0.0);
        pElDriveA->updateVelocityCommand(0.0);
        pElDriveB->updateVelocityCommand(0.0);

        pAzDriveA->updateTorqueCommand(0.0);
        pAzDriveB->updateTorqueCommand(0.0);
        pElDriveA->updateTorqueCommand(0.0);
        pElDriveB->updateTorqueCommand(0.0);

        pAzDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
        pAzDriveB->setControlMode(KINCO::MOTOR_MODE_SPEED);
        pElDriveA->setControlMode(KINCO::MOTOR_MODE_SPEED);
        pElDriveB->setControlMode(KINCO::MOTOR_MODE_SPEED);

        pAzDriveA->setDriverState(KINCO::POWER_ON_MOTOR);
        pAzDriveB->setDriverState(KINCO::POWER_ON_MOTOR);
        pElDriveA->setDriverState(KINCO::POWER_ON_MOTOR);
        pElDriveB->setDriverState(KINCO::POWER_ON_MOTOR);
    }
    catch (const std::exception &e)
    {
#if defined(LFAST_TERMINAL)
        terminal->addDebugMessage(e.what(), TERM::WARNING);
#else
        std::cout << e.what();
#endif
    }
    return true;
}

void PaddleController::updateCommands()
{
}

#if defined(LFAST_TERMINAL)
void PaddleController::setupTerminal(TerminalInterface *_terminal)
{
    this->terminal = _terminal;

    pAzDriveA->connectTerminalInterface(terminal);
    pAzDriveB->connectTerminalInterface(terminal);
    pElDriveA->connectTerminalInterface(terminal);
    pElDriveB->connectTerminalInterface(terminal);

    terminal->printHeader();
}
#endif

void PaddleController::shutdown()
{
    try
    {
        pAzDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pAzDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pElDriveA->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);
        pElDriveB->setMaxSpeed(KINCO::MOTOR_MAX_SPEED_RPM);

        pAzDriveA->updateVelocityCommand(0);
        pAzDriveB->updateVelocityCommand(0);
        pElDriveA->updateVelocityCommand(0);
        pElDriveB->updateVelocityCommand(0);

        pAzDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
        pAzDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);
        pElDriveA->setDriverState(KINCO::POWER_OFF_MOTOR);
        pElDriveB->setDriverState(KINCO::POWER_OFF_MOTOR);
    }
    catch (const std::exception &e)
    {
#if defined(LFAST_TERMINAL)
        terminal->addDebugMessage(e.what(), TERM::WARNING);
#else
        std::cout << e.what();
#endif
    }
#if defined(LFAST_TERMINAL)
    std::cout << VT100::CURSOR_TO_ROW_COL(50, 0);
    std::cout << logPathStr;
#endif
}
