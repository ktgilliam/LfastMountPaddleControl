#include "PaddleController.h"
#include "config.h"
#include <cmath>

#include <iostream>
#include <stdio.h> // standard input / output functions
#include <stdlib.h>
#include <string.h> // string function definitions

#include <unistd.h>  // UNIX standard function definitions
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <termios.h> // POSIX terminal control definitions


// int drvAID = 1;
// int drvBID = 2;
PaddleController::PaddleController(const char *devPath, uint32_t baud)
: io(), serial(io, devPath)
{
    // configureSerialComms(devPath);
    size = RX_BUFF_SIZE;
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud));
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
void PaddleController::ReadSerialBuff()
{
    char data[RX_BUFF_SIZE];
    std::memset(data, '\0', sizeof(data));
    std::string received = ReadSerialBuff(data, RX_BUFF_SIZE);
}
std::string PaddleController::ReadSerialBuff(char *data, size_t size)
{
    char c;
    std::string result;

    if(readData.size()>0)//If there is some data from a previous read
    {
        std::istream is(&readData);
        size_t toRead=std::min(readData.size(),size);//How many bytes to read?
        is.read(data,toRead);
        data+=toRead;
        size-=toRead;
        if(size==0) return "test";//If read data was enough, just return
    }
    return "abcde";
    

    // for(;;)
    // {
    //     boost::asio::read(serial, boost::asio::buffer(&c,1));
    //     switch(c)
    //     {
    //         case '\r':
    //             break;
    //         case '\n':
    //             return result;
    //         default:
    //             result+=c;
    //     }
    // }
}


#if defined(NO_BOOST)
void PaddleController::configureSerialComms(const char *devPath)
{
    paddleUSB = open(devPath, O_RDWR | O_NONBLOCK | O_NDELAY);
    struct termios tty;
    struct termios tty_old;
    memset(&tty, 0, sizeof(tty));

    /* Error Handling */
    if (tcgetattr(paddleUSB, &tty) != 0)
    {
        std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    }

    /* Save old tty parameters */
    tty_old = tty;

    /* Set Baud Rate */
    cfsetospeed(&tty, (speed_t)B9600);
    cfsetispeed(&tty, (speed_t)B9600);

    /* Setting other Port Stuff */
    tty.c_cflag &= ~PARENB; // Make 8n1
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    tty.c_cflag &= ~CRTSCTS;       // no flow control
    tty.c_cc[VMIN] = 1;            // read doesn't block
    tty.c_cc[VTIME] = 5;           // 0.5 seconds read timeout
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines

    /* Make raw */
    cfmakeraw(&tty);

    /* Flush Port, then applies attributes */
    tcflush(paddleUSB, TCIFLUSH);
    if (tcsetattr(paddleUSB, TCSANOW, &tty) != 0)
    {
        std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }
}

void PaddleController::ReadSerialBuff()
{
    int n = 0, spot = 0;
    char buf = '\0';

    /* Whole response*/
    char response[RX_BUFF_SIZE];
    memset(response, '\0', sizeof(response));

    do
    {
        n = read(paddleUSB, &buf, 1);
        sprintf(&response[spot], "%c", buf);
        spot += n;
    } while (buf != '\r' && n > 0);
    if (n > 0)
    {
        std::cout << "Response: " << response << std::endl;
        parseReceived(response);
    }
    // if (n < 0)
    // {
    //     std::cout << "Error reading: " << strerror(errno) << std::endl;
    // }
    // else if (n == 0)
    // {
    //     std::cout << "Read nothing!" << std::endl;
    // }
    // else
    // {
    //     std::cout << "Response: " << response << std::endl;
    //     parseReceived(response);
    // }
}

#endif
void PaddleController::parseReceived(char *buf)
{
    // size_t pos = 0;
    uint32_t firstFour = *buf;

    int a = 5;
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
