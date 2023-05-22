#include "KincoDriver.h"
#include <thread>
#include <vector>
#include <string>
#include <boost/asio.hpp>

class PaddleController
{
private:
    
    double ElMotorACommand;
    double ElMotorBCommand;
    double AzMotorACommand;
    double AzMotorBCommand;
    int paddleUSB;
    std::string logPathStr;

    KincoDriver *pElDriveA;
    KincoDriver *pElDriveB;
    KincoDriver *pAzDriveA;
    KincoDriver *pAzDriveB;
    #if defined(LFAST_TERMINAL)
    TerminalInterface *terminal;
    #endif
    bool connected;

    void updateCommands();
    void configureSerialComms(const char *devPath);
    void parseReceived(char* buf);


    boost::asio::io_service io;
    boost::asio::serial_port serial;
    boost::asio::streambuf readData; ///< Holds eventual read but not consumed
    size_t size;
    char *dataBuff; ///< Pointer to data array (valid if fixedSize=true)
public:
    PaddleController(const char* devPath, uint32_t baud);
    void ReadSerialBuff();
    std::string ReadSerialBuff(char *data, size_t size);

    #if defined(LFAST_TERMINAL)
    void setupTerminal(TerminalInterface *);
    #endif
    bool connectToDrivers(const char *);
    bool configureDrivers();
    void shutdown();
    std::thread commandUpdate()
    {
        return std::thread([=]
                           { updateCommands(); });
    }

    bool testComplete();
};