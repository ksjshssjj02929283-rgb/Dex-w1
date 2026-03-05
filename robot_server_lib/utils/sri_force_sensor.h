#ifndef SRI_FORCE_SERVER_H
#define SRI_FORCE_SERVER_H

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/signal.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

typedef std::function<bool(float fx, float fy, float fz, float mx, float my, float mz)> SRICommM8218CallbackFunction;

typedef unsigned char BYTE;

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef INT16
typedef short INT16;
#endif

#ifndef INT32
typedef int INT32;
#endif

#ifndef INT64
typedef long long INT64;
#endif

#ifndef byte
typedef unsigned char byte;
#endif

#define REC_DATA_LEN 8192

enum { ttyUSB0 = 0, ttyUSB1, ttyUSB2 };

class CSRICommCircularBuffer {
public:
    CSRICommCircularBuffer();
    ~CSRICommCircularBuffer();

    bool Init(int bufferMaxSize = 10240);
    int GetLength();
    int GetLength(int& wIndex, int& rIndex);
    bool Clear();
    bool Clear(int clearLen);

    int Write(BYTE* data, int dataLen);
    int Write(BYTE data);

    BYTE* Read(int& dataLen, int readLen = 0, bool delData = true);
    BYTE* ReadTry(int& dataLen, int readLen = 0);

private:
    BYTE* mBuffer;
    int mBufferSize;
    int mWIndex;
    int mRIndex;
};

class CSRICommM8218Parser {
public:
    CSRICommM8218Parser();
    ~CSRICommM8218Parser();

    bool SetM8218CallbackFunction(SRICommM8218CallbackFunction m8218CallbackFunction);

    bool OnReceivedData(BYTE* data, int dataLen);
    bool OnNetworkFailure(std::string infor);

private:
    CSRICommCircularBuffer mCircularBuffer;
    SRICommM8218CallbackFunction mM8218CallbackFunction;

    bool ParseDataFromBuffer(int& delLen, float& fx, float& fy, float& fz, float& mx, float& my, float& mz);
    int ParseGetHeadIndex(BYTE* data, int dataLen);
    uint8_t MyCRC_GetCRC8(uint8_t* pDat, uint8_t bytes);
};

class SerialManeger_linux {
public:
    SerialManeger_linux();
    ~SerialManeger_linux();
    int SerialRun(int BaudRate);
    int OpenSerialPort(int index);
    int OpenSerialPort(const char* device);

    int Write(const char* data, int datalength);
    int Read();
    int ClosePort();

    bool AddCommParser(CSRICommM8218Parser* parser);
    bool OnReceivedData(BYTE* data, int dataLen);

    int OpenPort(int index);
    int OpenPort(const char* device);
    int SetPara(int serialfd, int speed = 2, int databits = 8, int stopbits = 2, int parity = 1);
    int WriteData(int fd, const char* data, int datalength);
    int ReadData(int fd, unsigned char* data, int datalength);
    void ClosePort(int fd);
    int BaudRate(int baudrate);

private:
    CSRICommM8218Parser* mParser;
    int m_serial_fd;
    pthread_t m_rec_threadid;
    unsigned char* mReadBuffer;
};

#endif
