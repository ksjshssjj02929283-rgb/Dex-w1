#include "sri_force_sensor.h"
#include <pthread.h>

//====================================
// CSRICommCircularBuffer Implementation
//====================================

CSRICommCircularBuffer::CSRICommCircularBuffer()
	: mBuffer(nullptr), mBufferSize(0), mWIndex(0), mRIndex(0)
{
}

CSRICommCircularBuffer::~CSRICommCircularBuffer()
{
	delete[] mBuffer;
}

bool CSRICommCircularBuffer::Init(int bufferMaxSize)
{
	if (bufferMaxSize <= 0)
	{
		return false;
	}
	mWIndex = 0;
	mRIndex = 0;
	mBufferSize = bufferMaxSize;
	mBuffer = new BYTE[mBufferSize];
	return true;
}

int CSRICommCircularBuffer::GetLength()
{
	int wIndex = mWIndex;
	int rIndex = mRIndex;

	if (wIndex == rIndex)
	{
		return 0;
	}
	else if (wIndex > rIndex)
	{
		return wIndex - rIndex;
	}
	else
	{
		return mBufferSize - rIndex + wIndex;
	}
}

int CSRICommCircularBuffer::GetLength(int& wIndex, int& rIndex)
{
	wIndex = mWIndex;
	rIndex = mRIndex;

	if (wIndex == rIndex)
	{
		return 0;
	}
	else if (wIndex > rIndex)
	{
		return wIndex - rIndex;
	}
	else
	{
		return mBufferSize - rIndex + wIndex;
	}
}

bool CSRICommCircularBuffer::Clear()
{
	mWIndex = 0;
	mRIndex = 0;
	return true;
}

bool CSRICommCircularBuffer::Clear(int clearLen)
{
	if (clearLen < 0)
	{
		return false;
	}
	if (clearLen == 0)
	{
		return true;
	}
	int wIndex = 0;
	int rIndex = 0;
	int length = GetLength(wIndex, rIndex);
	if (clearLen > length)
	{
		return Clear();
	}
	mRIndex = (rIndex + clearLen) % mBufferSize;
	return true;
}

int CSRICommCircularBuffer::Write(BYTE* data, int dataLen)
{
	if (dataLen <= 0)
	{
		return 0;
	}
	int len = (dataLen > mBufferSize) ? mBufferSize : dataLen;

	int wIndex = 0;
	int rIndex = 0;
	int length = GetLength(wIndex, rIndex);
	int spaceLen = mBufferSize - length - 1;
	if (len > spaceLen)
	{
		return 0;
	}

	if (wIndex + len <= mBufferSize)
	{
		memcpy(mBuffer + wIndex, data, len);
		mWIndex = (wIndex + len) % mBufferSize;
	}
	else
	{
		int firstBlockLen = mBufferSize - wIndex;
		int secondBlockLen = len - firstBlockLen;
		memcpy(mBuffer + wIndex, data, firstBlockLen);
		memcpy(mBuffer, data + firstBlockLen, secondBlockLen);
		mWIndex = (wIndex + len) % mBufferSize;
	}
	return len;
}

int CSRICommCircularBuffer::Write(BYTE data)
{
	int wIndex = 0;
	int rIndex = 0;
	int length = GetLength(wIndex, rIndex);
	int spaceLen = mBufferSize - length - 1;
	if (1 > spaceLen)
	{
		return 0;
	}

	mBuffer[wIndex] = data;
	mWIndex = (wIndex + 1) % mBufferSize;
	return 1;
}

BYTE* CSRICommCircularBuffer::Read(int& dataLen, int readLen, bool delData)
{
	int wIndex = 0;
	int rIndex = 0;
	int length = GetLength(wIndex, rIndex);

	int len = (readLen <= 0) ? length : min(length, readLen);
	if (len <= 0)
	{
		return nullptr;
	}

	BYTE* data = new BYTE[len];
	dataLen = len;

	if (rIndex + len <= mBufferSize)
	{
		memcpy(data, mBuffer + rIndex, len);
		if (delData)
		{
			mRIndex = (rIndex + len) % mBufferSize;
		}
	}
	else
	{
		int firstBlockLen = mBufferSize - rIndex;
		int secondBlockLen = len - firstBlockLen;
		memcpy(data, mBuffer + rIndex, firstBlockLen);
		memcpy(data + firstBlockLen, mBuffer, secondBlockLen);
		if (delData)
		{
			mRIndex = (rIndex + len) % mBufferSize;
		}
	}
	return data;
}

BYTE* CSRICommCircularBuffer::ReadTry(int& dataLen, int readLen)
{
	return Read(dataLen, readLen, false);
}


//====================================
// CSRICommM8218Parser Implementation
//====================================

CSRICommM8218Parser::CSRICommM8218Parser()
{
	mCircularBuffer.Init(102400);
}

CSRICommM8218Parser::~CSRICommM8218Parser()
{
}

bool CSRICommM8218Parser::SetM8218CallbackFunction(SRICommM8218CallbackFunction m8218CallbackFunction)
{
	mM8218CallbackFunction = m8218CallbackFunction;
	return true;
}

bool CSRICommM8218Parser::OnReceivedData(BYTE* data, int dataLen)
{
	if (data == nullptr || dataLen <= 0)
	{
		return false;
	}
	mCircularBuffer.Write(data, dataLen);

	int delLen = 0;
	float fx, fy, fz, mx, my, mz;
	if (!ParseDataFromBuffer(delLen, fx, fy, fz, mx, my, mz))
	{
		mCircularBuffer.Clear(delLen);
		return false;
	}
	mCircularBuffer.Clear(delLen);

	if (mM8218CallbackFunction)
	{
		mM8218CallbackFunction(fx, fy, fz, mx, my, mz);
	}
	return true;
}

bool CSRICommM8218Parser::OnNetworkFailure(std::string infor)
{
	return true;
}

bool CSRICommM8218Parser::ParseDataFromBuffer(int& delLen, float& fx, float& fy, float& fz, float& mx, float& my, float& mz)
{
	delLen = 0;
	int dataLen = 0;
	BYTE* data = mCircularBuffer.ReadTry(dataLen);
	if (data == nullptr)
	{
		return false;
	}

	int headIndex = ParseGetHeadIndex(data, dataLen);
	if (headIndex == -1 || headIndex + 12 > dataLen)
	{
		delLen = (headIndex == -1) ? dataLen - 1 : headIndex;
		return false;
	}

	int index = headIndex + 2;
	byte buf[11];
	memcpy(&buf, data + index, 11);

	INT32 forceInt32[6];
	forceInt32[0] = (buf[0] << 9) + (buf[1] << 1) + ((buf[2] & 0x80) >> 7);
	forceInt32[1] = ((buf[2] & 0x7F) << 10) + ((buf[3] & 0xFF) << 2) + ((buf[4] & 0xC0) >> 6);
	forceInt32[2] = ((buf[4] & 0x3F) << 11) + ((buf[5] & 0xFF) << 3) + ((buf[6] & 0xE0) >> 5);
	forceInt32[3] = ((buf[6] & 0x1F) << 7) + ((buf[7] & 0xFE) >> 1);
	forceInt32[4] = ((buf[7] & 0x01) << 11) + ((buf[8] & 0xFF) << 3) + ((buf[9] & 0xE0) >> 5);
	forceInt32[5] = ((buf[9] & 0x1F) << 7) + ((buf[10] & 0xFE) >> 1);

	index = index + 11;
	BYTE crc = data[index];
	BYTE calCrc = MyCRC_GetCRC8(buf, 11);
	delLen = index;
	if (crc != calCrc)
	{
		return false;
	}

	float f[6];
	for (int i = 0; i < 3; i++)
	{
		f[i] = (forceInt32[i] & 0x10000) ? -(float)((forceInt32[i] & 0xffff) / 10.0)
										 : (float)((forceInt32[i] & 0xffff) / 10.0);
	}
	for (int i = 3; i < 6; i++)
	{
		f[i] = (forceInt32[i] & 0x0800) ? -(float)((forceInt32[i] & 0x07ff) / 10.0)
										: (float)((forceInt32[i] & 0x07ff) / 10.0);
	}
	fx = f[0];
	fy = f[1];
	fz = f[2];
	mx = f[3];
	my = f[4];
	mz = f[5];
	return true;
}

uint8_t CSRICommM8218Parser::MyCRC_GetCRC8(uint8_t* pDat, uint8_t bytes)
{
	uint8_t CRC8_POLY = 0x8C;
	uint8_t crc = 0;
	while (bytes--)
	{
		crc ^= *pDat;
		for (int i = 0; i < 8; i++)
		{
			if (crc & 0x01)
			{
				crc = (crc >> 1) ^ CRC8_POLY;
			}
			else
			{
				crc >>= 1;
			}
		}
		pDat++;
	}
	return crc;
}

int CSRICommM8218Parser::ParseGetHeadIndex(BYTE* data, int dataLen)
{
	int headIndex = -1;
	int countHead = 0;
	for (int i = 0; i < dataLen - 1; i++)
	{
		if (data[i] == 0xAA && data[i + 1] == 0x55)
		{
			headIndex = i;
			countHead++;
		}
	}

	if (headIndex + 12 > dataLen && countHead > 1)
	{
		int RcountHead = 0;
		for (int i = 0; i < dataLen - 1; i++)
		{
			if (data[i] == 0xAA && data[i + 1] == 0x55)
			{
				if (++RcountHead == countHead - 1)
				{
					return i;
				}
			}
		}
	}
	return headIndex;
}


//====================================
// SerialManeger_linux Implementation
//====================================

void* ReadSerialDate(void* para)
{
	auto* p_serial_test = static_cast<SerialManeger_linux*>(para);
	printf("ReadSerialDate Thread Start\n");
	while (true)
	{
		p_serial_test->Read();
		usleep(100);
	}
	printf("Receive thread break!\n");
	pthread_detach(pthread_self());
	pthread_exit(nullptr);
	return nullptr;
}

SerialManeger_linux::SerialManeger_linux()
	: mParser(nullptr), m_serial_fd(-1), mReadBuffer(nullptr)
{
	mReadBuffer = new unsigned char[REC_DATA_LEN];
}

SerialManeger_linux::~SerialManeger_linux()
{
	delete[] mReadBuffer;
}

int SerialManeger_linux::Read()
{
	if (m_serial_fd < 0) return -1;

	int recvDataLen = ReadData(m_serial_fd, mReadBuffer, REC_DATA_LEN);
	if (recvDataLen > 0)
	{
		OnReceivedData(mReadBuffer, recvDataLen);
		return 0;
	}
	return -1;
}

bool SerialManeger_linux::OnReceivedData(BYTE* data, int dataLen)
{
	if (mParser != nullptr)
	{
		mParser->OnReceivedData(data, dataLen);
	}
	return true;
}

bool SerialManeger_linux::AddCommParser(CSRICommM8218Parser* parser)
{
	mParser = parser;
	return true;
}

int SerialManeger_linux::OpenSerialPort(int index)
{
	m_serial_fd = OpenPort(index);
	if (m_serial_fd < 0)
	{
		printf("Open serial port %d failed!\n", index);
		return -1;
	}
	printf("Open serial port %d Success\n", m_serial_fd);
	return 0;
}

int SerialManeger_linux::OpenSerialPort(const char* device)
{
	m_serial_fd = OpenPort(device);
	if (m_serial_fd < 0)
	{
		printf("Open serial port %s failed!\n", device);
		return -1;
	}
	printf("Open serial port %d Success\n", m_serial_fd);
	return 0;
}

int SerialManeger_linux::SerialRun(int BaudRate)
{
	SetPara(m_serial_fd, BaudRate);
	if (pthread_create(&m_rec_threadid, nullptr, ReadSerialDate, this) != 0)
	{
		printf("creat Receive_Thread failed \n");
		ClosePort(m_serial_fd);
		m_serial_fd = -1;
		return -1;
	}
	return 0;
}

int SerialManeger_linux::ClosePort()
{
	ClosePort(m_serial_fd);
	m_serial_fd = -1;
	return 0;
}

int SerialManeger_linux::Write(const char* data, int datalength)
{
	if (WriteData(m_serial_fd, data, datalength) < 0)
	{
		printf("Write Data Fail!\n");
		return -1;
	}
	return 0;
}

int SerialManeger_linux::BaudRate(int baudrate)
{
	switch (baudrate)
	{
	case 115200: return B115200;
	case 460800: return B460800;
	default:    return B9600;
	}
}

int SerialManeger_linux::SetPara(int serialfd, int speed, int databits, int stopbits, int parity)
{
	struct termios tty;
	bzero(&tty, sizeof(tty));
	cfmakeraw(&tty);
	tty.c_cflag = BaudRate(speed) | CLOCAL | CREAD;
	tty.c_cflag &= ~CSIZE;

	switch (databits)
	{
	case 5: tty.c_cflag |= CS5; break;
	case 6: tty.c_cflag |= CS6; break;
	case 7: tty.c_cflag |= CS7; break;
	default: tty.c_cflag |= CS8; break;
	}

	switch (parity)
	{
	case 1: tty.c_cflag |= PARENB; tty.c_cflag &= ~PARODD; break;
	case 2: tty.c_cflag |= PARENB | PARODD; break;
	default: tty.c_cflag &= ~PARENB; break;
	}

	switch (stopbits)
	{
	case 2: tty.c_cflag |= CSTOPB; break;
	default: tty.c_cflag &= ~CSTOPB; break;
	}

	tcflush(serialfd, TCIFLUSH);
	tty.c_cc[VTIME] = 1;
	tty.c_cc[VMIN] = 1;
	return tcsetattr(serialfd, TCSANOW, &tty);
}

int SerialManeger_linux::WriteData(int fd, const char* data, int datalength)
{
	if (fd < 0) return -1;

	int len = 0, total_len = 0;
	while (total_len < datalength)
	{
		len = write(fd, &data[total_len], datalength - total_len);
		if (len > 0)
		{
			total_len += len;
		}
		else
		{
			return -1;
		}
	}
	return total_len;
}

int SerialManeger_linux::ReadData(int fd, unsigned char* data, int datalength)
{
	if (fd < 0) return -1;

	fd_set readset;
	FD_ZERO(&readset);
	FD_SET(fd, &readset);

	struct timeval tv = {0, 1000};
	if (select(fd + 1, &readset, nullptr, nullptr, &tv) <= 0)
	{
		return 0;
	}
	return FD_ISSET(fd, &readset) ? read(fd, data, datalength) : 0;
}

void SerialManeger_linux::ClosePort(int fd)
{
	if (fd > 0)
	{
		::close(fd);
	}
}

int SerialManeger_linux::OpenPort(int index)
{
	const char* device;
	switch (index)
	{
	case ttyUSB1: device = "/dev/ttyUSB1"; break;
	case ttyUSB2: device = "/dev/ttyUSB2"; break;
	default:      device = "/dev/ttyUSB0"; break;
	}
	return OpenPort(device);
}

int SerialManeger_linux::OpenPort(const char* device)
{
	int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
	return fd;
}
