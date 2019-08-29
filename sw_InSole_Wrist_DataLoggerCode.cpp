// Data logger code to run on ODROID board
// 3 Nodes: LShoe, RShoe, RWrist
// With Broadcast Reference Time Synchronization

// INCLUDE LIBRARIES
#include <iostream>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <algorithm>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/ioctl.h>

#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/time.h>
#include <time.h>  
#include <fcntl.h>
#include <unistd.h>

#include <math.h>
#include <string.h>
#include <inttypes.h>

// DEFINE GLOBAL VARIABLES
#define US_CYCLES 1000 //[us]

#define BUFFER 4096
#define PACKET_LENGTH_WIFI  51 // Length of the sole data packet
#define PACKET_LENGTH_LOG   129 // Length of the log data packet
#define PACKET_LENGTH_PD    24 // Not used
#define PACKET_LENGTH_WRIST 29 // Length of the wrist data packet
#define PACKET_LENGTH_LED    8
#define PACKET_LENGTH_TIME   16 // Length of the broadcast data packet
#define PACKET_LENGTH_SYNC   7
#define N_PACKET_LED 20
#define RADTODEG 57.295780
#define FACTOR_SCALE_ANGLE 5000.0f
#define FACTOR_SCALE_ACCELERATION 1000.0f
#define FACTOR_SCALE_ACC_ADXL 128.0f
#define MAX_UINT16_FLOAT 65536.0f
#define MAX_UINT16 65536
#define L1_NORM_ACC_SCALE 4.0f

#define N_STR 512
#define N_MAX_MODES 50
#define N_PRESS_SENS 4

#define RESET_PORT 3461
//#define SYNC_PORT 3462
#define PRESSUREVAL_PORT 3463
#define PACKET_LENGTH_RESET_LED 6
#define PACKET_LENGTH_PRESSUREVAL 8
#define PACKET_LENGTH_MATLAB 7

#define ADXL345_LSB_G_2G            256.0f
#define ADXL345_LSB_G_4G            128.0f
#define ADXL345_LSB_G_8G             64.0f

#define TIME_TO_WAIT_US 11000000 // wait 11s before starting the metronome
using namespace std;

mutex dataMutex;

// DATA STRUCTURES
struct structDataPacketPureData 
{
	uint32_t timestamp;
		
	float yaw1,pitch1,roll1;
	float ax1,ay1,az1;
	uint16_t p1,p2,p3,p4,p5,p6,p7,p8;

	uint64_t Odroid_Timestamp;
	uint8_t Odroid_Trigger;
	uint32_t timestamp2;
};

struct structDataPacketPureDataRAW
{
	uint32_t timestamp;
		
	int16_t yaw1,pitch1,roll1;
	int16_t ax1,ay1,az1;
	uint16_t p1,p2,p3,p4,p5,p6,p7,p8;

	uint64_t Odroid_Timestamp;
	uint8_t Odroid_Trigger;
	uint32_t timestamp2;
};

struct structDataPacketPureDataWrist
{
	uint32_t timestamp;

	float ax, ay, az;

	uint64_t Odroid_Timestamp;
	uint8_t Odroid_Trigger;
	uint32_t timestamp2;
};

struct structDataPacketPureDataRAWWrist
{
	uint32_t timestamp;

	int16_t ax, ay, az;

	uint64_t Odroid_Timestamp;
	uint8_t Odroid_Trigger;
	uint32_t timestamp2;
};

struct structDataPacket 
{
	uint32_t timestamp;
		
	float qw1,qx1,qy1,qz1;
	float yaw1,pitch1,roll1;
	float ax1,ay1,az1;
	
	uint16_t p1,p2,p3,p4,p5,p6,p7,p8;
	uint16_t range1, range2, range3;
	uint64_t Odroid_Timestamp;
	uint8_t Odroid_Trigger;
	uint32_t timestamp2;
};

struct structSyncPacket 
{
	uint8_t Ext_Trigger;
};


struct structSWstat 
{
	unsigned int packetLedSent;
	unsigned int packetGuiSent;
	unsigned int packetPdSent;
	unsigned int packetErrorPdShoeL;
	unsigned int packetErrorPdShoeR;
	unsigned int packetErrorWristR;
	unsigned int packetErrorSync;
	unsigned int packetBroadSent;
	unsigned int packetReceivedPdShoeL;
	unsigned int packetReceivedPdShoeR;
	unsigned int packetReceivedWristR;
	unsigned int packetReceivedSync;
};

inline void reconstructStruct(structDataPacketPureDataRAW dataPacketRAW, structDataPacketPureData &dataPacket)
{
	dataPacket.timestamp=dataPacketRAW.timestamp;
 
	dataPacket.yaw1=((float)dataPacketRAW.yaw1)/FACTOR_SCALE_ANGLE;
	dataPacket.pitch1=((float)dataPacketRAW.pitch1)/FACTOR_SCALE_ANGLE;
	dataPacket.roll1=((float)dataPacketRAW.roll1)/FACTOR_SCALE_ANGLE;

	dataPacket.ax1=((float)dataPacketRAW.ax1)/FACTOR_SCALE_ACCELERATION;
	dataPacket.ay1=((float)dataPacketRAW.ay1)/FACTOR_SCALE_ACCELERATION;
	dataPacket.az1=((float)dataPacketRAW.az1)/FACTOR_SCALE_ACCELERATION;

	dataPacket.p1=dataPacketRAW.p1;
	dataPacket.p2=dataPacketRAW.p2;
	dataPacket.p3=dataPacketRAW.p3;
	dataPacket.p4=dataPacketRAW.p4;
	dataPacket.p5=dataPacketRAW.p5;
	dataPacket.p6=dataPacketRAW.p6;
	dataPacket.p7=dataPacketRAW.p7;
	dataPacket.p8=dataPacketRAW.p8;
	
	dataPacket.Odroid_Timestamp=dataPacketRAW.Odroid_Timestamp;
	dataPacket.Odroid_Trigger=dataPacketRAW.Odroid_Trigger;

	dataPacket.timestamp2=dataPacketRAW.timestamp2;
}

inline void reconstructStructWrist(structDataPacketPureDataRAWWrist dataPacketRAW, structDataPacketPureDataWrist &dataPacket)
{
	dataPacket.timestamp = dataPacketRAW.timestamp;

	dataPacket.ax = ((float)dataPacketRAW.ax) / ADXL345_LSB_G_4G;
	dataPacket.ay = ((float)dataPacketRAW.ay) / ADXL345_LSB_G_4G;
	dataPacket.az = ((float)dataPacketRAW.az) / ADXL345_LSB_G_4G;

	dataPacket.Odroid_Timestamp = dataPacketRAW.Odroid_Timestamp;
	dataPacket.Odroid_Trigger = dataPacketRAW.Odroid_Trigger;

	dataPacket.timestamp2 = dataPacketRAW.timestamp2;
}

inline void reconstructStructSyncPacket(uint8_t* recvbuffer,structSyncPacket  &dataPacket)
{
	uint8_t *pointer;
	
	//recvbuffer[0];
	//recvbuffer[1];
	//recvbuffer[2];	
	
	pointer=(uint8_t*)&dataPacket.Ext_Trigger;
	pointer[0]=recvbuffer[3];
	
	//recvbuffer[4];
	//recvbuffer[5];
	//recvbuffer[6];
}


inline void reconstructStructPureDataRAW(uint8_t* recvbuffer,structDataPacketPureDataRAW &dataPacket)
{
	uint8_t *pointer;

	// HEAD
	//recvbuffer[0];
	//recvbuffer[1];
	//recvbuffer[2];

	pointer=(uint8_t*)&dataPacket.timestamp;
	pointer[3]=recvbuffer[3];
	pointer[2]=recvbuffer[4];
	pointer[1]=recvbuffer[5];
	pointer[0]=recvbuffer[6];

	pointer=(uint8_t*)&dataPacket.yaw1;
	pointer[1]=recvbuffer[7];
	pointer[0]=recvbuffer[8];

	pointer=(uint8_t*)&dataPacket.pitch1;
	pointer[1]=recvbuffer[9];
	pointer[0]=recvbuffer[10];

	pointer=(uint8_t*)&dataPacket.roll1;
	pointer[1]=recvbuffer[11];
	pointer[0]=recvbuffer[12];

	pointer=(uint8_t*)&dataPacket.ax1;
	pointer[1]=recvbuffer[13];
	pointer[0]=recvbuffer[14];

	pointer=(uint8_t*)&dataPacket.ay1;
	pointer[1]=recvbuffer[15];
	pointer[0]=recvbuffer[16];

	pointer=(uint8_t*)&dataPacket.az1;
	pointer[1]=recvbuffer[17];
	pointer[0]=recvbuffer[18];

	/////////////////////////////////////////////

	pointer=(uint8_t*)&dataPacket.p1;
	pointer[1]=recvbuffer[19];
	pointer[0]=recvbuffer[20];

	pointer=(uint8_t*)&dataPacket.p2;
	pointer[1]=recvbuffer[21];
	pointer[0]=recvbuffer[22];

	pointer=(uint8_t*)&dataPacket.p3;
	pointer[1]=recvbuffer[23];
	pointer[0]=recvbuffer[24];

	pointer=(uint8_t*)&dataPacket.p4;
	pointer[1]=recvbuffer[25];
	pointer[0]=recvbuffer[26];

	pointer=(uint8_t*)&dataPacket.p5;
	pointer[1]=recvbuffer[27];
	pointer[0]=recvbuffer[28];

	pointer=(uint8_t*)&dataPacket.p6;
	pointer[1]=recvbuffer[29];
	pointer[0]=recvbuffer[30];

	pointer=(uint8_t*)&dataPacket.p7;
	pointer[1]=recvbuffer[31];
	pointer[0]=recvbuffer[32];

	pointer=(uint8_t*)&dataPacket.p8;
	pointer[1]=recvbuffer[33];
	pointer[0]=recvbuffer[34];

	/////////////////////////////////////////////

    //Timestamp_Odroid
	pointer=(uint8_t*)&dataPacket.Odroid_Timestamp;
	pointer[7]=recvbuffer[35];
	pointer[6]=recvbuffer[36];
	pointer[5]=recvbuffer[37];
	pointer[4]=recvbuffer[38];
	pointer[3]=recvbuffer[39];
	pointer[2]=recvbuffer[40];
	pointer[1]=recvbuffer[41];
	pointer[0]=recvbuffer[42];

	// trigger
	pointer=(uint8_t*)&dataPacket.Odroid_Trigger;
	pointer[0]=recvbuffer[43];

	// timestamp2
	pointer=(uint8_t*)&dataPacket.timestamp2;
	pointer[3]=recvbuffer[44];
	pointer[2]=recvbuffer[45];
	pointer[1]=recvbuffer[46];
	pointer[0]=recvbuffer[47];

	// TAIL
	//recvbuffer[48];
	//recvbuffer[49];
	//recvbuffer[50];
}

inline void reconstructStructPureDataRAWWrist(uint8_t* recvbuffer, structDataPacketPureDataRAWWrist &dataPacket)
{
	uint8_t *pointer;

	// HEAD
	//recvbuffer[0];
	//recvbuffer[1];
	//recvbuffer[2];

	pointer = (uint8_t*)&dataPacket.timestamp;
	pointer[3] = recvbuffer[3];
	pointer[2] = recvbuffer[4];
	pointer[1] = recvbuffer[5];
	pointer[0] = recvbuffer[6];

	pointer = (uint8_t*)&dataPacket.ax;
	pointer[1] = recvbuffer[7];
	pointer[0] = recvbuffer[8];

	pointer = (uint8_t*)&dataPacket.ay;
	pointer[1] = recvbuffer[9];
	pointer[0] = recvbuffer[10];

	pointer = (uint8_t*)&dataPacket.az;
	pointer[1] = recvbuffer[11];
	pointer[0] = recvbuffer[12];

	/////////////////////////////////////////////

	//Timestamp_Odroid
	pointer = (uint8_t*)&dataPacket.Odroid_Timestamp;
	pointer[7] = recvbuffer[13];
	pointer[6] = recvbuffer[14];
	pointer[5] = recvbuffer[15];
	pointer[4] = recvbuffer[16];
	pointer[3] = recvbuffer[17];
	pointer[2] = recvbuffer[18];
	pointer[1] = recvbuffer[19];
	pointer[0] = recvbuffer[20];

	// trigger
	pointer = (uint8_t*)&dataPacket.Odroid_Trigger;
	pointer[0] = recvbuffer[21];

	// timestamp2
	pointer=(uint8_t*)&dataPacket.timestamp2;
	pointer[3] = recvbuffer[22];
	pointer[2] = recvbuffer[23];
	pointer[1] = recvbuffer[24];
	pointer[0] = recvbuffer[25];

	// TAIL
	//recvbuffer[26];
	//recvbuffer[27];
	//recvbuffer[28];
}

inline uint16_t convert2uint16(float fValue,float scale)
{
	uint32_t iValue=(uint32_t)(MAX_UINT16_FLOAT*(fValue)/scale);	
	if (iValue>MAX_UINT16)
	{
		iValue=MAX_UINT16;
	}	
	return (uint16_t) iValue;	
}

struct structPDShoe
{
	string ipAddress;
	string name;
	int port;
	uint8_t lastPacket[PACKET_LENGTH_WIFI];
	unsigned int packetError;
	unsigned int packetReceived;
	uint8_t id;
};

struct structWrist
{
	string ipAddress;
	string name;
	int port;
	uint8_t lastPacket[PACKET_LENGTH_WRIST];
	unsigned int packetError;
	unsigned int packetReceived;
	uint8_t id;
};

struct structSync
{
	string ipAddress;
	string name;
	int port;
	uint8_t lastPacket[PACKET_LENGTH_SYNC];
	unsigned int packetError;
	unsigned int packetReceived;
	uint8_t id;
};

struct structSensSettings{
	uint16_t maxUpToNow = 0;
	uint16_t maxInUse = 1023;
	uint16_t minUpToNow = 1023;
	uint16_t minInUse = 0;
	uint16_t perMilleThreshold = 400;	
};

struct structPressSettings
{
	structSensSettings front;
	structSensSettings heel;
};

// BOOLEAN FUNCTION TO CHECK THE PACKETS
// CHECK SOLE PACKET
inline bool checkPacket(uint8_t *buffer,int ret)
{
	return (buffer[0]==0x07 && buffer[1]==0x08 && buffer[2]==0x09 && buffer[48]==0xA && buffer[49]==0xB && buffer[50]==0xC && ret==PACKET_LENGTH_WIFI);
};

// CHECK WRIST PACKET
inline bool checkPacketWrist(uint8_t *buffer, int ret)
{
	return (buffer[0] == 0x07 && buffer[1] == 0x08 && buffer[2] == 0x09 && buffer[26] == 0xA && buffer[27] == 0xB && buffer[28] == 0xC && ret == PACKET_LENGTH_WRIST);
};

// NOT USED
inline bool checkResetPacket(uint8_t *buffer,int ret)
{
	return (buffer[0]==0x01 && buffer[1]==0x02 && buffer[2]==0x03 && buffer[3]==0x4 && buffer[4]==0x5 && buffer[5]==0x6 && ret==PACKET_LENGTH_RESET_LED);
};

// NOT USED
inline uint8_t checkMatlabIncomingPacket(uint8_t *buffer,int ret)
{
	uint8_t integrityCheck =(uint8_t)(buffer[0]==0x01 && buffer[1]==0x02 && buffer[2]==0x03 && buffer[4]==0x4 && buffer[5]==0x5 && buffer[6]==0x6 && ret==PACKET_LENGTH_MATLAB);
	return (integrityCheck * buffer[3]);
};

// NOT USED
inline bool checkSyncPacket(uint8_t *buffer,int ret)
{
	return (buffer[0]==0x01 && buffer[1]==0x02 && buffer[2]==0x03 && buffer[4]==0x4 && buffer[5]==0x5 && buffer[6]==0x6 && ret==PACKET_LENGTH_SYNC);
};


void threadSYNCreceive(structSync* SyncBoard)
{
	dataMutex.lock();
	
	printf("Hello from Thread! [%s ip: %s - port: %d]\n",SyncBoard->name.c_str(),SyncBoard->ipAddress.c_str(),SyncBoard->port);

	//unsigned int nReset=0;
	uint8_t id;
	int sockfdServer;
	socklen_t len;
	int ret;
	uint8_t recvBuffer[BUFFER];
	
	struct sockaddr_in addrServer;
	struct sockaddr_in addrClient;
	
	sockfdServer=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
	//printf("sockfdServer=%d\n",sockfdServer);
	
	bzero(&addrServer,sizeof(addrServer));
	addrServer.sin_family = AF_INET;
	addrServer.sin_addr.s_addr=htonl(INADDR_ANY);//IP_ADDRESS(127, 0, 0, 1);
	addrServer.sin_port=htons(SyncBoard->port);
	
	bind(sockfdServer,(struct sockaddr *)&addrServer,sizeof(addrServer));
	
	len = (socklen_t)sizeof(addrClient);
	id=SyncBoard->id;
	
	dataMutex.unlock();
	usleep(250);
		
	while(1)
	{		
		ret=recvfrom(sockfdServer,recvBuffer,PACKET_LENGTH_SYNC,0,(struct sockaddr *)&addrClient,&len);
		
		if (ret>1)
		{			
			dataMutex.lock();
			
			SyncBoard->packetReceived++;
			if (checkSyncPacket(recvBuffer,ret))
			{
				memcpy(SyncBoard->lastPacket,recvBuffer,PACKET_LENGTH_SYNC);
			}
			else
			{
				SyncBoard->packetError++;	
				//printf("Ret=%d\n", ret);
			}			
			dataMutex.unlock();			
		}	
	}
}


void threadUDPreceive(structPDShoe* PDShoe)
{
	dataMutex.lock();
	
	printf("Hello from Thread! [%s ip: %s - port: %d]\n",PDShoe->name.c_str(),PDShoe->ipAddress.c_str(),PDShoe->port);
	
	uint8_t id;
	unsigned long int cycles=0;
	int sockfdServer;
	socklen_t len;
	int ret;
	uint8_t recvBuffer[BUFFER];
	
	struct sockaddr_in addrServer;
	struct sockaddr_in addrClient;
	
	sockfdServer=socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP);
	//printf("sockfdServer=%d\n",sockfdServer);
	
	bzero(&addrServer,sizeof(addrServer));
	addrServer.sin_family = AF_INET;
	addrServer.sin_addr.s_addr=htonl(INADDR_ANY);//IP_ADDRESS(127, 0, 0, 1);
	addrServer.sin_port=htons(PDShoe->port);
	
	bind(sockfdServer,(struct sockaddr *)&addrServer,sizeof(addrServer));
	
	len = (socklen_t)sizeof(addrClient);
	
	id=PDShoe->id;
	dataMutex.unlock();
	usleep(250);
		
	while(1)
	{		
		ret=recvfrom(sockfdServer,recvBuffer,PACKET_LENGTH_WIFI,0,(struct sockaddr *)&addrClient,&len);
		
		if (ret>1)
		{
			//printf("Shoe Data!\n");
			
			dataMutex.lock();
			
			PDShoe->packetReceived++;
			if (checkPacket(recvBuffer,ret))
			{
				memcpy(PDShoe->lastPacket,recvBuffer,PACKET_LENGTH_WIFI);
			}
			else
			{
				PDShoe->packetError++;	
				//printf("Ret=%d\n", ret);
			}			
			dataMutex.unlock();			
		}
		cycles++;
	}
}

void threadUDPreceiveWrist(structWrist* Wrist)
{
	dataMutex.lock();

	printf("Hello from Thread! [%s ip: %s - port: %d]\n", Wrist->name.c_str(), Wrist->ipAddress.c_str(), Wrist->port);

	uint8_t id;
	unsigned long int cycles = 0;
	int sockfdServer;
	socklen_t len;
	int ret;
	uint8_t recvBuffer[BUFFER];

	struct sockaddr_in addrServer;
	struct sockaddr_in addrClient;

	sockfdServer = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	//printf("sockfdServer=%d\n",sockfdServer);

	bzero(&addrServer, sizeof(addrServer));
	addrServer.sin_family = AF_INET;
	addrServer.sin_addr.s_addr = htonl(INADDR_ANY);//IP_ADDRESS(127, 0, 0, 1);
	addrServer.sin_port = htons(Wrist->port);

	bind(sockfdServer, (struct sockaddr *)&addrServer, sizeof(addrServer));

	len = (socklen_t)sizeof(addrClient);

	id = Wrist->id;
	dataMutex.unlock();
	usleep(250);

	while (1)
	{
		ret = recvfrom(sockfdServer, recvBuffer, PACKET_LENGTH_WRIST, 0, (struct sockaddr *)&addrClient, &len);

		if (ret>1)
		{
			//printf("Wrist Data!\n");

			dataMutex.lock();

			Wrist->packetReceived++;
			if (checkPacketWrist(recvBuffer, ret))
			{
				memcpy(Wrist->lastPacket, recvBuffer, PACKET_LENGTH_WRIST);
			}
			else
			{
				Wrist->packetError++;
				//printf("Ret=%d\n", ret);
			}
			dataMutex.unlock();
		}
		cycles++;
	}
}

// FUNCTION TO CREATE LOG DATA PACKET
void createLogPacket_3nodes(uint8_t* buffer_out,uint8_t* buffer_01,uint8_t* buffer_02, uint8_t* buffer_03,uint8_t Odroid_Trigger,uint64_t currenttime,uint8_t Ext_Trigger)
{
	uint8_t *pointer;

	buffer_out[0]=0x01;
	buffer_out[1]=0x02;
	buffer_out[2]=0x03;

	// Put data of the LShoe and RShoe to the log packet
	for(int i=0;i<45;i++)
	{
		buffer_out[i+3]=buffer_01[i+3];
		buffer_out[i+48]=buffer_02[i+3];
	}

	// Put data of the RWrist to the log packet
	for (int i = 0; i<23; i++)
	{
		buffer_out[i + 93] = buffer_03[i + 3];
	}

	// ODROID current time
	pointer=(uint8_t*)&currenttime;
	buffer_out[116]=pointer[7];
	buffer_out[117]=pointer[6];
	buffer_out[118]=pointer[5];
	buffer_out[119]=pointer[4];
	buffer_out[120]=pointer[3];
	buffer_out[121]=pointer[2];
	buffer_out[122]=pointer[1];
	buffer_out[123]=pointer[0];

    buffer_out[124]=Odroid_Trigger;
	
	buffer_out[125]=Ext_Trigger;

	buffer_out[126]=0x4;
	buffer_out[127]=0x5;
	buffer_out[128]=0x6;
}

uint64_t getMicrosTimeStamp() 
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_sec*(uint64_t)1000000+tv.tv_usec;
}

void createTimePacket(uint8_t* buffer_out,uint64_t currenttime,uint8_t Odroid_Trigger)
{

	uint8_t *pointer;
	buffer_out[0]=0x01; buffer_out[1]=0x02; buffer_out[2]=0x03;
	
	buffer_out[3]=0x02; // 0 -> Reboot   1 -> Enable   2 -> Broadcast
	
	pointer=(uint8_t*)&currenttime;
	buffer_out[4]=pointer[7];
	buffer_out[5]=pointer[6];
	buffer_out[6]=pointer[5];
	buffer_out[7]=pointer[4];
	buffer_out[8]=pointer[3];
	buffer_out[9]=pointer[2];
	buffer_out[10]=pointer[1];
	buffer_out[11]=pointer[0];
	
	buffer_out[12]=Odroid_Trigger;

	buffer_out[13]=0x04; buffer_out[14]=0x05; buffer_out[15]=0x06;
}

///////////////////////////////// MAIN CODE /////////////////////////////////

int main(int argc, char* argv[])
{	
	printf("\nPediSole v1.9\n\n");
	printf("3 Nodes: LSole, RSole, Wrist\n\n");
		
	// Declare data name variable
	FILE * TemporalFile;
	char strDate[N_STR];
	char strFile[N_STR];
	char strSession[N_STR];
	char strTemporalFile[N_STR];
	
	if (argc>2)
	{
		printf("Error on input argument!\n");
		return -1;
	}
	else if (argc == 1)
	{
		// No session name, simple functioning
		sprintf(strSession, "%s", "");
	}
	else if (argc == 2)
	{
		// Simple functioning
		sprintf(strSession, "%s", argv[1]);
		printf("Session Name: %s\n", strSession);
	}
	
	bool resetLED = 0x00;
	bool condResetLed;
	bool resettingState = false;
	uint32_t lastResetTimestamp;

	// Initialize buffer packets
	uint8_t bufferLog[PACKET_LENGTH_LOG];
	uint8_t bufferPd[PACKET_LENGTH_PD];
	uint8_t bufferWrist[PACKET_LENGTH_WRIST];
	uint8_t bufferLed[PACKET_LENGTH_LED];
	uint8_t bufferSync[PACKET_LENGTH_SYNC];
	uint8_t bufferTime[PACKET_LENGTH_TIME];
	
	// Initialize threads
	thread threadPDShoeL;
	thread threadPDShoeR;
	thread threadWristR;
	thread threadSync;
	
	// Initialize data structures
	structPDShoe PDShoeL;
	structPDShoe PDShoeR;
	structWrist WristR;
	structSWstat swStat;
	structSync Sync;

	// Initialize RAW data strutures
	structDataPacketPureDataRAW dataPacketRawL;
	structDataPacketPureDataRAW dataPacketRawR;
	structDataPacketPureDataRAWWrist dataPacketRawWristR;
	structDataPacketPureData dataPacketL;
	structDataPacketPureData dataPacketR;
	structDataPacketPureDataWrist dataPacketWristR;
	structSyncPacket SyncPacket;
	
	swStat.packetLedSent=0;
	swStat.packetGuiSent=0;
	swStat.packetPdSent=0;
	swStat.packetErrorPdShoeL=0;
	swStat.packetErrorPdShoeR=0;
	swStat.packetErrorWristR = 0;
	swStat.packetErrorSync=0;
	swStat.packetBroadSent=0;
	
	// Timing variables
	struct timeval tv;
	uint64_t timestamp;
	uint64_t timestamp_start;
	uint32_t tCycle;
	uint64_t cycleMicrosTime=US_CYCLES;
	uint32_t cycles=0;
	
	time_t timer;
	struct tm tstruct;
	
	int sockfdPd;
	int sockfdGui;
	int sockfdLed;
	int sockfdSync;
	int sockfdBroad;
	
	struct sockaddr_in addrPd;
	struct sockaddr_in addrGui;
	struct sockaddr_in addrLed;
	struct sockaddr_in addrSync;
	struct sockaddr_in addrBroad;
	
	// Set up ports for network nodes
	PDShoeL.name="PD Shoe [Left]";
	PDShoeL.ipAddress="192.168.1.11";
	PDShoeL.port=3456;
	//PDShoeL.frequencyError=0;
	PDShoeL.packetError=0;
	PDShoeL.packetReceived=0;
	PDShoeL.id=1;
	
	PDShoeR.name="PD Shoe [Right]";
	PDShoeR.ipAddress="192.168.1.12";
	PDShoeR.port=3457;
	//PDShoeR.frequencyError=0;
	PDShoeR.packetError=0;
	PDShoeR.packetReceived=0;
	PDShoeR.id=2;

	WristR.name = "Wrist [Right]";
	WristR.ipAddress = "192.168.1.16";
	WristR.port = 3466;
	//WristR.frequencyError=0;
	WristR.packetError = 0;
	WristR.packetReceived = 0;
	WristR.id = 5;
	
	Sync.name="External Sync";
	Sync.ipAddress="192.168.1.13";
	Sync.port=3462;
	//Sync.frequencyError=0;
	Sync.packetError=0;
	Sync.packetReceived=0;
	Sync.id=3;
	
	// Initialize sockets
	sockfdPd=socket(AF_INET,SOCK_DGRAM,0);
	sockfdGui=socket(AF_INET,SOCK_DGRAM,0);
	sockfdLed=socket(AF_INET,SOCK_DGRAM,0);
	sockfdSync=socket(AF_INET,SOCK_DGRAM,0);
	sockfdBroad=socket(AF_INET,SOCK_DGRAM,0);
	
	// Enable broadcast
	int broadcastEnable=1;
	int ret=setsockopt(sockfdBroad, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
	
	// ODROID
	bzero(&addrPd,sizeof(addrPd));
	addrPd.sin_family = AF_INET;
    addrPd.sin_addr.s_addr=inet_addr("192.168.1.100");///("192.168.1.100");//("192.168.1.102");///
	addrPd.sin_port=htons(3459);
	
	// Sync Board (Infrared LED Cluster)
	bzero(&addrLed,sizeof(addrLed));
	addrLed.sin_family = AF_INET;
	addrLed.sin_addr.s_addr=inet_addr("192.168.1.13");
	addrLed.sin_port=htons(3458);
	
	// Computer GUI
	bzero(&addrGui,sizeof(addrGui));
	addrGui.sin_family = AF_INET;
	addrGui.sin_addr.s_addr=inet_addr("192.168.1.101");
	addrGui.sin_port=htons(3460);
	
	// Broadcast address
	bzero(&addrBroad,sizeof(addrBroad));
	addrBroad.sin_family = AF_INET;
	//addrBroad.sin_addr.s_addr=inet_addr("192.168.1.11");
	addrBroad.sin_addr.s_addr=inet_addr("192.168.1.255");
	addrBroad.sin_port=htons(3464);

	// Start User Datagram Protocol (UDP) threads
	threadPDShoeL=thread(threadUDPreceive,&PDShoeL);
	threadPDShoeR=thread(threadUDPreceive,&PDShoeR);
	threadWristR=thread(threadUDPreceiveWrist, &WristR);
	threadSync=thread(threadSYNCreceive,&Sync);
		
	threadPDShoeL.detach();
	threadPDShoeR.detach();
	threadWristR.detach();
	threadSync.detach();
	
	usleep(1000);
	
	uint8_t precTrigger;
	
	uint nPacketLed=0;
	uint64_t currenttime=0;
	
	timer=time(0);
	tstruct = *localtime(&timer);

	// Open .dat file to store data on SD card
	sprintf(strFile, "/media/rootfs/sd/log/%s.dat", strSession);
	FILE * pFile;
	pFile = fopen (strFile, "wb");

// Number of cycles in the buffer before writing to SD card		
#define CYCLES_WRITE 500
	
	// Buffer until reaching 500 cycles
	uint8_t vFileBuffer[CYCLES_WRITE*PACKET_LENGTH_LOG];
	unsigned int idxFileWrite=0;
	
	uint8_t Odroid_Trigger=1;
	uint8_t sendSportSole=2;
	
	// Some initial blinking...
	for (int un=20; un>0; un--){
		if(Odroid_Trigger>0)
		{
			Odroid_Trigger=0;
		}
		else{
			Odroid_Trigger=1;
		}
	    usleep(75000);
	  }
	
	// Waiting for incoming data packet from the nodes to start logging	
	printf("Waiting...\n");
	
	bool cond=false;
	while(!cond)
	{
		dataMutex.lock();
		// Only start recording when there are packets from all 3 nodes: LSole, RSole, RWrist
		cond=(PDShoeL.packetReceived>0) && (PDShoeR.packetReceived>0) && (WristR.packetReceived>0); 
		dataMutex.unlock();
		
		usleep(500);
	}
	
	// Start recording
	printf("Start!\n");
	timestamp_start=getMicrosTimeStamp();
		
	while(1)
	{
		timestamp=getMicrosTimeStamp();
		
		dataMutex.lock();
				
		// Reconstruct binary (RAW) data from LSole, RSole, RWrist, and SyncBoard (use external trigger pin from Zeno Mat or VICON)
		reconstructStructPureDataRAW(PDShoeL.lastPacket,dataPacketRawL);
		reconstructStructPureDataRAW(PDShoeR.lastPacket,dataPacketRawR);
		reconstructStructPureDataRAWWrist(WristR.lastPacket, dataPacketRawWristR);
		reconstructStructSyncPacket(Sync.lastPacket,SyncPacket);
		
		// Update the error stats if the packets are bad
		swStat.packetErrorPdShoeL=PDShoeL.packetError;
		swStat.packetErrorPdShoeR=PDShoeR.packetError;
		swStat.packetErrorWristR = WristR.packetError;
		swStat.packetErrorSync=Sync.packetError;

		// Update the received stats if the packets are good
		swStat.packetReceivedPdShoeL=PDShoeL.packetReceived;
		swStat.packetReceivedPdShoeR=PDShoeR.packetReceived;
		swStat.packetReceivedWristR = WristR.packetReceived;
		swStat.packetReceivedSync=Sync.packetReceived;
		
		dataMutex.unlock();
		
		// Reconstruct data types from binary (RAW) data above
		reconstructStruct(dataPacketRawL,dataPacketL);
		reconstructStruct(dataPacketRawR,dataPacketR);
		reconstructStructWrist(dataPacketRawWristR, dataPacketWristR);
	
		// Send to GUI after every 70 cycles
		if ((cycles%70)==0)
		{
			sendto(sockfdGui,bufferLog,sizeof(bufferLog),0,(struct sockaddr *)&addrGui,sizeof(addrGui));
			swStat.packetGuiSent++;
		}
		
		// Send synchronization/trigger packet out after every 1000 cycles
		if ((cycles%1000)==0)
		{
			if(Odroid_Trigger>0)
			{
				Odroid_Trigger=0;
			}
			else{
				Odroid_Trigger=1;
			}
			currenttime = getMicrosTimeStamp()-timestamp_start;
			createTimePacket(bufferTime,currenttime,Odroid_Trigger);
			sendto(sockfdBroad,bufferTime,PACKET_LENGTH_TIME,0,(struct sockaddr *)&addrBroad,sizeof(addrBroad));
			swStat.packetBroadSent++;
		}
		
		// Create log packet from the reconstructed packets (LSole, RSole, RWrist)
		createLogPacket_3nodes(bufferLog, PDShoeL.lastPacket, PDShoeR.lastPacket, WristR.lastPacket, Odroid_Trigger, currenttime, SyncPacket.Ext_Trigger);

		// Only write the log data to SD card when the write cycle is reached (500) 
		if (idxFileWrite==(CYCLES_WRITE-1))
		{
			memcpy(&vFileBuffer[idxFileWrite*PACKET_LENGTH_LOG],&bufferLog,PACKET_LENGTH_LOG);
			fwrite(&vFileBuffer,CYCLES_WRITE*PACKET_LENGTH_LOG, 1, pFile);
			idxFileWrite=0;
		}
		else
		{
			memcpy(&vFileBuffer[idxFileWrite*PACKET_LENGTH_LOG],&bufferLog,PACKET_LENGTH_LOG);
			idxFileWrite++;
		}
		
		// Show the info on the terminal after every 1000 cycles
		if((cycles%1000)==0)
		{
			float currenttime_float_sec = ((float)(currenttime))/1000000.0f;
			printf("Cycles=%d - Err(L)=%d - Err(R)=%d - Err(W)=%d - P(L)=%d - P(R)=%d - P(W)=%d - Tr=%d - ESync=%d\n", cycles, swStat.packetErrorPdShoeL, swStat.packetErrorPdShoeR, swStat.packetErrorWristR, swStat.packetReceivedPdShoeL, swStat.packetReceivedPdShoeR, swStat.packetReceivedWristR, Odroid_Trigger, SyncPacket.Ext_Trigger);
	    }
		
		// Increase the cycle
		cycles++;
		
		// Compute the cycle time
		tCycle=getMicrosTimeStamp()-timestamp;
				
		// To control the speed of the code (in microseconds)
#define US_SLEEP_CORRECTION 0 //48
		if(!(tCycle>cycleMicrosTime-US_SLEEP_CORRECTION)) usleep(cycleMicrosTime-US_SLEEP_CORRECTION-tCycle);
	}
	
	// Close the threads. Most of the time the user just use Ctrl + C to terminate the code
	threadPDShoeL.~thread();
	threadPDShoeR.~thread();	
	threadWristR.~thread();
	threadSync.~thread();
	return 0; 
} 

