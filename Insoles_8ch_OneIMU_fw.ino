// Insoles firmware with 1 YEI IMU and 8 pressure sensors
/////////////////////////////////////////////////////////////////////

// DEFINE GLOBAL BOOLEAN VARIABLES
#define TRUE 1
#define FALSE 0

// DEFINE DATA PACKET LENGTH TO SEND OVER WIFI
#define BYTE_PUREDATA_PACKET 51
#define BYTE_WIFI_INCOMING_PACKET 16

// DEFINE REGISTERS TO RESTART TEENSY REMOTELY
/////////////////
#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);
/////////////////

// PARAMETERS FOR MULTIPLEXER
#define PIN_ENABLE 12
#define PIN_A      11
#define PIN_B      10
#define PIN_C      9
#define PIN_PRESSURE A9

// ONBOARD LED FOR DEBUGGING
#define LED_TEENSY 13

// DEFINE REGISTER ADDRESSES FOR YEI IMU
#define MAX_YEI_DATA_PACKET 255

#define DEGTORAD  0.017453f
#define RADTODEG 57.295779f

#define NO_SLOT	255
#define READ_TARED_ORIENTATION_AS_QUATERNION 0
#define READ_TARED_ORIENTATION_AS_EULER_ANGLES 1
#define READ_UNTARED_ORIENTATION_AS_QUATERNION 6
#define READ_UNTARED_ORIENTATION_AS_EULER_ANGLES 7
#define READ_NORMALIZED_ACCELEROMETER_VECTOR  34
#define READ_CORRECTED_ACCELEROMETER_VECTOR   39
#define READ_CORRECTED_LINEAR_ACCELERATION    41
#define READ_RAW_GYROSCOPE_VECTOR 65
#define READ_RAW_ACCELEROMETER_VECTOR 66
#define	READ_RAW_COMPASS_VECTOR 67

#define CMD_SET_BASE_OFFSET_WITH_CURRENT_ORIENTATION 22
#define CMD_SET_STREAMING_SLOT 80
#define CMD_SET_STREAMING_TIMING 82
#define CMD_GET_STREAMING_BATCH 84
#define CMD_START_STREAMING 85
#define CMD_STOP_STREAMING 86
#define CMD_TARE_WITH_CURRENT_ORIENTATION 96
#define CMD_SET_REFERENCE_VECTOR_MODE 105
#define CMD_SET_COMPASS_ENABLE 109
#define CMD_RESET_FILTER 120
#define CMD_SET_ACCELEROMETER_RANGE 121
#define CMD_SET_FILTER_MODE 123
#define CMD_SET_GYROSCOPE_RANGE 125
#define CMD_SET_COMPASS_RANGE 126

#define CMD_GET_COMPASS_ENABLED_STATE 142
#define CMD_GET_ACCELEROMETER_RANGE 148
#define CMD_GET_FILTER_MODE 152
#define CMD_GET_GYROSCOPE_RANGE 154
#define CMD_GET_COMPASS_RANGE 155
#define CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION 165
#define CMD_SET_CALIBRATION_MODE 169
#define CMD_RESPONSE_HEADER_BITFIELD 221
#define CMD_SET_UART_BAUD_RATE 231
#define CMD_GET_UART_BAUD_RATE 232
#define START_SPI_DATA_TRANSFER 0xF6
#define START_NO_RESP_HEADER 0xF7
#define START_RESP_HEADER 0xF9

#define SPI_IDLE_STATE 0x00
#define SPI_READY_STATE 0x01
#define SPI_BUSY_STATE 0x02
#define SPI_ACC_STATE 0x04
#define DELAY_SPI_YEI 5 //[us]
#define DELAY_WAIT_SERIAL_YEI 1 //[us]
#define DELAY_SERIAL_YEI 10 //[us]

#define REFERENCE_VECTOR_SINGLE_STATIC_MODE 0
#define REFERENCE_VECTOR_SINGLE_AUTO_MODE 1
#define REFERENCE_VECTOR_SINGLE_AUTO_CONTINUOUS_MODE 2
#define REFERENCE_VECTOR_MULTI_REFERENCE_MODE 3

#define ACCELEROMETER_RANGE_2G 0
#define ACCELEROMETER_RANGE_4G 1
#define ACCELEROMETER_RANGE_8G 2

#define GYROSCOPE_RANGE_250 0
#define GYROSCOPE_RANGE_500 1
#define GYROSCOPE_RANGE_2000 2

#define COMPASS_RANGE_0_8 0
#define COMPASS_RANGE_1_3 1
#define COMPASS_RANGE_1_9 2
#define COMPASS_RANGE_2_5 3
#define COMPASS_RANGE_4_0 4
#define COMPASS_RANGE_4_7 5
#define COMPASS_RANGE_5_6 6
#define COMPASS_RANGE_8_1 7

#define FILTER_IMU 0
#define FILTER_KALMAN 1
#define FILTER_KALMAN_ALTERNATING 2
#define FILTER_COMPLEMENTARY 3
#define FILTER_QUATERNION_GRADIENT_DESCEND 4
#define FILTER_MAGNETORESISTIVE_QUATERNION_GRADIENT_DESCEND 5

#define CALIBRATION_MODE_BIAS 0
#define CALIBRATION_MODE_BIAS_SCALE 1
#define CALIBRATION_MODE_ORTHO 2

#define CMD_GET_FIRMWARE_VERSION 0xDF
#define CMD_GET_HARDWARE_VERSION 0xE6

#define YEI_DELAY_AFTER_COMMAND 100 //[ms]

uint8_t YEIdataPacket[MAX_YEI_DATA_PACKET];

// SET SERIAL 3 FOR IMU, SERIAL 1 FOR WIFI (THROUGH XBEE EXPLORER)
#define IMU1 Serial3//Serial1//Serial3 //Beta
#define WIFI Serial1//Serial3

// CURVE FITTING COEFFICIENTS FOR PRESSURE SENSORS
const float coef1[] = {4.9623, 4.9715, 4.5513, 4.9613, 5.0471, 4.8370, 4.7539, 4.8434}; // Multiply by 10000
const float coef2[] = {22.869, 28.117, 22.400, 21.504, 26.127, 22.159, 25.630, 22.922}; // Multiply by 1000
const uint16_t segmentpoint = 581;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// DATA STRUCTURES
struct structStreamingTimingInformation
{
  unsigned int interval;
  unsigned int duration;
  unsigned int delay;
} sStreamingTime;

struct structComponentLinearAcceleration
{
  //Big Endian
  float az;
  float ay;
  float ax;
};

struct structComponentQuaternion
{
  //Big Endian
  float qw;
  float qz;
  float qy;
  float qx;
};

struct structEulerAngles
{
  //Big Endian
  float roll;
  float yaw;
  float pitch;
};

struct structComponentSensorData
{
  //Big Endian
  float mx;
  float my;
  float mz;

  float ax;
  float ay;
  float az;

  float gx;
  float gy;
  float gz;
};

struct structStreamingData
{
  //Big Endian
  structComponentLinearAcceleration lAcc;
  //structEulerAngles eulerAngles;
  structComponentQuaternion q;
};

struct structIncomingWifiPacket
{
  uint8_t Start_bytes[3];
  uint8_t val; // 0 - Reboot ; 1 - Start Recording ; 2 - Broadcast Timestamp
  uint8_t Odroid_Timestamp[8];
  uint8_t Odroid_Trigger;
  uint8_t Stop_bytes[3];
};

union unionIncomingWifi
{
  uint8_t vData[BYTE_WIFI_INCOMING_PACKET];
  structIncomingWifiPacket Raw;
} Wifi_Incoming_Data;

union unionStreamingData
{
  structStreamingData sData;
  uint8_t vData[sizeof(structStreamingData)];

} uStreamingDataIMU1;

union unionComponentSensorData
{
  structComponentSensorData sData;
  uint8_t vData[sizeof(structComponentSensorData)];

} uCompSensData;

uint8_t calcCRC256(uint8_t* dataPacket, uint8_t nByte)
{
  uint16_t checksum = 0;
  for (uint8_t i = 1; i < nByte; i++)
  {
    checksum += dataPacket[i];
  }

  return (checksum % 256);
}

int waitByteCountFromSerial(HardwareSerial& serial, unsigned int bytecount)
{
  while (serial.available() < bytecount)
  {
    delayMicroseconds(DELAY_WAIT_SERIAL_YEI);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// FUNCTIONS TO READ DATA FROM IMU USING SERIAL COMMUNICATION
void YEIsettingsHeader(HardwareSerial& serial)
{
  // Settings Header
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = CMD_RESPONSE_HEADER_BITFIELD;
  YEIdataPacket[2] = 0x00;
  YEIdataPacket[3] = 0x00;
  YEIdataPacket[4] = 0x00;
  YEIdataPacket[5] = 0x00;
  YEIdataPacket[6] = calcCRC256(YEIdataPacket, 6);
  serial.write(YEIdataPacket, 7);
  delay(YEI_DELAY_AFTER_COMMAND);
}

void YEIwriteCommandNoDelay(HardwareSerial& serial, uint8_t cmd)
{
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = cmd;
  YEIdataPacket[2] = calcCRC256(YEIdataPacket, 2);
  serial.write(YEIdataPacket, 3);
}

void YEIwriteCommand(HardwareSerial& serial, uint8_t cmd)
{
  YEIwriteCommandNoDelay(serial, cmd);
  delay(YEI_DELAY_AFTER_COMMAND);
}

void YEIwriteCommand(HardwareSerial& serial, uint8_t cmd, uint8_t value)
{
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = cmd;
  YEIdataPacket[2] = value;
  YEIdataPacket[3] = calcCRC256(YEIdataPacket, 3);
  serial.write(YEIdataPacket, 4);
  delay(YEI_DELAY_AFTER_COMMAND);
}

uint8_t YEIgetValue(HardwareSerial& serial, uint8_t cmd)
{
  YEIwriteCommand(serial, cmd);
  return serial.read();
}

void YEIsetStreamingTime(HardwareSerial& serial)
{
  // Set Streaming Time

  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = CMD_SET_STREAMING_TIMING;

  uint8_t *pointer = (uint8_t*)&sStreamingTime.interval;
  YEIdataPacket[2] = pointer[3];
  YEIdataPacket[3] = pointer[2];
  YEIdataPacket[4] = pointer[1];
  YEIdataPacket[5] = pointer[0];

  pointer = (uint8_t*)&sStreamingTime.duration;
  YEIdataPacket[6] = pointer[3];
  YEIdataPacket[7] = pointer[2];
  YEIdataPacket[8] = pointer[1];
  YEIdataPacket[9] = pointer[0];

  pointer = (uint8_t*)&sStreamingTime.delay;
  YEIdataPacket[10] = pointer[3];
  YEIdataPacket[11] = pointer[2];
  YEIdataPacket[12] = pointer[1];
  YEIdataPacket[13] = pointer[0];

  YEIdataPacket[14] = calcCRC256(YEIdataPacket, 14);
  serial.write(YEIdataPacket, 15);

  //delay(YEI_DELAY_AFTER_COMMAND);
}

void YEIsetStreamingMode(HardwareSerial& serial, uint8_t slot1, uint8_t slot2, uint8_t slot3, uint8_t slot4, uint8_t slot5, uint8_t slot6, uint8_t slot7, uint8_t slot8)
{
  // Setting Streaming Mode
  YEIdataPacket[0] = START_NO_RESP_HEADER;
  YEIdataPacket[1] = CMD_SET_STREAMING_SLOT;

  YEIdataPacket[2] = slot1; //1st slot
  YEIdataPacket[3] = slot2; //2nd slot
  YEIdataPacket[4] = slot3; //3rd slot
  YEIdataPacket[5] = slot4; //4th slot
  YEIdataPacket[6] = slot5; //5th slot
  YEIdataPacket[7] = slot6; //6th slot
  YEIdataPacket[8] = slot7; //7th slot
  YEIdataPacket[9] = slot8; //8th slot

  YEIdataPacket[10] = calcCRC256(YEIdataPacket, 10);
  serial.write(YEIdataPacket, 11);

  delay(YEI_DELAY_AFTER_COMMAND);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// THIS FUNCTION IS TO CREATE THE DATA PACKET THAT IS SENT THROUGH SERIAL OR WIFI
uint8_t dataCommunicationPacket[BYTE_PUREDATA_PACKET];

#ifndef SWAP_ELEMENT
#define SWAP_ELEMENT(a,b) { a^=b; b^=a; a^=b; }
#endif

uint8_t nPacketStreamingData = sizeof(structStreamingData);

// SOME VARIABLES FOR TIMING
unsigned long startTime;
unsigned long nowTime = 0;
unsigned long nowTime2 = 0;
unsigned long precTimestamp;

unsigned long int cycles = 0;
boolean led_teensy = 0;

boolean reset_imu;
boolean trigger;
boolean signal;

uint8_t bIMU1;

float yaw1, pitch1, roll1;

uint8_t Odroid_Timestamp[8]; //[ms]
uint8_t Odroid_Trigger; //[ms]
unsigned long currenttime;
uint8_t cmd;
uint16_t p[] = {0, 0, 0, 0, 0, 0, 0, 0};
uint16_t offset_p[] = {0, 0, 0, 0, 0, 0, 0, 0};


void createDataPacket_PureData(uint8_t *dataPacket,
                               unsigned long timestamp,
                               float yaw1, float pitch1, float roll1,
                               float ax1, float ay1, float az1,
                               uint16_t p1, uint16_t p2, uint16_t p3, uint16_t p4,
                               uint16_t p5, uint16_t p6, uint16_t p7, uint16_t p8,
                               uint8_t Odroid_Timestamp[8], uint8_t Odroid_Trigger, unsigned long timestamp2)
{
  uint8_t* pointer;
  int16_t val;

  // START
  dataPacket[0] = 0x07;
  dataPacket[1] = 0x08;
  dataPacket[2] = 0x09;

  //Timestamp
  pointer = (uint8_t*)&timestamp;
  dataPacket[3] = pointer[3];
  dataPacket[4] = pointer[2];
  dataPacket[5] = pointer[1];
  dataPacket[6] = pointer[0];

  //yaw1
  val = int16_t(yaw1 * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[7] = pointer[1];
  dataPacket[8] = pointer[0];

  //pitch1
  val = int16_t(pitch1 * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[9] = pointer[1];
  dataPacket[10] = pointer[0];

  //roll1
  val = int16_t(roll1 * 5000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[11] = pointer[1];
  dataPacket[12] = pointer[0];

  //ax1
  val = int16_t(ax1 * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[13] = pointer[1];
  dataPacket[14] = pointer[0];

  //ay1
  val = int16_t(ay1 * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[15] = pointer[1];
  dataPacket[16] = pointer[0];

  //az1
  val = int16_t(az1 * 1000.0f);
  pointer = (uint8_t*)&val;
  dataPacket[17] = pointer[1];
  dataPacket[18] = pointer[0];

  //p1
  pointer = (uint8_t*)&p1;
  dataPacket[19] = pointer[1];
  dataPacket[20] = pointer[0];

  //p2
  pointer = (uint8_t*)&p2;
  dataPacket[21] = pointer[1];
  dataPacket[22] = pointer[0];

  //p3
  pointer = (uint8_t*)&p3;
  dataPacket[23] = pointer[1];
  dataPacket[24] = pointer[0];

  //p4
  pointer = (uint8_t*)&p4;
  dataPacket[25] = pointer[1];
  dataPacket[26] = pointer[0];

  //p5
  pointer = (uint8_t*)&p5;
  dataPacket[27] = pointer[1];
  dataPacket[28] = pointer[0];

  //p6
  pointer = (uint8_t*)&p6;
  dataPacket[29] = pointer[1];
  dataPacket[30] = pointer[0];

  //p7
  pointer = (uint8_t*)&p7;
  dataPacket[31] = pointer[1];
  dataPacket[32] = pointer[0];

  //p8
  pointer = (uint8_t*)&p8;
  dataPacket[33] = pointer[1];
  dataPacket[34] = pointer[0];

  //Timestamp from ODROID
  dataPacket[35] = Odroid_Timestamp[0];
  dataPacket[36] = Odroid_Timestamp[1];
  dataPacket[37] = Odroid_Timestamp[2];
  dataPacket[38] = Odroid_Timestamp[3];
  dataPacket[39] = Odroid_Timestamp[4];
  dataPacket[40] = Odroid_Timestamp[5];
  dataPacket[41] = Odroid_Timestamp[6];
  dataPacket[42] = Odroid_Timestamp[7];

  // Trigger from ODROID
  dataPacket[43] = Odroid_Trigger;

  //Timestamp2
  pointer = (uint8_t*)&timestamp2;
  dataPacket[44] = pointer[3];
  dataPacket[45] = pointer[2];
  dataPacket[46] = pointer[1];
  dataPacket[47] = pointer[0];

  // STOP
  dataPacket[48] = 0xA;
  dataPacket[49] = 0xB;
  dataPacket[50] = 0xC;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// CONVERT QUATERNIONS FROM IMU TO EULER ANGLES
void quaternion2euler(float q0, float q1, float q2, float q3, float& phi, float& theta, float& psi)
{
  q1 = -q1;
  q2 = -q2;
  q3 = -q3;

  float R11 = 2 * q0 * q0 - 1 + 2 * q1 * q1;
  float R21 = 2 * (q1 * q2 - q0 * q3);
  float R31 = 2 * (q1 * q3 + q0 * q2);
  float R32 = 2 * (q2 * q3 - q0 * q1);
  float R33 = 2 * q0 * q0 - 1 + 2 * q3 * q3;

  phi = atan2(R32, R33);
  theta = -atan(R31 / sqrt(1 - R31 * R31) );
  psi = atan2(R21, R11 );
}

// FUNCTION TO READ SIGNAL FROM PRESSURE SENSORS THROUGH MULTIPLEXER
uint16_t get_pressure(uint8_t num)
{
  uint16_t val = 0;
  //int val;
  digitalWrite(PIN_ENABLE, LOW);
  switch (num) {
    case 1:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, LOW);
      break;
    case 2:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      break;
    case 3:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, LOW);
      break;
    case 4:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, HIGH);
      break;
    case 5:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, HIGH);
      break;
    case 6:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, HIGH);
      digitalWrite(PIN_C, HIGH);
      break;
    case 7:
      digitalWrite(PIN_A, HIGH);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      break;
    case 8:
      digitalWrite(PIN_A, LOW);
      digitalWrite(PIN_B, LOW);
      digitalWrite(PIN_C, LOW);
      break;
  }
  delayMicroseconds(4);
  val = analogRead(PIN_PRESSURE);
  val = Volt2Pressure(num, val);
  return ((uint16_t) val);
}

// CONVERT VOLTAGE READINGS TO PRESSURES
uint16_t Volt2Pressure(uint16_t num, uint16_t RawOutput)
{
  float val;
  if (RawOutput <= segmentpoint)
  {
    val = coef1[num - 1] * RawOutput;
  } 
  else 
  {
    val = coef2[num - 1] * (RawOutput - segmentpoint) + coef1[num - 1] * segmentpoint;
  }
  return ((uint16_t) val);
}

// BOOLEAN FUNCTIONS TO CHECK THE INCOMING WIFI PACKETS 
// Check if the received packet has the correct head and tail bytes
boolean checkWIFIPacket(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03  && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6);
}

// Check for start command
boolean checkRemoteStart2(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03 && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6) && (buffer[3] == 0x01);
}

// Check for reboot command
boolean checkRemoteReboot2(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03 && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6) && (buffer[3] == 0x00);
}

// Check for timestamp sent from ODROID
boolean checkOdroidTimestamp(uint8_t *buffer)
{
  return (buffer[0] == 0x01 && buffer[1] == 0x02 && buffer[2] == 0x03 && buffer[BYTE_WIFI_INCOMING_PACKET - 3] == 0x4 && buffer[BYTE_WIFI_INCOMING_PACKET - 2] == 0x5 && buffer[BYTE_WIFI_INCOMING_PACKET - 1] == 0x6) && (buffer[3] == 0x02);
}

////////////////////////////////////////   MAIN CODE   //////////////////////////////////////////////////

void setup()
{
  // Set up baud rate for Serial and WIFI
  Serial.begin(115200);
  IMU1.begin(921600);
  WIFI.begin(921600);
  Serial.setTimeout(100);
  IMU1.setTimeout(100);
  WIFI.setTimeout(100);

  delay(100);
  IMU1.clear();
  WIFI.clear();

  pinMode(LED_TEENSY, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_A, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  pinMode(PIN_C, OUTPUT);

  // START WITH EMPTY DATA PACKET
  for (int i = 0; i < sizeof(dataCommunicationPacket); i++) dataCommunicationPacket[i] = 0x00;

  // Some initial blinks
  for (int i = 0; i < 20; i++)
  {
    led_teensy = !led_teensy;
    digitalWrite(LED_TEENSY, led_teensy);
    delay(75);
  }
  led_teensy = TRUE;
  digitalWrite(LED_TEENSY, led_teensy);

  // WAIT FOR START COMMAND FROM WIFI
  boolean StartRec = FALSE;
  uint8_t bWIFI;
  while (StartRec == FALSE)
  {
    if (WIFI.available() > 0)
    {
      bWIFI = 0;
      while (!(bWIFI >= BYTE_WIFI_INCOMING_PACKET))
      {
        if ((WIFI.available() > 0) && (bWIFI < BYTE_WIFI_INCOMING_PACKET))
        {
          Wifi_Incoming_Data.vData[bWIFI] = WIFI.read();
          bWIFI++;
        }
      }
      //check integrity of the packet, process the packet
      if (checkRemoteStart2(Wifi_Incoming_Data.vData))
      {
        StartRec = TRUE;
      }
    }
    led_teensy = !led_teensy;
    digitalWrite(LED_TEENSY, led_teensy);
    delay(75);
  }
 
  // SEND COMMAND STOP STREAMING TO IMU IN CASE OF COLLECTING DATA IN CONSECUTIVE SESSIONS
  YEIsettingsHeader(IMU1);
  YEIwriteCommand(IMU1, CMD_STOP_STREAMING);
  IMU1.clear();

  // INITIALIZE IMU SETTINGS
  YEIwriteCommand(IMU1, CMD_SET_ACCELEROMETER_RANGE, ACCELEROMETER_RANGE_4G);
  YEIwriteCommand(IMU1, CMD_SET_GYROSCOPE_RANGE, GYROSCOPE_RANGE_2000);
  YEIwriteCommand(IMU1, CMD_SET_COMPASS_RANGE, COMPASS_RANGE_1_3);
  YEIwriteCommand(IMU1, CMD_SET_CALIBRATION_MODE, CALIBRATION_MODE_BIAS_SCALE);
  YEIwriteCommand(IMU1, CMD_SET_REFERENCE_VECTOR_MODE, REFERENCE_VECTOR_MULTI_REFERENCE_MODE);
  YEIwriteCommand(IMU1, CMD_SET_COMPASS_ENABLE, FALSE);

  // START CALIBRATING GYROSCOPE AND RESET ALL FILTERS
  YEIwriteCommandNoDelay(IMU1, CMD_BEGIN_GYROSCOPE_AUTOCALIBRATION);
  delay(3000);
  YEIwriteCommandNoDelay(IMU1, CMD_RESET_FILTER);
  delay(1000);

  YEIsetStreamingMode(IMU1, READ_TARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_ACCELEROMETER_VECTOR, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);

  sStreamingTime.interval = 2000; //10000; //[us]
  sStreamingTime.duration = 0xFFFFFFFF;
  sStreamingTime.delay = 0;	//[us]
  YEIsetStreamingTime(IMU1);

  IMU1.clear();
  IMU1.flush();

  reset_imu = FALSE;
  signal = FALSE;

  cmd = 0x00;

  // START COUNTING TIME
  startTime = micros();
  precTimestamp = micros();
}

void loop()
{
  // CHECK FOR INCOMING WIFI PACKET FOR ODROID TIMESTAMP OR REBOOT COMMAND
  uint8_t bWIFI;
  if (WIFI.available() > 0)
  {
    bWIFI = 0;
    while (!(bWIFI >= BYTE_WIFI_INCOMING_PACKET))
    {
      if ((WIFI.available() > 0) && (bWIFI < BYTE_WIFI_INCOMING_PACKET))
      {
        Wifi_Incoming_Data.vData[bWIFI] = WIFI.read();
        bWIFI++;
      }
    }
    //Serial.println("Received!");
    //check integrity
    if (checkWIFIPacket(Wifi_Incoming_Data.vData))
    {
      //Serial.println("Received and validated!");
      if (checkOdroidTimestamp(Wifi_Incoming_Data.vData))
      {
		nowTime2 = micros() - startTime;  
        for (int i = 0; i < sizeof(Odroid_Timestamp); i++) Odroid_Timestamp[i] = Wifi_Incoming_Data.Raw.Odroid_Timestamp[i];
        Odroid_Trigger = Wifi_Incoming_Data.Raw.Odroid_Trigger;
        //Serial.print(Odroid_Trigger);
        //Serial.println("This is Odroid Timestamp!");
      }
      else if (checkRemoteReboot2(Wifi_Incoming_Data.vData))
      {
        //Serial.println("This is Odroid Reboot!"); delay(1000);
        CPU_RESTART;
      }
      else
      {
        //Serial.println("Bad Packet!");
      }
    }
  }

  // IN THE FIRST 4 SECONDS, IMU RETURNS DATA IN POLLING MODE. THIS WAS USED IN GRAVITY COMPENSATION IN DATA POST PROCESSING.
  nowTime = micros() - startTime;
  if (nowTime >= 4000000)
  {
    if (reset_imu == false)
    {
      reset_imu = true;
      YEIwriteCommandNoDelay(IMU1, CMD_STOP_STREAMING);
      IMU1.flush();
      YEIsetStreamingMode(IMU1, READ_TARED_ORIENTATION_AS_QUATERNION, READ_CORRECTED_LINEAR_ACCELERATION, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT, NO_SLOT);
      YEIwriteCommandNoDelay(IMU1, CMD_TARE_WITH_CURRENT_ORIENTATION);
      IMU1.flush();
      YEIwriteCommandNoDelay(IMU1, CMD_START_STREAMING);
      signal = !signal;
    }
    else
    {
      bIMU1 = 0;
      while (!(bIMU1 >= nPacketStreamingData))
      {
        if ((IMU1.available() > 0) && (bIMU1 < nPacketStreamingData))
        {
          uStreamingDataIMU1.vData[nPacketStreamingData - bIMU1 - 1] = IMU1.read();
          bIMU1++;
        }

      }
      quaternion2euler(uStreamingDataIMU1.sData.q.qw, uStreamingDataIMU1.sData.q.qy, uStreamingDataIMU1.sData.q.qx, uStreamingDataIMU1.sData.q.qz, yaw1, pitch1, roll1);
    }
  }
  else // AFTER 4 SECONDS, IMU RETURNS DATA IN STREAMING MODE
  {
    YEIwriteCommandNoDelay(IMU1, CMD_GET_STREAMING_BATCH);
    waitByteCountFromSerial(IMU1, nPacketStreamingData);
    for (uint8_t idx = nPacketStreamingData; idx > 0; idx--)
    {
      uStreamingDataIMU1.vData[idx - 1] = IMU1.read();
    }
  }

  // READ PRESSURE SENSORS
  for (uint8_t h = 0; h < 8; h++) {
    p[h] = get_pressure(h + 1) - offset_p[h];
  }

  // CREATE DATA PACKET
  createDataPacket_PureData(dataCommunicationPacket,
                            nowTime, // Internal Timestamp of the microcontroller

                            // IMU data
                            yaw1,
                            pitch1,
                            roll1,
                            -uStreamingDataIMU1.sData.lAcc.ax,
                            -uStreamingDataIMU1.sData.lAcc.az,
                            uStreamingDataIMU1.sData.lAcc.ay,

                            // Pressure sensor data
                            p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7],

                            // Broadcast Synchronization
                            Odroid_Timestamp, Odroid_Trigger, nowTime2);

  // SEND DATA PACKET TO WIFI SERIAL PORT
  WIFI.write(dataCommunicationPacket, BYTE_PUREDATA_PACKET);

  // BLINK THE ONBOARD LED EVERY 250 CYCLES
  if ((cycles % 250) == 0)
  {
    led_teensy = !led_teensy;
    digitalWrite(LED_TEENSY, led_teensy);
  }

  cycles++; // CYCLE JUST FOR COUNTING, NOT USED IN DATA PACKET
}
