#include <Nextion.h>  //Library untuk layar sentuh Nextion

#include <EEPROM.h>   //untuk membaca dan menulis data ke dan dari EEPROM

#include <avr/wdt.h>  //Library untuk watchdog (digunakan untuk mereset Arduino jika loop memakan waktu terlalu lama, tetapi juga digunakan sebagai opsi reset lunak)

//Definisikan konstanta untuk port input yang digunakan dalam proyek ini
const int led = 13;

float startPosX = 215.30;
float startPosY = -1.00;
float startPosZ = -59.00;
float startPosR = 0.00; //Posisi dobot saat start awal
float currentX = startPosX;
float currentY = startPosY;
float currentZ = startPosZ;
float currentR = startPosR;
bool currentVac = false;

float dobotPartX = 0;
float dobotPartY = 0;
float intermediateX = 0;
float intermediateY = 0; // variabel perantara yang digunakan dalam perhitungan

float moveInterval = 1.0; //langkah peningkatan dalam mm yang digunakan untuk jogging
int activeAxis = 1; //sumbu aktif untuk jogging atau perintah gerakan lainnya

bool refreshScreen = true; // variabel untuk menunjukkan item layar perlu (di)gambar ulang

//variables used for delay timer without delay function (to not lock controls during delay)
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
long delayInterval = 1500; // delay between moves of the robot arm to make sure all commands are processed properly
//int moveStep = 0; //index number for the current movement step in automatic cycle
//
uint32_t speedRatio = 0;
uint32_t accelRatio = 0;

//variables for Nextion

String displayText = "...";
int CurrentPage = 0;

//Declare Nextion , pageid, component id., component name//
NexPage page0 = NexPage(0, 0, "page0");
NexDSButton bt1 = NexDSButton(0, 4, "bt1");
NexDSButton bt2 = NexDSButton(0, 5, "bt2");
NexDSButton bt3 = NexDSButton(0, 6, "bt3");
NexDSButton bt4 = NexDSButton(0, 7, "bt4");
NexDSButton bt5 = NexDSButton(0, 8, "bt5");
NexDSButton bt6 = NexDSButton(0, 9, "bt6");
NexDSButton bt7 = NexDSButton(0, 10, "bt7");
NexDSButton bt8 = NexDSButton(0, 11, "bt8");
NexButton b7 = NexButton(0, 2, "b7");
NexButton b12 = NexButton(0, 3, "b12");
NexText t0 = NexText(0, 1, "t0");
NexText t1 = NexText(0, 3, "t1");
NexText t2 = NexText(0, 12, "t2");
NexText t3 = NexText(0, 13, "t3");
NexText t4 = NexText(0, 14, "t4");

char buffer[100] = {
  0
};

//Nextion display : Register a button object to the touch event list//
NexTouch * nex_listen_list[] = {
  &
  page0,
  &
  bt1,
  &
  bt2,
  &
  bt3,
  &
  bt4,
  &
  bt5,
  &
  bt6,
  &
  bt7,
  &
  bt8,
  &
  b7,
  &
  b12,

  NULL
};

//Dobot Magician//
#include "stdio.h"

#include "Protocol.h"

#include "command.h"

#include "FlexiTimer2.h"

//Set Serial TX&RX Buffer Size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

//#define JOG_STICK 

// Global parameters Dobot Magician//

EndEffectorParams gEndEffectorParams;

JOGJointParams gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd gPTPCmd;

uint64_t gQueuedCmdIndex;
/********************************/

// Page change event:
void page0PushCallback(void * ptr) // If page 0 is loaded on the display, the following is going to execute:
{
  CurrentPage = 0; // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
  refreshScreen = true;
  Serial.println(F("Page 0 activated"));
} // End of press event

// Page change event:
void page1PushCallback(void * ptr) // If page 1 is loaded on the display, the following is going to execute:
{
  CurrentPage = 1; // Set variable to 1 so from now on arduino knows page 1 is loaded on the display
  refreshScreen = true;
  Serial.println(F("Page 1 activated"));
} // End of press event

void bt1PopCallback(void * ptr) //Axis X
{
  uint32_t dual_state;
  bt1.getValue( & dual_state);
  if (dual_state) {
    activeAxis = 1;
  } else {}
}

void bt2PopCallback(void * ptr) //Axis Y
{
  uint32_t dual_state;
  bt2.getValue( & dual_state);
  if (dual_state) {
    activeAxis = 2;
  } else {}
}
void bt3PopCallback(void * ptr) //Axis Z
{
  uint32_t dual_state;
  bt3.getValue( & dual_state);
  if (dual_state) {
    activeAxis = 3;
  } else {}
}
void bt4PopCallback(void * ptr) //Axis R
{
  uint32_t dual_state;
  bt4.getValue( & dual_state);
  if (dual_state) {
    activeAxis = 4;
  } else {}
}

void bt5PopCallback(void * ptr) {
  uint32_t dual_state;
  bt5.getValue( & dual_state);
  if (dual_state) {
    moveInterval = 0.1;
  } else {}
}
void bt6PopCallback(void * ptr) {
  uint32_t dual_state;
  bt6.getValue( & dual_state);
  if (dual_state) {
    moveInterval = 1.0;
  } else {}
}

void bt7PopCallback(void * ptr) {
  uint32_t dual_state;
  bt7.getValue( & dual_state);
  if (dual_state) {
    moveInterval = 10;
  } else {}
}
void bt8PopCallback(void * ptr) {
  uint32_t dual_state;
  bt8.getValue( & dual_state);
  if (dual_state) {
    moveInterval = 20;
  } else {}
}
void b0PopCallback(void * ptr) {
  Serial.println(F("Jog"));
  refreshScreen = true;
}
void b1PopCallback(void * ptr) {
  Serial.println(F("Auto"));
  refreshScreen = true;
}
void b2PopCallback(void * ptr) {
  Serial.println(F("Settings"));
  refreshScreen = true;
}

void b7PopCallback(void * ptr) {
  if (activeAxis == 1) moveArm((gPTPCmd.x += moveInterval), currentY, currentZ, currentR, currentVac);
  if (activeAxis == 2) moveArm(currentX, (gPTPCmd.y += moveInterval), currentZ, currentR, currentVac);
  if (activeAxis == 3) moveArm(currentX, currentY, (gPTPCmd.z += moveInterval), currentR, currentVac);
  if (activeAxis == 4) moveArm(currentX, currentY, currentZ, (gPTPCmd.r += moveInterval), currentVac);
  refreshScreen = true;
}

//Button b12 component popcallback function
// When OFF button is released the LED turns OFF and the state text changes

void b12PopCallback(void * ptr) {
  if (activeAxis == 1) moveArm((gPTPCmd.x -= moveInterval), currentY, currentZ, currentR, currentVac);
  if (activeAxis == 2) moveArm(currentX, (gPTPCmd.y -= moveInterval), currentZ, currentR, currentVac);
  if (activeAxis == 3) moveArm(currentX, currentY, (gPTPCmd.z -= moveInterval), currentR, currentVac);
  if (activeAxis == 4) moveArm(currentX, currentY, currentZ, (gPTPCmd.r -= moveInterval), currentVac);
  refreshScreen = true;
}

//end Nextion popcallback functions//

void setup() {

  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // **** setup miscellaneous ********************************** 
  Serial.begin(9600); //arduino debugging port
  delay(500); //give serial ports time to initialize before using them
  Serial1.begin(115200); //pins 18 and 19 on Arduino Mega board (attach Dobot Magician)
  delay(500); //give serial ports time to initialize before using them
  Serial2.begin(115200); //pins 16 and 17 on Arduino Mega board (attach Nextion Display)
  delay(500); //give serial ports time to initialize before using them
  printf_begin();
  //Set Timer Interrupt
  FlexiTimer2::set(100, Serialread);
  FlexiTimer2::start();

  nexInit(); // initialize Nextion

  //Nextion Display: Register the pop event callback function of the components
  page0.attachPush(page0PushCallback); // Page press event
  bt1.attachPop(bt1PopCallback, & bt1);
  bt2.attachPop(bt2PopCallback, & bt2);
  bt3.attachPop(bt3PopCallback, & bt3);
  bt4.attachPop(bt4PopCallback, & bt4);
  bt5.attachPop(bt5PopCallback, & bt5);
  bt6.attachPop(bt6PopCallback, & bt6);
  bt7.attachPop(bt7PopCallback, & bt7);
  bt8.attachPop(bt8PopCallback, & bt8);
  b7.attachPop(b7PopCallback, & b7);
  b12.attachPop(b12PopCallback, & b12);

  Serial.println(F("|==>START MANUAL JOGGING<==|"));
  //setup arduino pins//

  Serial2.print("bt1.val="); // set initial state of button X
  Serial2.print(1); // this state means button is pressed
  Serial2.write(0xff); // always send these 3 lines after a writing to the display
  Serial2.write(0xff);
  Serial2.write(0xff);
  // turn rest of axis buttons off
  Serial2.print("bt2.val=");
  Serial2.print(0);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("bt3.val=");
  Serial2.print(0);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("bt4.val=");
  Serial2.print(0);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

  Serial2.print("bt6.val="); // set initial state of button increment 1
  Serial2.print(1); // this state means button is pressed
  Serial2.write(0xff); // always send these 3 lines after a writing to the display
  Serial2.write(0xff);
  Serial2.write(0xff);
  // turn rest of increment buttons off
  Serial2.print("bt5.val=");
  Serial2.print(0);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("bt7.val=");
  Serial2.print(0);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.print("bt8.val=");
  Serial2.print(0);
  Serial2.write(0xff);
  Serial2.write(0xff);
  Serial2.write(0xff);

}
void Serialread() {
  while (Serial1.available()) {
    uint8_t data = Serial1.read();
    if (RingBufferIsFull( & gSerialProtocolHandler.rxRawByteQueue) == false) {
      RingBufferEnqueue( & gSerialProtocolHandler.rxRawByteQueue, & data);
    }
  }
}

int Serial_putc(char c, struct __file * ) {
  Serial.write(c);
  return c;
}

void printf_begin(void) {
  fdevopen( & Serial_putc, 0);
}

void InitRAM(void) {
  //Set JOG Model
  gQueuedCmdIndex = 0;

  gJOGJointParams.velocity[0] = 100;
  gJOGJointParams.velocity[1] = 100;
  gJOGJointParams.velocity[2] = 100;
  gJOGJointParams.velocity[3] = 100;
  gJOGJointParams.acceleration[0] = 80;
  gJOGJointParams.acceleration[1] = 80;
  gJOGJointParams.acceleration[2] = 80;
  gJOGJointParams.acceleration[3] = 80;

  gJOGCoordinateParams.velocity[0] = 100;
  gJOGCoordinateParams.velocity[1] = 100;
  gJOGCoordinateParams.velocity[2] = 100;
  gJOGCoordinateParams.velocity[3] = 100;
  gJOGCoordinateParams.acceleration[0] = 80;
  gJOGCoordinateParams.acceleration[1] = 80;
  gJOGCoordinateParams.acceleration[2] = 80;
  gJOGCoordinateParams.acceleration[3] = 80;

  gJOGCommonParams.velocityRatio = 50;
  gJOGCommonParams.accelerationRatio = 50;

  gJOGCmd.cmd = AP_DOWN;
  gJOGCmd.isJoint = JOINT_MODEL;

  //Set PTP Model
  gPTPCoordinateParams.xyzVelocity = 100;
  gPTPCoordinateParams.rVelocity = 100;
  gPTPCoordinateParams.xyzAcceleration = 80;
  gPTPCoordinateParams.rAcceleration = 80;

  gPTPCommonParams.velocityRatio = 50;
  gPTPCommonParams.accelerationRatio = 50;

  gPTPCmd.ptpMode = MOVL_XYZ;

  //Set initial pose (start position)
  moveArm(startPosX, startPosY, startPosZ, startPosR, false);
}
void moveArm(float x, float y, float z, float r, bool vacuumOn) {

  gPTPCmd.x = x;
  gPTPCmd.y = y;
  gPTPCmd.z = z;
  gPTPCmd.r = r;

  Serial.print("move to x:");
  Serial.print(gPTPCmd.x);
  Serial.print(" y:");
  Serial.print(gPTPCmd.y);
  Serial.print(" z:");
  Serial.println(gPTPCmd.r);

  SetPTPCmd( & gPTPCmd, true, & gQueuedCmdIndex);

  ProtocolProcess();

  currentX = x;
  currentY = y;
  currentZ = z;
  currentR = r;
  currentVac = vacuumOn;
}

void loop() {

  InitRAM();

  ProtocolInit();

  SetJOGJointParams( & gJOGJointParams, true, & gQueuedCmdIndex);

  SetJOGCoordinateParams( & gJOGCoordinateParams, true, & gQueuedCmdIndex);

  SetJOGCommonParams( & gJOGCommonParams, true, & gQueuedCmdIndex);

  SetPTPCmd( & gPTPCmd, true, & gQueuedCmdIndex);
  ProtocolProcess();

  for (;;) //start infinite loop
  {

    nexLoop(nex_listen_list); // Nextion 

    if (refreshScreen) {
      displayText = String((gPTPCmd.x), 1);
      t1.setText(displayText.c_str());
      displayText = String((gPTPCmd.y), 1);
      t2.setText(displayText.c_str());
      displayText = String((gPTPCmd.z), 1);
      t3.setText(displayText.c_str());
      displayText = String((gPTPCmd.r), 1);
      t4.setText(displayText.c_str());
      refreshScreen = false;
    }

    delay(10);
  }
}
