/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/chipmc/Documents/Maker/Particle/Projects/Cellular-Pressure-Next/src/Cellular-Pressure-Next.ino"
/*
* Project Electron-Connected-Counters-Dual-Sensor - converged software for Low Power and Solar
* Description: Cellular Connected Data Logger for Utility and Solar powered installations
* Author: Chip McClelland
* Date:10 January 2018
*/

/*  The idea of this release is to use the new sensor model which should work with multiple sensors
    Both utility and solar implementations will move over to the finite state machine approach
    Both implementations will observe the park open and closing hours
    I will add two new states: 1) Low Power mode - maintains functionality but conserves battery by
    enabling sleep  2) Low Battery Mode - reduced functionality to preserve battery charge

    The mode will be set and recoded in the FRAM::controlRegisterAddr so resets will not change the mode
    Control Register - bits 7-5, 4-Connected Status, 3 - Verbose Mode, 2- Solar Power Mode, 1 - Low Battery Mode, 0 - Low Power Mode
*/

//v1.0 - Added the capability to monitor a second counter to track cumulative traffic in a parking lot.
//v1.1 - Changed the way we count out to improve accuracy - no bebounce / back-tire counting
//v1.2 - Added a feature to catch missed counts on the outbound tube
//v1.3 - Changed to adaptive reporting.  Will "report" whenever the net value changes by 20 (fixed now much could make settable later)
//v1.4 - Added a Particle Variable for net count and increased the time before we complete a missed out count
//v1.5 - Reducing the likelyhood that a stray frontTire2Flag will be counted 4 x debounce now
//v1.6 - Devcie has consistently overcounted cars leaving the parking lot - taking out the "frontTire2Flag" provision from the IDLE_STATE
//v1.7 - Cleaned up the record count function.  Moved userSwitch to an interrupt
//v1.8 - Moved to Async Publish Library - Got rid of the connection API as well.



// namespaces and #define statements - avoid if possible
const int FRAMversionNumber = 10;                       // Increment this number each time the memory map is changed
namespace FRAM {                                    // Moved to namespace instead of #define to limit scope
  enum Addresses {
    versionAddr           = 0x0,                    // Where we store the memory map version number
    debounceAddr          = 0x2,                    // Where we store debounce in dSec or 1/10s of a sec (ie 1.6sec is stored as 16)
    resetCountAddr        = 0x3,                    // This is where we keep track of how often the Electron was reset
    timeZoneAddr          = 0x4,                    // Store the local time zone data
    openTimeAddr          = 0x5,                    // Hour for opening the park / store / etc - military time (e.g. 6 is 6am)
    closeTimeAddr         = 0x6,                    // Hour for closing of the park / store / etc - military time (e.g 23 is 11pm)
    controlRegisterAddr   = 0x7,                    // This is the control register for storing the current state
    currentHourlyCountAddr =0x8,                    // Current Hourly Count - 16 bits
    currentNetCountAddr   = 0xA,                    // Current net count - when there are two sensors and you are tracking In - Out counts
    currentDailyCountAddr = 0xC,                    // Current Daily Count - 16 bits
    currentCountsTimeAddr = 0xE,                    // Time of last count - 32 bits
    alertsCountAddr       = 0x12,                   // Current Hour Alerts Count - one Byte
    maxMinLimitAddr       = 0x13,                   // Current value for MaxMin Limit - one Byte
    lastHookResponseAddr  = 0x14,                   // When is the last time we got a valid Webhook Response - 32 bits
    DSTOffsetValueAddr    = 0x18                    // When we apply Daylight Savings Time - what is the offset (0.0 to 2.0) - byte
    
  };
};

// Included Libraries
#include "Particle.h"                               // Particle's libraries - new for product 
void setup();
void loop();
int resetFRAM(String command);
int resetCounts(String command);
int hardResetNow(String command);
int sendNow(String command);
void resetEverything();
int setSolarMode(String command);
int setVerboseMode(String command);
int setTimeZone(String command);
int setDSTOffset(String command);
int setOpenTime(String command);
int setCloseTime(String command);
int setLowPowerMode(String command);
int setMaxMinLimit(String command);
int setNetCount(String command);
void recordCount();
void sendEvent();
void UbidotsHandler(const char *event, const char *data);
void takeMeasurements();
void getSignalStrength();
int getTemperature();
void sensor1ISR();
void sensor2ISR();
void watchdogISR();
void petWatchdog();
void userSwitchISR();
void PMICreset();
bool connectToParticle();
bool disconnectFromParticle();
bool notConnected();
bool meterParticlePublish(void);
void publishStateTransition(void);
void fullModemReset();
void dailyCleanup();
bool isDSTusa();
bool isDSTnz();
#line 55 "/Users/chipmc/Documents/Maker/Particle/Projects/Cellular-Pressure-Next/src/Cellular-Pressure-Next.ino"
#include "Adafruit_FRAM_I2C.h"                      // Library for FRAM functions
#include "FRAM-Library-Extensions.h"                // Extends the FRAM Library
#include "electrondoc.h"                            // Documents pinout
#include "PowerCheck.h"
#include "PublishQueueAsyncRK.h"


// System Macto Instances
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

// Particle Product definitions
PRODUCT_ID(11313);                                   // Connected Counter Dual Product group
PRODUCT_VERSION(1);
#define DSTRULES isDSTusa
const char releaseNumber[4] = "1.8";                  // Displays the release on the menu 


// Constants

// Pin Constants - Electron Carrier Board
const int tmp36Pin =      A0;                       // Simple Analog temperature sensor
const int wakeUpPin =     A7;                       // This is the Particle Electron WKP pin
const int tmp36Shutdwn =  B5;                       // Can turn off the TMP-36 to save energy
const int hardResetPin =  D4;                       // Power Cycles the Electron and the Carrier Board
const int donePin =       D6;                       // Pin the Electron uses to "pet" the watchdog
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D5;                       // User switch with a pull-up resistor
// Pin Constants - Sensor
const int intPin1 =       B1;                       // Pressure Sensor #1 interrupt pin - This is the tube across 1/2 the road that counts cars "In"
const int intPin2 =       B3;                       // Pressure Sensor #2 interrupt pin - This sensor is across the whole road and is used to count cars "out"
const int disableModule = B2;                       // Bringining this low turns on the sensor (pull-up on sensor board)
const int ledPower =      B4;                       // Allows us to control the indicator LED on the sensor board

// Timing Constants
const int wakeBoundary = 1*3600 + 0*60 + 0;         // 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000;          // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 45000;            // How long will we wair for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset

// Variables
// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, REPORTING_STATE, RESP_WAIT_STATE };
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Reporting", "Response Wait" };
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;
// Timing Variables
unsigned long stayAwakeTimeStamp = 0;               // Timestamps for our timing variables..
unsigned long stayAwake;                            // Stores the time we need to wait before napping
unsigned long webhookTimeStamp = 0;                 // Webhooks...
unsigned long resetTimeStamp = 0;                   // Resets - this keeps you from falling into a reset loop
unsigned long lastPublish = 0;                      // Can only publish 1/sec on avg and 4/sec burst
// Program Variables
int temperatureF;                                   // Global variable so we can monitor via cloud variable
int resetCount;                                     // Counts the number of times the Electron has had a pin reset
bool awokeFromNap = false;                          // In low power mode, we can't use standard millis to debounce
volatile bool watchdogFlag;                         // Flag to let us know we need to pet the dog
volatile bool exitLowPowerFlag;                     // Flag that the user button has been pressed and we need to exit low power mode
volatile bool frontTire1Flag = false;
bool dataInFlight = false;                          // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
byte controlRegisterValue;                          // Stores the control register values
bool lowPowerMode;                                  // Flag for Low Power Mode operations
bool connectionMode;                                // Need to store if we are going to connect or not in the register
bool solarPowerMode;                                // Changes the PMIC settings
bool verboseMode;                                   // Enables more active communications for configutation and setup
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
const char* radioTech[10] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154","LTE_CAT_M1","LTE_CAT_NB1"};
// Daily Operations Variables
int openTime;                                       // Park Opening time - (24 hr format) sets waking
int closeTime;                                      // Park Closing time - (24 hr format) sets sleep
byte currentDailyPeriod;                            // Current day
byte currentHourlyPeriod;                           // This is where we will know if the period changed
char currentOffsetStr[10];                          // What is our offset from UTC
// Battery monitoring
int stateOfCharge = 0;                              // stores battery charge level value
char powerContext[24];                              // One word that describes whether the device is getting power, charging, discharging or too cold to charge
char hourlyCountStr[16];
char dailyCountStr[16];
char currentNetStr[16];                             // Need to create string versions
// Sensor Variables
volatile bool sensor1Detect = false;                // This is the flag that an interrupt is triggered on sensor 1
volatile bool sensor2Detect = false;                // This is the flag that an interrupt is triggered on sensor 2
volatile int hourlyEventCount = 0;                           // hourly counter
int hourlyEventCountSent = 0;                       // Person count in flight to Ubidots
int currentNetCountSent = 0;                        // Keep track for adaptive reporting
volatile int dailyEventCount = 0;                            // daily counter
volatile int currentNetCount = 0;                            // Net count when you have two sensors and are tracking in - out

// Diagnostic Variables
int alerts = 0;                                     // Alerts are triggered when MaxMinLimit is exceeded or a reset due to errors
int maxMin = 0;                                     // What is the current maximum count in a minute for this reporting period
int maxMinLimit;                                    // Counts above this amount will be deemed erroroneus

// Prototypes and System Mode calls
FuelGauge batteryMonitor;                           // Prototype for the fuel gauge (included in Particle core library)
PMIC pmic;                                         //Initalize the PMIC class so you can call the Power Management functions below.
PowerCheck powerCheck;                              // instantiates the PowerCheck class to help us provide context with charge level reporting
retained uint8_t publishQueueRetainedBuffer[2048];
PublishQueueAsync publishQueue(publishQueueRetainedBuffer, sizeof(publishQueueRetainedBuffer));


void setup()                                        // Note: Disconnected Setup()
{
  /* Setup is run for three reasons once we deploy a sensor:
       1) When you deploy the sensor
       2) Each hour while the device is sleeping
       3) After a reset event
    All three of these have some common code - this will go first then we will set a conditional
    to determine which of the three we are in and finish the code
  */
  pinMode(wakeUpPin,INPUT);                         // This pin is active HIGH
  pinMode(userSwitch,INPUT);                        // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output
  digitalWrite(blueLED,HIGH);                       // Turn on the Blue LED for Startup
  pinMode(tmp36Shutdwn,OUTPUT);                     // Supports shutting down the TMP-36 to save juice
  digitalWrite(tmp36Shutdwn, HIGH);                 // Turns on the temp sensor
  pinResetFast(donePin);
  pinResetFast(hardResetPin);
  // Pressure / PIR Module Pin Setup
  pinMode(intPin1,INPUT_PULLDOWN);                  // pressure sensor interrupt #1
  pinMode(intPin2,INPUT_PULLDOWN);                  // Pressure sensor interrupt #2
  pinMode(disableModule,OUTPUT);                    // Turns on the module when pulled low
  pinResetFast(disableModule);                      // Turn on the module - send high to switch off board
  pinMode(ledPower,OUTPUT);                         // Turn on the lights
  digitalWrite(ledPower,HIGH);                      // Turns on the LED on the sensor board
  pinMode(donePin,OUTPUT);                          // Allows us to pet the watchdog
  pinMode(hardResetPin,OUTPUT);                     // For a hard reset active HIGH

  petWatchdog();                                    // Pet the watchdog - not necessary in a power on event but just in case
  attachInterrupt(wakeUpPin, watchdogISR, RISING);  // The watchdog timer will signal us and we have to respond
  attachInterrupt(userSwitch, userSwitchISR, FALLING); // Press the user switch to exit low power mode

  char responseTopic[125];
  String deviceID = System.deviceID();              // Multiple devices share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);          // Puts the deviceID into the response topic array
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("HourlyCount", hourlyCountStr);                   // Define my Particle variables
  Particle.variable("DailyCount", dailyCountStr);                     // Note: Don't have to be connected for any of this!!!
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Temperature",temperatureF);
  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", stateOfCharge);
  Particle.variable("PowerContext",powerContext);
  Particle.variable("OpenTime",openTime);
  Particle.variable("CloseTime",closeTime);
  Particle.variable("MaxMinLimit",maxMinLimit);
  Particle.variable("Alerts",alerts);
  Particle.variable("TimeOffset",currentOffsetStr);
  Particle.variable("NetCarCount",currentNetStr);

  Particle.function("resetFRAM", resetFRAM);                            // These are the functions exposed to the mobile app and console
  Particle.function("resetCounts",resetCounts);
  Particle.function("HardReset",hardResetNow);
  Particle.function("SendNow",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-DSTOffset",setDSTOffset);
  Particle.function("Set-OpenTime",setOpenTime);
  Particle.function("Set-Close",setCloseTime);
  Particle.function("Set-MaxMin-Limit",setMaxMinLimit);
  Particle.function("Set-Net-Count",setNetCount);

  // Load the elements for improving troubleshooting and reliability
  powerCheck.setup();

  // Load FRAM and reset variables to their correct values
  if (!fram.begin()) state = ERROR_STATE;                               // You can stick the new i2c addr in here, e.g. begin(0x51);
  else if (FRAMread8(FRAM::versionAddr) == 9) {                         // One time upgrade from 9 to 10
    Time.setDSTOffset(1);                                               // Set a default if out of bounds
    FRAMwrite8(FRAM::DSTOffsetValueAddr,1);                             // Write new offset value to FRAM
    FRAMwrite8(FRAM::timeZoneAddr,-5);                                  // OK, since the DSTOffset is initialized, so is the Timezone
  }
  else if (FRAMread8(FRAM::versionAddr) != FRAMversionNumber) {         // Check to see if the memory map in the sketch matches the data on the chip
    ResetFRAM();                                                        // Reset the FRAM to correct the issue
    if (FRAMread8(FRAM::versionAddr) != FRAMversionNumber)state = ERROR_STATE; // Resetting did not fix the issue
  }

  alerts = FRAMread8(FRAM::alertsCountAddr);                            // Load the alerts count
  resetCount = FRAMread8(FRAM::resetCountAddr);                         // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER)  // Check to see if we are starting from a pin reset or a reset in the sketch
  {
    resetCount++;
    FRAMwrite8(FRAM::resetCountAddr,static_cast<uint8_t>(resetCount));// If so, store incremented number - watchdog must have done This
  }

  // Check and import values from FRAM
  openTime = FRAMread8(FRAM::openTimeAddr);
  if (openTime < 0 || openTime > 22) openTime = 0;                      // Open and close in 24hr format
  closeTime = FRAMread8(FRAM::closeTimeAddr);
  if (closeTime < 1 || closeTime > 23) closeTime = 23;
  int8_t tempFRAMvalue = FRAMread8(FRAM::timeZoneAddr);
  if (tempFRAMvalue > 12 || tempFRAMvalue < -12)  Time.zone(-5);      // Default is EST in case proper value not in FRAM
  else Time.zone((float)tempFRAMvalue);                                 // Load Timezone from FRAM
  int8_t tempDSTOffset = FRAMread8(FRAM::DSTOffsetValueAddr);           // Load the DST offset value
  if (tempDSTOffset < 0 || tempDSTOffset > 2) {
    Time.setDSTOffset(1);                                               // Set a default if out of bounds
    FRAMwrite8(FRAM::DSTOffsetValueAddr,1);                             // Write new offset value to FRAM
  } 
  else Time.setDSTOffset(tempDSTOffset);                                // Set the value from FRAM if in limits     
  if (Time.isValid()) DSTRULES() ? Time.beginDST() : Time.endDST();     // Perform the DST calculation here 
  maxMinLimit = FRAMread8(FRAM::maxMinLimitAddr);                       // This is the maximum number of counts in a minute
  if (maxMinLimit < 2 || maxMinLimit > 100) maxMinLimit = 10;            // If value has never been intialized - reasonable value

  controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);          // Read the Control Register for system modes
  lowPowerMode    = (0b00000001 & controlRegisterValue);                // Bitwise AND to set the lowPowerMode flag from control Register
  verboseMode     = (0b00001000 & controlRegisterValue);                // verboseMode
  solarPowerMode  = (0b00000100 & controlRegisterValue);                // solarPowerMode
  connectionMode  = (0b00010000 & controlRegisterValue);                // connected mode 1 = connected and 0 = disconnected

  PMICreset();                                                          // Executes commands that set up the PMIC for Solar charging

  currentHourlyPeriod = Time.hour();                                    // Sets the hour period for when the count starts (see #defines)
  currentDailyPeriod = Time.day();                                      // What day is it?
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);

  time_t unixTime = FRAMread32(FRAM::currentCountsTimeAddr);            // Need to reload last recorded event - current periods set from this event
  dailyEventCount = FRAMread16(FRAM::currentDailyCountAddr);            // Load Daily Count from memory
  hourlyEventCount = FRAMread16(FRAM::currentHourlyCountAddr);          // Load Hourly Count from memory
  currentNetCount = FRAMread16(FRAM::currentNetCountAddr);              // Load the current net count

  snprintf(hourlyCountStr, sizeof(hourlyCountStr),"%i hourly",hourlyEventCount);
  snprintf(dailyCountStr, sizeof(dailyCountStr),"%i daily",dailyEventCount);
  snprintf(currentNetStr, sizeof(currentNetStr),"%i net",currentNetCount);

  takeMeasurements();                                                 // Important as we might make decisions based on temp or battery level

  // Here is where the code diverges based on why we are running Setup()
  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
  if (currentDailyPeriod != Time.day(unixTime)) {
    resetEverything();                                                // Zero the counts for the new day
    if (solarPowerMode && !lowPowerMode) setLowPowerMode("1");        // If we are running on solar, we will reset to lowPowerMode at Midnight
  }

  if ((currentHourlyPeriod > closeTime || currentHourlyPeriod < openTime)) {}         // The park is closed - sleep
  else {                                                              // Park is open let's get ready for the day
    attachInterrupt(intPin1, sensor1ISR, RISING);                      // Pressure Sensor 1 interrupt from low to high
    attachInterrupt(intPin2, sensor2ISR, RISING);                      // Second sensor
    if (connectionMode) connectToParticle();                          // Only going to connect if we are in connectionMode
    stayAwake = stayAwakeLong;                                        // Keeps Electron awake after reboot - helps with recovery
  }

  digitalWrite(ledPower,LOW);                                         // Turns off the LEDs on the sensor board and Electron
  digitalWrite(blueLED,LOW);

  if (state == INITIALIZATION_STATE) state = IDLE_STATE;              // IDLE unless otherwise from above code
}

void loop()
{
  switch(state) {
  case IDLE_STATE:                                                    // Where we spend most time - note, the order of these conditionals is important
    if (verboseMode && state != oldState) publishStateTransition();
    if (watchdogFlag) petWatchdog();                                  // Watchdog flag is raised - time to pet the watchdog
    if (exitLowPowerFlag) setLowPowerMode("0");                       // Exit low power mode
    if (sensor1Detect || sensor2Detect) recordCount();                // One or both ISRs had raised the sensor flag
    if (hourlyEventCountSent) {                                      // Cleared here as there could be counts coming in while "in Flight"
      hourlyEventCount -= hourlyEventCountSent;                     // Confirmed that count was recevied - clearing
      FRAMwrite16(FRAM::currentHourlyCountAddr, static_cast<uint16_t>(hourlyEventCount));  // Load Hourly Count to memory
      hourlyEventCountSent = maxMin = alerts = 0;                    // Zero out the counts until next reporting period
      FRAMwrite8(FRAM::alertsCountAddr,0);
      if (currentHourlyPeriod == 23) resetEverything();                // We have reported for the previous day - reset for the next - only needed if no sleep
    }
    if (Time.hour() == 2 && Time.isValid()) DSTRULES() ? Time.beginDST() : Time.endDST();    // Each day, at 2am we will check to see if we need a DST offset
    if (lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = NAPPING_STATE;  // When in low power mode, we can nap between taps
    if (Time.hour() != currentHourlyPeriod) state = REPORTING_STATE;  // We want to report on the hour but not after bedtime
    if ((Time.hour() > closeTime || Time.hour() < openTime)) state = SLEEPING_STATE;   // The park is closed - sleep - Note thie means that close time = 21 will not sleep till 22
    break;

  case SLEEPING_STATE: {                                              // This state is triggered once the park closes and runs until it opens
    if (verboseMode && state != oldState) publishStateTransition();
    detachInterrupt(intPin1);                                          // Done sensing for the day
    pinSetFast(disableModule);                                        // Turn off the pressure module for the hour
    if (hourlyEventCount) {                                          // If this number is not zero then we need to send this last count
      state = REPORTING_STATE;
      break;
    }
    if (connectionMode) disconnectFromParticle();                     // Disconnect cleanly from Particle
    digitalWrite(blueLED,LOW);                                        // Turn off the LED
    digitalWrite(tmp36Shutdwn, LOW);                                  // Turns off the temp sensor
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    System.sleep(SLEEP_MODE_DEEP,wakeInSeconds);                      // Very deep sleep till the next hour - then resets
    } break;

  case NAPPING_STATE: {  // This state puts the device in low power mode quickly
    if (verboseMode && state != oldState) publishStateTransition();
    if (sensor1Detect || sensor2Detect) break;                                          // Don't nap until we are done with event
    if ((0b00010000 & controlRegisterValue)) {                        // If we are in connected mode
      disconnectFromParticle();                                       // Disconnect from Particle
      controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);    // Get the control register (general approach)
      controlRegisterValue = (0b11101111 & controlRegisterValue);     // Turn off connected mode 1 = connected and 0 = disconnected
      connectionMode = false;
      FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);     // Write to the control register
    }
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    petWatchdog();                                                    // Reset the watchdog
    System.sleep({intPin1,intPin2},RISING,wakeInSeconds);                                             // Sensor will wake us with an interrupt or timeout at the hour
    if (sensor1Detect || sensor2Detect) {
       awokeFromNap=true;                                             // Since millis() stops when sleeping - need this to debounce
       stayAwakeTimeStamp = millis();
    }
    state = IDLE_STATE;                                               // Back to the IDLE_STATE after a nap - not enabling updates here as napping is typicallly disconnected
    } break;

  case REPORTING_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (!(0b00010000 & controlRegisterValue)) connectToParticle();    // Only attempt to connect if not already New process to get connected
    if (Particle.connected()) {
      if (Time.hour() == closeTime) dailyCleanup();                   // Once a day, clean house
      takeMeasurements();                                             // Update Temp, Battery and Signal Strength values
      sendEvent();                                                    // Send data to Ubidots
      state = RESP_WAIT_STATE;                                        // Wait for Response
    }
    else state = ERROR_STATE;
    break;

  case RESP_WAIT_STATE:
    if (verboseMode && state != oldState) publishStateTransition();
    if (!dataInFlight)                                                // Response received back to IDLE state
    {
      state = IDLE_STATE;
      stayAwake = stayAwakeLong;                                      // Keeps Electron awake after reboot - helps with recovery
      stayAwakeTimeStamp = millis();
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      resetTimeStamp = millis();
      // // waitUntil(meterParticlePublish);
      publishQueue.publish("spark/device/session/end", "", PRIVATE, WITH_ACK);      // If the device times out on the Webhook response, it will ensure a new session is started on next connect
      state = ERROR_STATE;                                            // Response timed out
    }
    break;

  case ERROR_STATE:                                                   // To be enhanced - where we deal with errors
    if (verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait)
    {
      if (resetCount <= 3) {                                          // First try simple reset
        // // waitUntil(meterParticlePublish);
        if (Particle.connected()) publishQueue.publish("State","Error State - Reset", PRIVATE, WITH_ACK);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else if (Time.now() - FRAMread32(FRAM::lastHookResponseAddr) > 7200L) { //It has been more than two hours since a sucessful hook response
        // // waitUntil(meterParticlePublish);
        if (Particle.connected()) publishQueue.publish("State","Error State - Power Cycle", PRIVATE, WITH_ACK);  // Broadcast Reset Action
        delay(2000);
        FRAMwrite8(FRAM::resetCountAddr,0);                           // Zero the ResetCount
        digitalWrite(hardResetPin,HIGH);                              // This will cut all power to the Electron AND the carrier board
      }
      else {                                                          // If we have had 3 resets - time to do something more
        // // waitUntil(meterParticlePublish);
        if (Particle.connected()) publishQueue.publish("State","Error State - Full Modem Reset", PRIVATE, WITH_ACK);            // Brodcase Reset Action
        delay(2000);
        FRAMwrite8(FRAM::resetCountAddr,0);                           // Zero the ResetCount
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }
}

// Functions that support the Particle Functions defined in Setup()
// These are called from the console or via an API call
int resetFRAM(String command)                                           // Will reset the local counts
{
  if (command == "1")
  {
    ResetFRAM();
    return 1;
  }
  else return 0;
}

int resetCounts(String command) {                                       // Resets the current hourly and daily counts
  if (command == "1") {
    resetEverything();
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)   {                                    // Will perform a hard reset on the Electron
  if (command == "1")
  {
    publishQueue.publish("Reset","Hard Reset in 2 seconds",PRIVATE, WITH_ACK);
    delay(2000);
    digitalWrite(hardResetPin,HIGH);                                    // This will cut all power to the Electron AND the carrir board
    return 1;                                                           // Unfortunately, this will never be sent
  }
  else return 0;
}

int sendNow(String command) {                                           // Function to force sending data in current hour

  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

void resetEverything() {                                                // The device is waking up in a new day or is a new install
  FRAMwrite16(FRAM::currentDailyCountAddr, 0);                          // Reset the counts in FRAM as well
  FRAMwrite16(FRAM::currentHourlyCountAddr, 0);
  FRAMwrite16(FRAM::currentNetCountAddr,0);
  FRAMwrite32(FRAM::currentCountsTimeAddr,Time.now());                  // Set the time context to the new day
  FRAMwrite8(FRAM::resetCountAddr,0);
  FRAMwrite8(FRAM::alertsCountAddr,0);
  FRAMwrite8(FRAM::alertsCountAddr,0);
  hourlyEventCount = hourlyEventCountSent = dailyEventCount = currentNetCount = resetCount = alerts = currentNetCountSent = 0;         // Reset everything for the day
  snprintf(hourlyCountStr, sizeof(hourlyCountStr),"%i hourly",hourlyEventCount);
  snprintf(dailyCountStr, sizeof(dailyCountStr),"%i daily",dailyEventCount);
  snprintf(currentNetStr, sizeof(currentNetStr),"%i net",currentNetCount);
  publishQueue.publish("Mode","Reset all counts", PRIVATE, WITH_ACK);
  dataInFlight = false;
}

int setSolarMode(String command) {                                      // Function to force sending data in current hour

  if (command == "1") {
    solarPowerMode = true;
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b00000100 | controlRegisterValue);          // Turn on solarPowerMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);               // Write it to the register
    PMICreset();                                               // Change the power management Settings
    // // waitUntil(meterParticlePublish);
    if (Particle.connected()) publishQueue.publish("Mode","Set Solar Powered Mode", PRIVATE, WITH_ACK);
    return 1;
  }
  else if (command == "0") {
    solarPowerMode = false;
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b11111011 & controlRegisterValue);           // Turn off solarPowerMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);                // Write it to the register
    PMICreset();                                                // Change the power management settings
    // // waitUntil(meterParticlePublish);
    if (Particle.connected()) publishQueue.publish("Mode","Cleared Solar Powered Mode", PRIVATE, WITH_ACK);
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending more information on the state of the device
{
  if (command == "1") {
    verboseMode = true;
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b00001000 | controlRegisterValue);         // Turn on verboseMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);         // Write it to the register
    // // waitUntil(meterParticlePublish);
    if (Particle.connected()) publishQueue.publish("Mode","Set Verbose Mode", PRIVATE, WITH_ACK);
    oldState = state;                                                   // Avoid a bogus publish once set
    return 1;
  }
  else if (command == "0") {
    verboseMode = false;
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b11110111 & controlRegisterValue);         // Turn off verboseMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);         // Write it to the register
    // // waitUntil(meterParticlePublish);
    if (Particle.connected()) publishQueue.publish("Mode","Cleared Verbose Mode", PRIVATE, WITH_ACK);
    return 1;
  }
  else return 0;
}

int setTimeZone(String command) {                                       // Set the Base Time Zone vs GMT - not Daylight savings time
  char * pEND;
  char data[256];
  time_t t = Time.now();
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                 // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) || (tempTimeZoneOffset > 12)) return 0; // Make sure it falls in a valid range or send a "fail" result
  Time.zone((float)tempTimeZoneOffset);
  FRAMwrite8(FRAM::timeZoneAddr,tempTimeZoneOffset);                    // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
  // // waitUntil(meterParticlePublish);
  if (Time.isValid()) DSTRULES() ? Time.beginDST() : Time.endDST();     // Perform the DST calculation here 
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);
  if (Particle.connected()) publishQueue.publish("Time",data, PRIVATE, WITH_ACK);
  // // waitUntil(meterParticlePublish);
  if (Particle.connected()) publishQueue.publish("Time",Time.timeStr(t), PRIVATE, WITH_ACK);
  return 1;
}

int setDSTOffset(String command) {                                      // This is the number of hours that will be added for Daylight Savings Time 0 (off) - 2 
  char * pEND;
  char data[256];
  time_t t = Time.now();
  int8_t tempDSTOffset = strtol(command,&pEND,10);                      // Looks for the first integer and interprets it
  if ((tempDSTOffset < 0) | (tempDSTOffset > 2)) return 0;              // Make sure it falls in a valid range or send a "fail" result
  Time.setDSTOffset((float)tempDSTOffset);                              // Set the DST Offset
  FRAMwrite8(FRAM::DSTOffsetValueAddr,tempDSTOffset);                   // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "DST offset %i",tempDSTOffset);
  // // waitUntil(meterParticlePublish);
  if (Time.isValid()) isDSTusa() ? Time.beginDST() : Time.endDST();     // Perform the DST calculation here 
  snprintf(currentOffsetStr,sizeof(currentOffsetStr),"%2.1f UTC",(Time.local() - Time.now()) / 3600.0);
  if (Particle.connected()) publishQueue.publish("Time",data, PRIVATE, WITH_ACK);
  // // waitUntil(meterParticlePublish);
  if (Particle.connected()) publishQueue.publish("Time",Time.timeStr(t), PRIVATE, WITH_ACK);
  return 1;
}

int setOpenTime(String command) {                                       // Set the park opening time in local 
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                              // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;                      // Make sure it falls in a valid range or send a "fail" result
  openTime = tempTime;
  FRAMwrite8(FRAM::openTimeAddr,openTime);                              // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Open time set to %i",openTime);
  // // waitUntil(meterParticlePublish);
  if (Particle.connected()) publishQueue.publish("Time",data, PRIVATE, WITH_ACK);
  return 1;
}

int setCloseTime(String command) {                                      // Set the close time in local - device will sleep between close and open
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                              // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;                      // Make sure it falls in a valid range or send a "fail" result
  closeTime = tempTime;
  FRAMwrite8(FRAM::closeTimeAddr,closeTime);                            // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Closing time set to %i",closeTime);
  // // waitUntil(meterParticlePublish);
  if (Particle.connected()) publishQueue.publish("Time",data, PRIVATE, WITH_ACK);
  return 1;
}

int setLowPowerMode(String command) {                                   // This is where we can put the device into low power mode if needed

  if (command != "1" && command != "0") return 0;                       // Before we begin, let's make sure we have a valid input
  controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);          // Get the control register (general approach)
  if (command == "1")                                                   // Command calls for setting lowPowerMode
  {
    if (verboseMode && Particle.connected()) {
      // // waitUntil(meterParticlePublish);
      publishQueue.publish("Mode","Low Power", PRIVATE, WITH_ACK);
    }
    if ((0b00010000 & controlRegisterValue)) {                          // If we are in connected mode
      Particle.disconnect();                                            // Otherwise Electron will attempt to reconnect on wake
      controlRegisterValue = (0b11101111 & controlRegisterValue);       // Turn off connected mode 1 = connected and 0 = disconnected
      connectionMode = false;
      Cellular.off();
      delay(1000);                                                      // Bummer but only should happen once an hour
    }
    controlRegisterValue = (0b00000001 | controlRegisterValue);         // If so, flip the lowPowerMode bit
    lowPowerMode = true;
  }
  else if (command == "0") {                                             // Command calls for clearing lowPowerMode
    if (verboseMode && Particle.connected()) {
      // // waitUntil(meterParticlePublish);
      publishQueue.publish("Mode","Normal Operations", PRIVATE, WITH_ACK);
    }
    
    exitLowPowerFlag = false;

    if ((Time.hour() > closeTime || Time.hour() < openTime))  {         // Device may also be sleeping due to time or TimeZone setting
      openTime = 0;                                                     // Only change these values if it is an issue
      FRAMwrite8(FRAM::openTimeAddr,0);                                 // Reset open and close time values to ensure device is awake
      closeTime = 23;
      FRAMwrite8(FRAM::closeTimeAddr,23);
    }
    if (!Particle.connected()) {
      connectionMode = true;
      controlRegisterValue = (0b00010000 | controlRegisterValue);       // Turn on connected mode 1 = connected and 0 = disconnected
      connectToParticle();                                              // Get connected to Particle    }
    }
    controlRegisterValue = (0b11111110 & controlRegisterValue);         // If so, flip the lowPowerMode bit
    lowPowerMode = false;
  }
  FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);           // Write to the control register
  return 1;
}


int setMaxMinLimit(String command) {                                    // This limits the number of cars in an hour - also used for diagnostics
  char * pEND;
  char data[256];
  int tempMaxMinLimit = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempMaxMinLimit < 2) || (tempMaxMinLimit > 100)) return 0;        // Make sure it falls in a valid range or send a "fail" result
  maxMinLimit = tempMaxMinLimit;
  FRAMwrite8(FRAM::maxMinLimitAddr,maxMinLimit);                        // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "MaxMin limit set to %i",maxMinLimit);
  // // waitUntil(meterParticlePublish);
  if (Particle.connected()) publishQueue.publish("MaxMin",data,PRIVATE, WITH_ACK);
  return 1;
}

int setNetCount(String command) {                                       // Set the park opening time in local 
  char * pEND;
  char data[256];
  int tempCount = strtol(command,&pEND,10);                              // Looks for the first integer and interprets it
  if ((tempCount < 0) || (tempCount > dailyEventCount)) return 0;                      // Make sure it falls in a valid range or send a "fail" result
  currentNetCount = tempCount;
  snprintf(currentNetStr, sizeof(currentNetStr),"%i net",currentNetCount);
  FRAMwrite16(FRAM::currentNetCountAddr,currentNetCount);                              // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Net count is set to %i",currentNetCount);
  // // waitUntil(meterParticlePublish);
  if (Particle.connected()) publishQueue.publish("Net",data, PRIVATE, WITH_ACK);
  return 1;
}

// Here are the primary "business" functions that are specific to this devices function
void recordCount() {                                                    // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the Arduino
  static byte currentMinutePeriod;                                    // Current minute
  static int currentMinuteCount = 0;                                  // What is the count for the current minute

  pinSetFast(blueLED);                                                // Turn on the blue LED

  if (sensor1Detect) {
    snprintf(hourlyCountStr, sizeof(hourlyCountStr),"%i hourly",hourlyEventCount);
    snprintf(dailyCountStr, sizeof(dailyCountStr),"%i daily",dailyEventCount);
    FRAMwrite16(FRAM::currentHourlyCountAddr, hourlyEventCount);       // Load Hourly Count to memory
    FRAMwrite16(FRAM::currentDailyCountAddr, dailyEventCount);         // Load Daily Count to memory
    FRAMwrite32(FRAM::currentCountsTimeAddr, Time.now());              // Write to FRAM - this is so we know when the last counts were saved 

    // Diagnostic code
    if (currentMinutePeriod != Time.minute()) {                       // Done counting for the last minute
      currentMinutePeriod = Time.minute();                            // Reset period
      currentMinuteCount = 1;                                         // Reset for the new minute
    }
    else currentMinuteCount++;

    if (currentMinuteCount >= maxMin) maxMin = currentMinuteCount;    // Save only if it is the new maxMin
    // End diagnostic code


    // Set an alert if we exceed the MaxMinLimit
    if (currentMinuteCount >= maxMinLimit) {
      if (verboseMode && Particle.connected()) {
        // // waitUntil(meterParticlePublish);
        publishQueue.publish("Alert", "Exceeded Maxmin limit", PRIVATE, WITH_ACK);
      }
      alerts++;
      FRAMwrite8(FRAM::alertsCountAddr,alerts);                        // Save counts in case of reset
    }
    sensor1Detect = false;
  }

  if (sensor2Detect) {
    // Adaptive rate reporting - net change in 20 from last sent value
    if (abs(currentNetCountSent - currentNetCount) >= 20) state= REPORTING_STATE;

    if (verboseMode && Particle.connected()) {
      char data[256];                                                  // Store the date in this character array - not global
      snprintf(data, sizeof(data), "Count, hourly: %i, daily: %i, net: %i",hourlyEventCount,dailyEventCount,currentNetCount);
      publishQueue.publish("Count",data, PRIVATE, WITH_ACK);                         // Helpful for monitoring and calibration
    }
    sensor2Detect = false;
  }

  snprintf(currentNetStr, sizeof(currentNetStr),"%i net",currentNetCount);
  FRAMwrite16(FRAM::currentNetCountAddr,currentNetCount);            // Store in FRAM

  pinResetFast(blueLED);
}


void sendEvent()
{
  char data[256];                                                     // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i, \"net\":%i, \"battery\":%i, \"key1\":\"%s\", \"temp\":%i, \"resets\":%i, \"alerts\":%i, \"maxmin\":%i}",hourlyEventCount, dailyEventCount, currentNetCount, stateOfCharge, powerContext, temperatureF, resetCount, alerts, maxMin);
  publishQueue.publish("Ubidots-Counter-Hook-Dual", data, PRIVATE,WITH_ACK);
  webhookTimeStamp = millis();
  currentHourlyPeriod = Time.hour();                                  // Change the time period
  if(currentHourlyPeriod == 23) hourlyEventCount++;                   // Ensures we don't have a zero here at midnigtt
  hourlyEventCountSent = hourlyEventCount;                            // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
  currentNetCountSent = currentNetCount;                              // What did we last send.  
  dataInFlight = true;                                                // set the data inflight flag
}

void UbidotsHandler(const char *event, const char *data)              // Looks at the response from Ubidots - Will reset Photon if no successful response
{                                                                     // Response Template: "{{hourly.0.status_code}}" so, I should only get a 3 digit number back
  char dataCopy[strlen(data)+1];                                      // data needs to be copied since if (Particle.connected()) publishQueue.publish() will clear it
  strncpy(dataCopy, data, sizeof(dataCopy));                          // Copy - overflow safe
  if (!strlen(dataCopy)) {                                            // First check to see if there is any data
    // // waitUntil(meterParticlePublish);
    if (Particle.connected()) publishQueue.publish("Ubidots Hook", "No Data", PRIVATE, WITH_ACK);
    return;
  }
  int responseCode = atoi(dataCopy);                                  // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    // // // waitUntil(meterParticlePublish);
    publishQueue.publish("State","Response Received", PRIVATE,WITH_ACK);
    FRAMwrite32(FRAM::lastHookResponseAddr,Time.now());                     // Record the last successful Webhook Response
    dataInFlight = false;                                             // Data has been received
  }
  else if (Particle.connected()) publishQueue.publish("Ubidots Hook", dataCopy, PRIVATE, WITH_ACK);                    // Publish the response code
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{
  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready
  stateOfCharge = int(batteryMonitor.getSoC());                       // Percentage of full charge

  getTemperature();                                                   // Load the current temp

  if (temperatureF <= 32 || temperatureF >= 113) {                      // Need to add temp charging controls - 
    snprintf(powerContext, sizeof(powerContext), "Chg Disabled Temp");
    pmic.disableCharging();                                          // Disable Charging if temp is too low or too high
    // // waitUntil(meterParticlePublish);
    if (Particle.connected()) publishQueue.publish("Alert", "Charging disabled Temperature",PRIVATE, WITH_ACK);
    return;                                                           // Return to avoid the enableCharging command at the end of the IF statement
  }
  else if (powerCheck.getIsCharging()) {
    snprintf(powerContext, sizeof(powerContext), "Charging");
  }
  else if (powerCheck.getHasPower()) {
    snprintf(powerContext, sizeof(powerContext), "DC-In Powered");
  }
  else if (powerCheck.getHasBattery()) {
    snprintf(powerContext, sizeof(powerContext), "Battery Discharging");
  }
  else snprintf(powerContext, sizeof(powerContext), "Undetermined");

  pmic.enableCharging();                                               // We are good to charge 
}


void getSignalStrength()
{
  // New Signal Strength capability - https://community.particle.io/t/boron-lte-and-cellular-rssi-funny-values/45299/8
  CellularSignal sig = Cellular.RSSI();

  auto rat = sig.getAccessTechnology();
 
  //float strengthVal = sig.getStrengthValue();
  float strengthPercentage = sig.getStrength();

  //float qualityVal = sig.getQualityValue();
  float qualityPercentage = sig.getQuality();

  snprintf(SignalString,sizeof(SignalString), "%s S:%2.0f%%, Q:%2.0f%% ", radioTech[rat], strengthPercentage, qualityPercentage);
}

int getTemperature() {
  int reading = analogRead(tmp36Pin);                                 //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                    //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration

  temperatureF = int((temperatureC * 9.0 / 5.0) + 32.0);              // now convert to Fahrenheit

  return temperatureF;
}


// Here are the various hardware and timer interrupt service routines
// These are supplemental or utility functions
void sensor1ISR() {
  if (!frontTire1Flag) frontTire1Flag = true;
}


void sensor2ISR() {                                                 // ISR for the second sensor
  static bool frontTire2Flag = false;

  if (frontTire2Flag) {                                                 // If the front tire has already passed, then proceed to count
    frontTire2Flag = false;
    sensor2Detect = true;                                               // Set the flag
    currentNetCount--;                                                 // The "Out" tube counts one as it counts both lanes
    if (currentNetCount < 0) currentNetCount = 0;                      // Can't go negative

    if (frontTire1Flag) {                                         // This case should not occur, Front Tire 1 Flag should be false if Front Tire 2 flag is
      frontTire1Flag = false;
      sensor1Detect = true;                                               // Reset the flag
      hourlyEventCount++;                                                // Increment the PersonCount
      dailyEventCount++;                                                 // Increment the PersonCount
      currentNetCount +=2;                                               // The "In" tube counts twice as it is only across 1/2 the road
    }
  }
  else frontTire2Flag = true;
}

void watchdogISR() {
  watchdogFlag = true;
}


void petWatchdog() {
  digitalWriteFast(donePin, HIGH);                                  // Pet the watchdog
  digitalWriteFast(donePin, LOW);
  watchdogFlag = false;
}

void userSwitchISR() {
  exitLowPowerFlag = true;
}

// Power Management function
void PMICreset() {
  pmic.begin();                                                      // Settings for Solar powered power management
  pmic.disableWatchdog();
  if (solarPowerMode) {
    pmic.setInputVoltageLimit(4840);                                 // Set the lowest input voltage to 4.84 volts best setting for 6V solar panels
    pmic.setInputCurrentLimit(900);                                  // default is 900mA
    pmic.setChargeCurrent(0,0,1,0,0,0);                              // default is 512mA matches my 3W panel
    pmic.setChargeVoltage(4208);                                     // Allows us to charge cloe to 100% - battery can't go over 45 celcius
  }
  else  {
    pmic.setInputVoltageLimit(4208);                                 // This is the default value for the Electron
    pmic.setInputCurrentLimit(1500);                                 // default is 900mA this let's me charge faster
    pmic.setChargeCurrent(0,1,1,0,0,0);                              // default is 2048mA (011000) = 512mA+1024mA+512mA)
    pmic.setChargeVoltage(4112);                                     // default is 4.112V termination voltage
  }
}

bool connectToParticle() {
  Cellular.on();
  Particle.connect();
  // wait for *up to* 5 minutes
  for (int retry = 0; retry < 300 && !waitFor(Particle.connected,1000); retry++) {
    if(sensor1Detect || sensor2Detect) recordCount(); // service the interrupt every second
    Particle.process();
  }
  if (Particle.connected()) {
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);        // Get the control register (general approach)
    controlRegisterValue = (0b00010000 | controlRegisterValue);         // Turn on connected mode 1 = connected and 0 = disconnected
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);         // Write to the control register
    return 1;                                                           // Were able to connect successfully
  }
  else {
    return 0;                                                           // Failed to connect
  }
}

bool disconnectFromParticle() {                                         // Ensures we disconnect cleanly from Particle
  Particle.disconnect();
  for (int retry = 0; retry < 15 && !waitFor(notConnected, 1000); retry++) {  // make sure before turning off the cellular modem
    if(sensor1Detect || sensor2Detect) recordCount(); // service the interrupt every second
    Particle.process();
  }
  Cellular.off();
  delay(2000);                                                          // Bummer but only should happen once an hour
  return true;
}

bool notConnected() {                                                   // Companion function for disconnectFromParticle
  return !Particle.connected();
}

bool meterParticlePublish(void) {                                       // Enforces Particle's limit on 1 publish a second
  static unsigned long lastPublish=0;                                   // Initialize and store value here
  if(millis() - lastPublish >= 1000) {                                  // Particle rate limits at 1 publish per second
    lastPublish = millis();
    return 1;
  }
  else return 0;
}

void publishStateTransition(void) {                                     // Mainly for troubleshooting - publishes the transition between states
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if(Particle.connected()) {
    // // waitUntil(meterParticlePublish);
    publishQueue.publish("State Transition",stateTransitionString, PRIVATE, WITH_ACK);
  }
  Serial.println(stateTransitionString);
}

void fullModemReset() {                                                 // Adapted form Rikkas7's https://github.com/rickkas7/electronsample
	Particle.disconnect(); 	                                              // Disconnect from the cloud
	unsigned long startTime = millis();  	                                // Wait up to 15 seconds to disconnect
	while(Particle.connected() && millis() - startTime < 15000) {
		delay(100);
	}
	// Reset the modem and SIM card
	// 16:MT silent reset (with detach from network and saving of NVM parameters), with reset of the SIM card
	Cellular.command(30000, "AT+CFUN=16\r\n");
	delay(1000);
	// Go into deep sleep for 10 seconds to try to reset everything. This turns off the modem as well.
	System.sleep(SLEEP_MODE_DEEP, 10);
}

void dailyCleanup() {                                                 // Function to clean house at the end of the day
  controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);        // Load the control Register

  // // waitUntil(meterParticlePublish);
  publishQueue.publish("Daily Cleanup","Running", PRIVATE, WITH_ACK);               // Make sure this is being run

  verboseMode = false;
  controlRegisterValue = (0b11110111 & controlRegisterValue);         // Turn off verboseMode

  Particle.syncTime();                                                // Set the clock each day
  waitFor(Particle.syncTimeDone,30000);                               // Wait for up to 30 seconds for the SyncTime to complete

  if (solarPowerMode || stateOfCharge <= 70) {                        // If the battery is being discharged
    controlRegisterValue = (0b00000001 | controlRegisterValue);       // If so, put the device in lowPowerMode
    lowPowerMode = true;
    controlRegisterValue = (0b11101111 & controlRegisterValue);       // Turn off connected mode 1 = connected and 0 = disconnected
    connectionMode = false;
  }
  FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);         // Write it to the register
}

bool isDSTusa() { 
  // United States of America Summer Timer calculation (2am Local Time - 2nd Sunday in March/ 1st Sunday in November)
  // Adapted from @ScruffR's code posted here https://community.particle.io/t/daylight-savings-problem/38424/4
  // The code works in from months, days and hours in succession toward the two transitions
  int dayOfMonth = Time.day();
  int month = Time.month();
  int dayOfWeek = Time.weekday() - 1; // make Sunday 0 .. Saturday 6

  // By Month - inside or outside the DST window
  if (month >= 4 && month <= 10)  
  { // April to October definetly DST
    return true;
  }
  else if (month < 3 || month > 11)
  { // before March or after October is definetly standard time
    return false;
  }

  boolean beforeFirstSunday = (dayOfMonth - dayOfWeek < 0);
  boolean secondSundayOrAfter = (dayOfMonth - dayOfWeek > 7);

  if (beforeFirstSunday && !secondSundayOrAfter) return (month == 11);
  else if (!beforeFirstSunday && !secondSundayOrAfter) return false;
  else if (!beforeFirstSunday && secondSundayOrAfter) return (month == 3);

  int secSinceMidnightLocal = Time.now() % 86400;
  boolean dayStartedAs = (month == 10); // DST in October, in March not
  // on switching Sunday we need to consider the time
  if (secSinceMidnightLocal >= 2*3600)
  { //  In the US, Daylight Time is based on local time 
    return !dayStartedAs;
  }
  return dayStartedAs;
}

bool isDSTnz() { 
  // New Zealand Summer Timer calculation (2am Local Time - last Sunday in September/ 1st Sunday in April)
  // Adapted from @ScruffR's code posted here https://community.particle.io/t/daylight-savings-problem/38424/4
  // The code works in from months, days and hours in succession toward the two transitions
  int dayOfMonth = Time.day();
  int month = Time.month();
  int dayOfWeek = Time.weekday() - 1; // make Sunday 0 .. Saturday 6

  // By Month - inside or outside the DST window - 10 out of 12 months with April and Septemper in question
  if (month >= 10 || month <= 3)  
  { // October to March is definetly DST - 6 months
    return true;
  }
  else if (month < 9 && month > 4)
  { // before September and after April is definetly standard time - - 4 months
    return false;
  }

  boolean beforeFirstSunday = (dayOfMonth - dayOfWeek < 6);
  boolean lastSundayOrAfter = (dayOfMonth - dayOfWeek > 23);

  if (beforeFirstSunday && !lastSundayOrAfter) return (month == 4);
  else if (!beforeFirstSunday && !lastSundayOrAfter) return false;
  else if (!beforeFirstSunday && lastSundayOrAfter) return (month == 9);

  int secSinceMidnightLocal = Time.now() % 86400;
  boolean dayStartedAs = (month == 10); // DST in October, in March not
  // on switching Sunday we need to consider the time
  if (secSinceMidnightLocal >= 2*3600)
  { //  In the US, Daylight Time is based on local time 
    return !dayStartedAs;
  }
  return dayStartedAs;
}
