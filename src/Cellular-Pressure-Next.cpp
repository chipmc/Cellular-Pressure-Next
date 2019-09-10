/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "/Users/chipmc/Documents/Maker/Particle/Projects/Cellular-Pressure-Next/src/Cellular-Pressure-Next.ino"
/*
* Project Cellular-MMA8452Q - converged software for Low Power and Solar
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

//v1.02 - Updated to zero alerts on reset and clear
//v1.03 - Enabled 24 hour operations
//v1.04 - Makes sure count gets reset at 11pm since Ubidots will miscount if we wait till midnight
//v1.04a - Fix to 1.04 for non-zero count hours
//v1.04b - Makes sure reset happens at 2300 hours - so Ubidots counts correctly
//v1.05 - Updated the Signal reporting for the console / mobile app
//v1.06 - Added ConnectionEVents from Rick's Electron Sample
//v1.07 - Added the full electronsample suite of tests
//v1.08 - Moved over to Rick's low battery checks - eliminated state
//v1.09 - Took out connection check as it was causing hourly reboots
//v1.10 - Added a syncTime instruction when there is a new day
//v1.11 - Took our alert increment except in exceeing maxMinLimit
//v1.12 - Took out App watchdog, session monitoring, added reset session on Webhook timeout and reset if more than 2 hrs on webhook
//v1.13 - Added some messaging to the ERROR_STATE
//v1.14 - More responsive to counts while connecting, better Synctime and revert to lowPower daily if on Solar
//v1.15 - Fixed reset do loop if more than 4 resets and more than 2 hours since reporting (or a new install)
//v1.16 - Added logic for when connections are not successful
//v1.17 - Fix for MaxMinLimit Particle variable
//v1.18 - Semi-Automatic mode vs. manual mode
//v1.19 - Improved disconnectFromParticle()
//v1.20 - Improved meterParticlePublish - fix for note getting connected with user button
//v1.21 - Added a nightly cleanup function for time, lowPower and verboseMode
//v1.21b - Fixed an error in meterParticlePublish()
//v2 - Turn off the LED on the sensor after Setup. Moving to product firmware numbering - integer
//v3 - Preventing an update while data is being uploaded

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
    currentDailyCountAddr = 0xC,                    // Current Daily Count - 16 bits
    currentCountsTimeAddr = 0xE,                    // Time of last count - 32 bits
    alertsCountAddr       = 0x12,                   // Current Hour Alerts Count
    maxMinLimitAddr       = 0x13,                   // Current value for MaxMin Limit
    lastHookResponseAddr  = 0x14                    // When is the last time we got a valid Webhook Response
  };
};

const int versionNumber = 9;                        // Increment this number each time the memory map is changed

const char releaseNumber[6] = "2";                  // Displays the release on the menu ****  this is not a production release ****

// Included Libraries
#include "Particle.h"
void setup();
void loop();
void recordCount();
void sendEvent();
void UbidotsHandler(const char *event, const char *data);
void takeMeasurements();
void getSignalStrength();
int getTemperature();
void sensorISR();
void watchdogISR();
void petWatchdog();
void PMICreset();
bool connectToParticle();
bool disconnectFromParticle();
bool notConnected();
int resetFRAM(String command);
int resetCounts(String command);
int hardResetNow(String command);
int setDebounce(String command);
int sendNow(String command);
void resetEverything();
int setSolarMode(String command);
int setVerboseMode(String command);
int setTimeZone(String command);
int setOpenTime(String command);
int setCloseTime(String command);
int setLowPowerMode(String command);
int setMaxMinLimit(String command);
bool meterParticlePublish(void);
void publishStateTransition(void);
void fullModemReset();
void dailyCleanup();
#line 68 "/Users/chipmc/Documents/Maker/Particle/Projects/Cellular-Pressure-Next/src/Cellular-Pressure-Next.ino"
#include "Adafruit_FRAM_I2C.h"                      // Library for FRAM functions
#include "FRAM-Library-Extensions.h"                // Extends the FRAM Library
#include "electrondoc.h"                            // Documents pinout
#include "ConnectionEvents.h"                       // Stores information on last connection attemt in memory
#include "BatteryCheck.h"

PRODUCT_ID(4441);
PRODUCT_VERSION(2);

// Prototypes and System Mode calls
SYSTEM_MODE(SEMI_AUTOMATIC);                        // This will enable user code to start executing automatically.
SYSTEM_THREAD(ENABLED);                             // Means my code will not be held up by Particle processes.
STARTUP(System.enableFeature(FEATURE_RESET_INFO));
FuelGauge batteryMonitor;                           // Prototype for the fuel gauge (included in Particle core library)
PMIC power;                                         //Initalize the PMIC class so you can call the Power Management functions below.
ConnectionEvents connectionEvents("connEventStats");// Connection events object
//ConnectionCheck connectionCheck;
//AppWatchdogWrapper watchdog(60000);
//SessionCheck sessionCheck(3600);
BatteryCheck batteryCheck(15.0, 3600);


// State Maching Variables
enum State { INITIALIZATION_STATE, ERROR_STATE, IDLE_STATE, SLEEPING_STATE, NAPPING_STATE, REPORTING_STATE, RESP_WAIT_STATE };
char stateNames[8][14] = {"Initialize", "Error", "Idle", "Sleeping", "Napping", "Reporting", "Response Wait" };
State state = INITIALIZATION_STATE;
State oldState = INITIALIZATION_STATE;


// Pin Constants - Carrier Board
const int tmp36Pin =      A0;                       // Simple Analog temperature sensor
const int wakeUpPin =     A7;                       // This is the Particle Electron WKP pin
const int tmp36Shutdwn =  B5;                       // Can turn off the TMP-36 to save energy
const int hardResetPin =  D4;                       // Power Cycles the Electron and the Carrier Board
const int donePin =       D6;                       // Pin the Electron uses to "pet" the watchdog
const int blueLED =       D7;                       // This LED is on the Electron itself
const int userSwitch =    D5;                       // User switch with a pull-up resistor
// Pin Constants - Sensor
const int intPin =        B1;                       // Pressure Sensor inerrupt pin
const int analogIn =      B2;                       // This pin sees the raw output of the pressure sensor
const int disableModule = B3;                       // Bringining this low turns on the sensor (pull-up on sensor board)
const int ledPower =      B4;                       // Allows us to control the indicator LED on the sensor board


// Timing Variables
const int wakeBoundary = 1*3600 + 0*60 + 0;         // 1 hour 0 minutes 0 seconds
const unsigned long stayAwakeLong = 90000;          // In lowPowerMode, how long to stay awake every hour
const unsigned long webhookWait = 45000;            // How long will we wair for a WebHook response
const unsigned long resetWait = 30000;              // How long will we wait in ERROR_STATE until reset
const int publishFrequency = 1000;                  // We can only publish once a second
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
bool dataInFlight = false;                          // Tracks if we have sent data but not yet cleared it from counts until we get confirmation
byte controlRegisterValue;                          // Stores the control register values
bool lowPowerMode;                                  // Flag for Low Power Mode operations
bool connectionMode;                                // Need to store if we are going to connect or not in the register
bool solarPowerMode;                                // Changes the PMIC settings
bool verboseMode;                                   // Enables more active communications for configutation and setup
char SignalString[64];                              // Used to communicate Wireless RSSI and Description
const char* radioTech[8] = {"Unknown","None","WiFi","GSM","UMTS","CDMA","LTE","IEEE802154"};

// Time Related Variables
int openTime;                                       // Park Opening time - (24 hr format) sets waking
int closeTime;                                      // Park Closing time - (24 hr format) sets sleep
byte currentDailyPeriod;                            // Current day
byte currentHourlyPeriod;                           // This is where we will know if the period changed

// Battery monitoring
int stateOfCharge = 0;                              // stores battery charge level value

// This section is where we will initialize sensor specific variables, libraries and function prototypes
// Pressure Sensor Variables
u_int16_t debounce;                                 // This is the numerical value of debounce - in millis()
char debounceStr[8] = "NA";                         // String to make debounce more readable on the mobile app
volatile bool sensorDetect = false;                 // This is the flag that an interrupt is triggered
unsigned long currentEvent = 0;                     // Time for the current sensor event
int hourlyPersonCount = 0;                          // hourly counter
int hourlyPersonCountSent = 0;                      // Person count in flight to Ubidots
int dailyPersonCount = 0;                           // daily counter

// These are diagnostic measures that I am playing with
int alerts = 0;                                     // Alerts are triggered when MaxMinLimit is exceeded or a reset due to errors
int maxMin = 0;                                     // What is the current maximum count in a minute for this reporting period
int maxMinLimit;                                    // Counts above this amount will be deemed erroroneus

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
  pinMode(analogIn,INPUT);                          // Not used but don't want it floating
  pinMode(userSwitch,INPUT);                        // Momentary contact button on board for direct user input
  pinMode(blueLED, OUTPUT);                         // declare the Blue LED Pin as an output
  pinMode(tmp36Shutdwn,OUTPUT);                     // Supports shutting down the TMP-36 to save juice
  digitalWrite(tmp36Shutdwn, HIGH);                 // Turns on the temp sensor
  pinMode(donePin,OUTPUT);                          // Allows us to pet the watchdog
  pinMode(hardResetPin,OUTPUT);                     // For a hard reset active HIGH
  // Pressure Module Pin Setup
  pinMode(intPin,INPUT_PULLDOWN);                   // pressure sensor interrupt
  pinMode(disableModule,OUTPUT);                    // Turns on the module when pulled low
  pinResetFast(disableModule);                      // Turn on the module - send high to switch off board
  pinMode(ledPower,OUTPUT);                         // Turn on the lights
  pinSetFast(ledPower);                             // Turns on the LED on the pressure sensor board

  petWatchdog();                                    // Pet the watchdog - not necessary in a power on event but just in case
  attachInterrupt(wakeUpPin, watchdogISR, RISING);  // The watchdog timer will signal us and we have to respond

  char responseTopic[125];
  String deviceID = System.deviceID();              // Multiple Electrons share the same hook - keeps things straight
  deviceID.toCharArray(responseTopic,125);          // Puts the deviceID into the response topic array
  Particle.subscribe(responseTopic, UbidotsHandler, MY_DEVICES);      // Subscribe to the integration response event

  Particle.variable("HourlyCount", hourlyPersonCount);                // Define my Particle variables
  Particle.variable("DailyCount", dailyPersonCount);                  // Note: Don't have to be connected for any of this!!!
  Particle.variable("Signal", SignalString);
  Particle.variable("ResetCount", resetCount);
  Particle.variable("Temperature",temperatureF);
  Particle.variable("Release",releaseNumber);
  Particle.variable("stateOfChg", stateOfCharge);
  Particle.variable("lowPowerMode",lowPowerMode);
  Particle.variable("OpenTime",openTime);
  Particle.variable("CloseTime",closeTime);
  Particle.variable("Debounce",debounceStr);
  Particle.variable("MaxMinLimit",maxMinLimit);
  Particle.variable("Alerts",alerts);

  Particle.function("resetFRAM", resetFRAM);                          // These are the functions exposed to the mobile app and console
  Particle.function("resetCounts",resetCounts);
  Particle.function("HardReset",hardResetNow);
  Particle.function("SendNow",sendNow);
  Particle.function("LowPowerMode",setLowPowerMode);
  Particle.function("Solar-Mode",setSolarMode);
  Particle.function("Verbose-Mode",setVerboseMode);
  Particle.function("Set-Timezone",setTimeZone);
  Particle.function("Set-OpenTime",setOpenTime);
  Particle.function("Set-Close",setCloseTime);
  Particle.function("Set-Debounce",setDebounce);
  Particle.function("Set-MaxMin-Limit",setMaxMinLimit);

  // Load the elements for improving troubleshooting and reliability
  connectionEvents.setup();                                           // For logging connection event data
  batteryCheck.setup();

  // Load FRAM and reset variables to their correct values
  if (!fram.begin()) state = ERROR_STATE;                             // You can stick the new i2c addr in here, e.g. begin(0x51);
  else if (FRAMread8(FRAM::versionAddr) != versionNumber) {           // Check to see if the memory map in the sketch matches the data on the chip
    ResetFRAM();                                                      // Reset the FRAM to correct the issue
    if (FRAMread8(FRAM::versionAddr) != versionNumber)state = ERROR_STATE; // Resetting did not fix the issue
  }

  alerts = FRAMread8(FRAM::alertsCountAddr);                          // Load the alerts count
  resetCount = FRAMread8(FRAM::resetCountAddr);                       // Retrive system recount data from FRAM
  if (System.resetReason() == RESET_REASON_PIN_RESET || System.resetReason() == RESET_REASON_USER)  // Check to see if we are starting from a pin reset or a reset in the sketch
  {
    resetCount++;
    FRAMwrite8(FRAM::resetCountAddr,static_cast<uint8_t>(resetCount));// If so, store incremented number - watchdog must have done This
  }

  // Check and import values from FRAM
  debounce = 100*FRAMread8(FRAM::debounceAddr);
  if (debounce <= 100 || debounce > 2000) debounce = 500;             // We store debounce in dSec so mult by 100 for mSec
  snprintf(debounceStr,sizeof(debounceStr),"%2.1f sec",(debounce/1000.0));
  openTime = FRAMread8(FRAM::openTimeAddr);
  if (openTime < 0 || openTime > 22) openTime = 0;                    // Open and close in 24hr format
  closeTime = FRAMread8(FRAM::closeTimeAddr);
  if (closeTime < 1 || closeTime > 23) closeTime = 23;
  int8_t tempFRAMvalue = FRAMread8(FRAM::timeZoneAddr);
  if (tempFRAMvalue >= 12 || tempFRAMvalue <= -12)  Time.zone(-5);    // Default is EST in case proper value not in FRAM
  else Time.zone((float)tempFRAMvalue);                               // Load Timezone from FRAM
  maxMinLimit = FRAMread8(FRAM::maxMinLimitAddr);                     // This is the maximum number of counts in a minute
  if (maxMinLimit < 2 || maxMinLimit > 30) maxMinLimit = 10;          // If value has never been intialized - reasonable value


  controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);        // Read the Control Register for system modes
  lowPowerMode    = (0b00000001 & controlRegisterValue);              // Bitwise AND to set the lowPowerMode flag from control Register
  verboseMode     = (0b00001000 & controlRegisterValue);              // verboseMode
  solarPowerMode  = (0b00000100 & controlRegisterValue);              // solarPowerMode
  connectionMode  = (0b00010000 & controlRegisterValue);              // connected mode 1 = connected and 0 = disconnected

  PMICreset();                                                        // Executes commands that set up the PMIC for Solar charging

  currentHourlyPeriod = Time.hour();                                  // Sets the hour period for when the count starts (see #defines)
  currentDailyPeriod = Time.day();                                    // What day is it?

  time_t unixTime = FRAMread32(FRAM::currentCountsTimeAddr);          // Need to reload last recorded event - current periods set from this event
  dailyPersonCount = FRAMread16(FRAM::currentDailyCountAddr);         // Load Daily Count from memory
  hourlyPersonCount = FRAMread16(FRAM::currentHourlyCountAddr);       // Load Hourly Count from memory

  if (!digitalRead(userSwitch)) {                                     // Rescue mode to locally take lowPowerMode so you can connect to device
    lowPowerMode = false;                                             // Press the user switch while resetting the device
    connectionMode = true;                                            // Set the stage for the devic to get connected
    controlRegisterValue = (0b11111110 & controlRegisterValue);       // Turn off Low power mode
    controlRegisterValue = (0b00010000 | controlRegisterValue);       // Turn on the connectionMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);       // Write it to the register
    if ((Time.hour() > closeTime || Time.hour() < openTime))  {       // Device may also be sleeping due to time or TimeZone setting
      openTime = 0;                                                   // Only change these values if it is an issue
      FRAMwrite8(FRAM::openTimeAddr,0);                               // Reset open and close time values to ensure device is awake
      closeTime = 23;
      FRAMwrite8(FRAM::closeTimeAddr,23);
    }
  }

  // Here is where the code diverges based on why we are running Setup()
  // Deterimine when the last counts were taken check when starting test to determine if we reload values or start counts over
  if (currentDailyPeriod != Time.day(unixTime)) {
    resetEverything();                                                // Zero the counts for the new day
    if (solarPowerMode && !lowPowerMode) setLowPowerMode("1");        // If we are running on solar, we will reset to lowPowerMode at Midnight
  }
  if ((currentHourlyPeriod > closeTime || currentHourlyPeriod < openTime)) {}         // The park is closed - sleep
  else {                                                              // Park is open let's get ready for the day
    attachInterrupt(intPin, sensorISR, RISING);                       // Pressure Sensor interrupt from low to high
    if (connectionMode) connectToParticle();                          // Only going to connect if we are in connectionMode
    takeMeasurements();                                               // Populates values so you can read them before the hour
    stayAwake = stayAwakeLong;                                        // Keeps Electron awake after reboot - helps with recovery
  }

  pinResetFast(ledPower);                                             // Turns off the LED on the sensor board

  if (state == INITIALIZATION_STATE) {
    state = IDLE_STATE;                                               // IDLE unless otherwise from above code
    //System.enableUpdates();                                           // Allowing updates as we are heading off to the idle state
  }
}

void loop()
{
  switch(state) {
  case IDLE_STATE:                                                    // Where we spend most time - note, the order of these conditionals is important
    if (verboseMode && state != oldState) publishStateTransition();
    if (watchdogFlag) petWatchdog();                                  // Watchdog flag is raised - time to pet the watchdog
    if (sensorDetect) recordCount();                                  // The ISR had raised the sensor flag
    if (hourlyPersonCountSent) {                                      // Cleared here as there could be counts coming in while "in Flight"
      hourlyPersonCount -= hourlyPersonCountSent;                     // Confirmed that count was recevied - clearing
      FRAMwrite16(FRAM::currentHourlyCountAddr, static_cast<uint16_t>(hourlyPersonCount));  // Load Hourly Count to memory
      hourlyPersonCountSent = 0;                                      // Zero out the counts until next reporting period
      maxMin = 0;
      alerts = 0;
      FRAMwrite8(FRAM::alertsCountAddr,0);
      if (currentHourlyPeriod == 23) resetEverything();                // We have reported for the previous day - reset for the next - only needed if no sleep
    }
    if (lowPowerMode && (millis() - stayAwakeTimeStamp) > stayAwake) state = NAPPING_STATE;  // When in low power mode, we can nap between taps
    if (Time.hour() != currentHourlyPeriod) state = REPORTING_STATE;  // We want to report on the hour but not after bedtime
    if ((Time.hour() > closeTime || Time.hour() < openTime)) state = SLEEPING_STATE;   // The park is closed - sleep
    break;

  case SLEEPING_STATE: {                                              // This state is triggered once the park closes and runs until it opens
    if (verboseMode && state != oldState) publishStateTransition();
    detachInterrupt(intPin);                                          // Done sensing for the day
    pinSetFast(disableModule);                                        // Turn off the pressure module for the hour
    if (hourlyPersonCount) {                                          // If this number is not zero then we need to send this last count
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
    if (sensorDetect) break;                                   // Don't nap until we are done with event
    if ((0b00010000 & controlRegisterValue)) {                        // If we are in connected mode
      disconnectFromParticle();                                       // Disconnect from Particle
      controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);    // Get the control register (general approach)
      controlRegisterValue = (0b11101111 & controlRegisterValue);     // Turn off connected mode 1 = connected and 0 = disconnected
      connectionMode = false;
      FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);     // Write to the control register
    }
    stayAwake = debounce;                                             // Once we come into this function, we need to reset stayAwake as it changes at the top of the hour                                                 // Increment the wakes per hour count
    int wakeInSeconds = constrain(wakeBoundary - Time.now() % wakeBoundary, 1, wakeBoundary);
    petWatchdog();                                                    // Reset the watchdog
    System.sleep(intPin, RISING, wakeInSeconds);                      // Sensor will wake us with an interrupt or timeout at the hour
    if (sensorDetect) {
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
      //System.enableUpdates();                                         // Allowing updates as we are heading off to the idle state
      stayAwake = stayAwakeLong;                                      // Keeps Electron awake after reboot - helps with recovery
      stayAwakeTimeStamp = millis();
    }
    else if (millis() - webhookTimeStamp > webhookWait) {             // If it takes too long - will need to reset
      resetTimeStamp = millis();
      Particle.publish("spark/device/session/end", "", PRIVATE);      // If the device times out on the Webhook response, it will ensure a new session is started on next connect
      state = ERROR_STATE;                                            // Response timed out
    }
    break;

  case ERROR_STATE:                                                   // To be enhanced - where we deal with errors
    if (verboseMode && state != oldState) publishStateTransition();
    if (millis() > resetTimeStamp + resetWait)
    {
      if (resetCount <= 3) {                                          // First try simple reset
        if (Particle.connected()) Particle.publish("State","Error State - Reset", PRIVATE);    // Brodcast Reset Action
        delay(2000);
        System.reset();
      }
      else if (Time.now() - FRAMread32(FRAM::lastHookResponseAddr) > 7200L) { //It has been more than two hours since a sucessful hook response
        if (Particle.connected()) Particle.publish("State","Error State - Power Cycle", PRIVATE);  // Broadcast Reset Action
        delay(2000);
        FRAMwrite8(FRAM::resetCountAddr,0);                           // Zero the ResetCount
        digitalWrite(hardResetPin,HIGH);                              // This will cut all power to the Electron AND the carrier board
      }
      else {                                                          // If we have had 3 resets - time to do something more
        if (Particle.connected()) Particle.publish("State","Error State - Full Modem Reset", PRIVATE);            // Brodcase Reset Action
        delay(2000);
        FRAMwrite8(FRAM::resetCountAddr,0);                           // Zero the ResetCount
        fullModemReset();                                             // Full Modem reset and reboots
      }
    }
    break;
  }
  connectionEvents.loop();
  batteryCheck.loop();
}

void recordCount() // This is where we check to see if an interrupt is set when not asleep or act on a tap that woke the Arduino
{
  static int currentMinuteCount = 0;                                  // What is the count for the current minute
  static byte currentMinutePeriod;                                    // Current minute

  pinSetFast(blueLED);                                                // Turn on the blue LED

  if (millis() - currentEvent >= debounce || awokeFromNap) {          // If this event is outside the debounce time, proceed
    currentEvent = millis();
    awokeFromNap = false;                                             // Reset the awoke flag
    while(millis()-currentEvent < debounce) {                         // Keep us tied up here until the debounce time is up
      delay(10);
      Particle.process();                                             // Just in case debouce gets set to some big number
    }

    // Diagnostic code
    if (currentMinutePeriod != Time.minute()) {                       // Done counting for the last minute
      currentMinutePeriod = Time.minute();                            // Reset period
      currentMinuteCount = 1;                                         // Reset for the new minute
    }
    else currentMinuteCount++;

    if (currentMinuteCount >= maxMin) maxMin = currentMinuteCount;    // Save only if it is the new maxMin
    // End diagnostic code

    // Fix for multiple counts
    if (currentMinuteCount >= maxMinLimit) {
      hourlyPersonCount -= currentMinuteCount;
      dailyPersonCount -= currentMinuteCount;
      currentMinuteCount = 0;
      if (verboseMode && Particle.connected()) {
        meterParticlePublish();
        Particle.publish("Alert", "Exceeded Maxmin limit", PRIVATE);
      }
      alerts++;
      FRAMwrite8(FRAM::alertsCountAddr,alerts);                        // Save counts in case of reset
    }

    hourlyPersonCount++;                                                // Increment the PersonCount
    FRAMwrite16(FRAM::currentHourlyCountAddr, hourlyPersonCount);       // Load Hourly Count to memory
    dailyPersonCount++;                                                 // Increment the PersonCount
    FRAMwrite16(FRAM::currentDailyCountAddr, dailyPersonCount);         // Load Daily Count to memory
    FRAMwrite32(FRAM::currentCountsTimeAddr, Time.now());              // Write to FRAM - this is so we know when the last counts were saved
    if (verboseMode && Particle.connected()) {
      char data[256];                                                    // Store the date in this character array - not global
      snprintf(data, sizeof(data), "Count, hourly: %i, daily: %i",hourlyPersonCount,dailyPersonCount);
      meterParticlePublish();
      Particle.publish("Count",data, PRIVATE);                                   // Helpful for monitoring and calibration
    }
  }
  else if(verboseMode && Particle.connected()) {
    meterParticlePublish();
    Particle.publish("Event","Debounced", PRIVATE);
  }

  if (!digitalRead(userSwitch)) {                     // A low value means someone is pushing this button - will trigger a send to Ubidots and take out of low power mode
    connectToParticle();                              // Get connected to Particle
    meterParticlePublish();
    if (Particle.connected()) Particle.publish("Mode","Normal Operations", PRIVATE);
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);      // Load the control register
    controlRegisterValue = (0b1111110 & controlRegisterValue);        // Will set the lowPowerMode bit to zero
    controlRegisterValue = (0b00010000 | controlRegisterValue);       // Turn on the connectionMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);
    lowPowerMode = false;
    connectionMode = true;
  }

  pinResetFast(blueLED);
  sensorDetect = false;                                               // Reset the flag
}


void sendEvent()
{
  char data[256];                                                     // Store the date in this character array - not global
  snprintf(data, sizeof(data), "{\"hourly\":%i, \"daily\":%i,\"battery\":%i, \"temp\":%i, \"resets\":%i, \"alerts\":%i, \"maxmin\":%i}",hourlyPersonCount, dailyPersonCount, stateOfCharge, temperatureF, resetCount, alerts, maxMin);
  Particle.publish("Ubidots-Car-Hook", data, PRIVATE);
  webhookTimeStamp = millis();
  currentHourlyPeriod = Time.hour();                                  // Change the time period
  if(currentHourlyPeriod == 23) hourlyPersonCount++;                  // Ensures we don't have a zero here at midnigtt
  hourlyPersonCountSent = hourlyPersonCount;                          // This is the number that was sent to Ubidots - will be subtracted once we get confirmation
  dataInFlight = true;                                                // set the data inflight flag
}

void UbidotsHandler(const char *event, const char *data)              // Looks at the response from Ubidots - Will reset Photon if no successful response
{                                                                     // Response Template: "{{hourly.0.status_code}}" so, I should only get a 3 digit number back
  char dataCopy[strlen(data)+1];                                      // data needs to be copied since if (Particle.connected()) Particle.publish() will clear it
  strncpy(dataCopy, data, sizeof(dataCopy));                          // Copy - overflow safe
  if (!strlen(dataCopy)) {                                            // First check to see if there is any data
    if (Particle.connected()) Particle.publish("Ubidots Hook", "No Data", PRIVATE);
    return;
  }
  int responseCode = atoi(dataCopy);                                  // Response is only a single number thanks to Template
  if ((responseCode == 200) || (responseCode == 201))
  {
    if (Particle.connected()) Particle.publish("State","Response Received", PRIVATE);
    FRAMwrite32(FRAM::lastHookResponseAddr,Time.now());                     // Record the last successful Webhook Response
    dataInFlight = false;                                             // Data has been received
  }
  else if (Particle.connected()) Particle.publish("Ubidots Hook", dataCopy, PRIVATE);                    // Publish the response code
}

// These are the functions that are part of the takeMeasurements call
void takeMeasurements()
{
  if (Cellular.ready()) getSignalStrength();                          // Test signal strength if the cellular modem is on and ready
  getTemperature();                                                   // Get Temperature at startup as well
  stateOfCharge = int(batteryMonitor.getSoC());                       // Percentage of full charge
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

int getTemperature()
{
  int reading = analogRead(tmp36Pin);                                 //getting the voltage reading from the temperature sensor
  float voltage = reading * 3.3;                                      // converting that reading to voltage, for 3.3v arduino use 3.3
  voltage /= 4096.0;                                                  // Electron is different than the Arduino where there are only 1024 steps
  int temperatureC = int(((voltage - 0.5) * 100));                    //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100) - 5 degree calibration
  temperatureF = int((temperatureC * 9.0 / 5.0) + 32.0);              // now convert to Fahrenheit
  return temperatureF;
}

// Here are the various hardware and timer interrupt service routines
void sensorISR()
{
  sensorDetect = true;                              // sets the sensor flag for the main loop
}

void watchdogISR()
{
  watchdogFlag = true;
}

void petWatchdog()
{
  digitalWrite(donePin, HIGH);                                        // Pet the watchdog
  digitalWrite(donePin, LOW);
  watchdogFlag = false;
}

// Power Management function
void PMICreset() {
  power.begin();                                                      // Settings for Solar powered power management
  power.disableWatchdog();
  if (solarPowerMode) {
    power.setInputVoltageLimit(4840);                                 // Set the lowest input voltage to 4.84 volts best setting for 6V solar panels
    power.setInputCurrentLimit(900);                                  // default is 900mA
    power.setChargeCurrent(0,0,1,0,0,0);                              // default is 512mA matches my 3W panel
    power.setChargeVoltage(4208);                                     // Allows us to charge cloe to 100% - battery can't go over 45 celcius
  }
  else  {
    power.setInputVoltageLimit(4208);                                 // This is the default value for the Electron
    power.setInputCurrentLimit(1500);                                 // default is 900mA this let's me charge faster
    power.setChargeCurrent(0,1,1,0,0,0);                              // default is 2048mA (011000) = 512mA+1024mA+512mA)
    power.setChargeVoltage(4112);                                     // default is 4.112V termination voltage
  }
}

 // These are the particle functions that allow you to configure and run the device
 // They are intended to allow for customization and control during installations
 // and to allow for management.

bool connectToParticle() {
  //System.disableUpdates();                                             // Disabling updates here so we can complete any data transfers in process - will enable when we get to IDLE
  Cellular.on();
  Particle.connect();
  // wait for *up to* 5 minutes
  for (int retry = 0; retry < 300 && !waitFor(Particle.connected,1000); retry++) {
    if(sensorDetect) recordCount(); // service the interrupt every 10 seconds
    Particle.process();
  }
  if (Particle.connected()) {
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);      // Get the control register (general approach)
    controlRegisterValue = (0b00010000 | controlRegisterValue);       // Turn on connected mode 1 = connected and 0 = disconnected
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);       // Write to the control register
    return 1;                               // Were able to connect successfully
  }
  else {
    return 0;                                                    // Failed to connect
  }
}

bool disconnectFromParticle()                                     // Ensures we disconnect cleanly from Particle
{
  Particle.disconnect();
  waitFor(notConnected, 15000);                                   // make sure before turning off the cellular modem
  Cellular.off();
  delay(2000);                                                    // Bummer but only should happen once an hour
  return true;
}

bool notConnected() {                                             // Companion function for disconnectFromParticle
    return !Particle.connected();
}

int resetFRAM(String command)                                         // Will reset the local counts
{
  if (command == "1")
  {
    ResetFRAM();
    return 1;
  }
  else return 0;
}

int resetCounts(String command)                                       // Resets the current hourly and daily counts
{
  if (command == "1")
  {
    FRAMwrite16(FRAM::currentDailyCountAddr, 0);                      // Reset Daily Count in memory
    FRAMwrite16(FRAM::currentHourlyCountAddr, 0);                     // Reset Hourly Count in memory
    FRAMwrite8(FRAM::resetCountAddr,0);                               // If so, store incremented number - watchdog must have done This
    FRAMwrite8(FRAM::alertsCountAddr,0);
    alerts = 0;
    resetCount = 0;
    hourlyPersonCount = 0;                                            // Reset count variables
    dailyPersonCount = 0;
    hourlyPersonCountSent = 0;                                        // In the off-chance there is data in flight
    dataInFlight = false;
    return 1;
  }
  else return 0;
}

int hardResetNow(String command)                                      // Will perform a hard reset on the Electron
{
  if (command == "1")
  {
    digitalWrite(hardResetPin,HIGH);                                  // This will cut all power to the Electron AND the carrir board
    return 1;                                                         // Unfortunately, this will never be sent
  }
  else return 0;
}

int setDebounce(String command)                                       // This is the amount of time in seconds we will wait before starting a new session
{
  char * pEND;
  float inputDebounce = strtof(command,&pEND);                        // Looks for the first float and interprets it
  if ((inputDebounce < 0.0) | (inputDebounce > 5.0)) return 0;        // Make sure it falls in a valid range or send a "fail" result
  debounce = int(inputDebounce*1000);                                 // debounce is how long we must space events to prevent overcounting
  int debounceFRAM = constrain(int(inputDebounce*10),1,255);          // Store as a byte in FRAM = 1.6 seconds becomes 16 dSec
  FRAMwrite8(FRAM::debounceAddr,static_cast<uint8_t>(debounceFRAM));        // Convert to Int16 and store
  snprintf(debounceStr,sizeof(debounceStr),"%2.1f sec",inputDebounce);
  if (verboseMode && Particle.connected()) {                                                  // Publish result if feeling verbose
    waitUntil(meterParticlePublish);
    Particle.publish("Debounce",debounceStr, PRIVATE);
  }
  return 1;                                                           // Returns 1 to let the user know if was reset
}

int sendNow(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    state = REPORTING_STATE;
    return 1;
  }
  else return 0;
}

void resetEverything() {                                            // The device is waking up in a new day or is a new install
  FRAMwrite16(FRAM::currentDailyCountAddr, 0);                      // Reset the counts in FRAM as well
  FRAMwrite16(FRAM::currentHourlyCountAddr, 0);
  FRAMwrite32(FRAM::currentCountsTimeAddr,Time.now());              // Set the time context to the new day
  FRAMwrite8(FRAM::resetCountAddr,0);
  FRAMwrite8(FRAM::alertsCountAddr,0);
  FRAMwrite8(FRAM::alertsCountAddr,0);
  hourlyPersonCount = dailyPersonCount = resetCount = alerts = 0;   // Reset everything for the day
}

int setSolarMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    solarPowerMode = true;
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b00000100 | controlRegisterValue);          // Turn on solarPowerMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);               // Write it to the register
    PMICreset();                                               // Change the power management Settings
    if (Particle.connected()) Particle.publish("Mode","Set Solar Powered Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    solarPowerMode = false;
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b11111011 & controlRegisterValue);           // Turn off solarPowerMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);                // Write it to the register
    PMICreset();                                                // Change the power management settings
    if (Particle.connected()) Particle.publish("Mode","Cleared Solar Powered Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

int setVerboseMode(String command) // Function to force sending data in current hour
{
  if (command == "1")
  {
    verboseMode = true;
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b00001000 | controlRegisterValue);                    // Turn on verboseMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);                        // Write it to the register
    if (Particle.connected()) Particle.publish("Mode","Set Verbose Mode", PRIVATE);
    return 1;
  }
  else if (command == "0")
  {
    verboseMode = false;
    controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);
    controlRegisterValue = (0b11110111 & controlRegisterValue);                    // Turn off verboseMode
    FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);                        // Write it to the register
    if (Particle.connected()) Particle.publish("Mode","Cleared Verbose Mode", PRIVATE);
    return 1;
  }
  else return 0;
}

int setTimeZone(String command)
{
  char * pEND;
  char data[256];
  time_t t = Time.now();
  int8_t tempTimeZoneOffset = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTimeZoneOffset < -12) | (tempTimeZoneOffset > 12)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  Time.zone((float)tempTimeZoneOffset);
  FRAMwrite8(FRAM::timeZoneAddr,tempTimeZoneOffset);                             // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Time zone offset %i",tempTimeZoneOffset);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  delay(1000);
  if (Particle.connected()) Particle.publish("Time",Time.timeStr(t), PRIVATE);
  return 1;
}

int setOpenTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  openTime = tempTime;
  FRAMwrite8(FRAM::openTimeAddr,openTime);                             // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Open time set to %i",openTime);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  return 1;
}

int setCloseTime(String command)
{
  char * pEND;
  char data[256];
  int tempTime = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempTime < 0) || (tempTime > 23)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  closeTime = tempTime;
  FRAMwrite8(FRAM::closeTimeAddr,closeTime);                             // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "Closing time set to %i",closeTime);
  if (Particle.connected()) Particle.publish("Time",data, PRIVATE);
  return 1;
}


int setLowPowerMode(String command)                                   // This is where we can put the device into low power mode if needed
{
  if (command != "1" && command != "0") return 0;                     // Before we begin, let's make sure we have a valid input
  controlRegisterValue = FRAMread8(FRAM::controlRegisterAddr);        // Get the control register (general approach)
  if (command == "1")                                                 // Command calls for setting lowPowerMode
  {
    if (verboseMode && Particle.connected()) {
      waitUntil(meterParticlePublish);
      Particle.publish("Mode","Low Power", PRIVATE);
    }
    if ((0b00010000 & controlRegisterValue)) {                        // If we are in connected mode
      Particle.disconnect();                                          // Otherwise Electron will attempt to reconnect on wake
      controlRegisterValue = (0b11101111 & controlRegisterValue);     // Turn off connected mode 1 = connected and 0 = disconnected
      connectionMode = false;
      Cellular.off();
      delay(1000);                                                    // Bummer but only should happen once an hour
    }
    controlRegisterValue = (0b00000001 | controlRegisterValue);       // If so, flip the lowPowerMode bit
    lowPowerMode = true;
  }
  else if (command == "0")                                            // Command calls for clearing lowPowerMode
  {
    if (verboseMode && Particle.connected()) {
      waitUntil(meterParticlePublish);
      Particle.publish("Mode","Normal Operations", PRIVATE);
    }
    if (!(0b00010000 & controlRegisterValue)) {
      controlRegisterValue = (0b00010000 | controlRegisterValue);    // Turn on connected mode 1 = connected and 0 = disconnected
      Particle.connect();
      waitFor(Particle.connected,60000);                             // Give us 60 seconds to connect
      Particle.process();
    }
    controlRegisterValue = (0b11111110 & controlRegisterValue);      // If so, flip the lowPowerMode bit
    lowPowerMode = false;
  }
  FRAMwrite8(FRAM::controlRegisterAddr,controlRegisterValue);        // Write to the control register
  return 1;
}

int setMaxMinLimit(String command)
{
  char * pEND;
  char data[256];
  int tempMaxMinLimit = strtol(command,&pEND,10);                       // Looks for the first integer and interprets it
  if ((tempMaxMinLimit < 2) || (tempMaxMinLimit > 30)) return 0;   // Make sure it falls in a valid range or send a "fail" result
  maxMinLimit = tempMaxMinLimit;
  FRAMwrite8(FRAM::maxMinLimitAddr,maxMinLimit);                             // Store the new value in FRAMwrite8
  snprintf(data, sizeof(data), "MaxMin limit set to %i",maxMinLimit);
  if (Particle.connected()) Particle.publish("MaxMin",data,PRIVATE);
  return 1;
}

bool meterParticlePublish(void)
{
  static unsigned long lastPublish=0;                                 // Initialize and store value here
  if(millis() - lastPublish >= publishFrequency) {
    lastPublish = millis();
    return 1;
  }
  else return 0;
}

void publishStateTransition(void)
{
  char stateTransitionString[40];
  snprintf(stateTransitionString, sizeof(stateTransitionString), "From %s to %s", stateNames[oldState],stateNames[state]);
  oldState = state;
  if(Particle.connected()) {
    waitUntil(meterParticlePublish);
    Particle.publish("State Transition",stateTransitionString, PRIVATE);
  }
  Serial.println(stateTransitionString);
}

void fullModemReset() {  // Adapted form Rikkas7's https://github.com/rickkas7/electronsample
	Particle.disconnect(); 	                                         // Disconnect from the cloud
	unsigned long startTime = millis();  	                           // Wait up to 15 seconds to disconnect
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

  waitUntil(meterParticlePublish);
  Particle.publish("Daily Cleanup","Running", PRIVATE);               // Make sure this is being run

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
