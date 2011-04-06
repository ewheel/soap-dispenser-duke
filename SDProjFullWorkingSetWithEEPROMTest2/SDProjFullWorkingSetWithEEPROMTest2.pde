#include <NewSoftSerial.h>
#include <Wire.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#define DEBUG 0
#define DEBUG2 0
#define DIAGNOSTIC_STATE 1
#define MSG_CRC_INIT 0xFFFF
#define MSG_CCITT_CRC_POLY 0x1021
#define M5E_STATUS1  3
#define M5E_STATUS2  4
#define M5E_LENGTH  1
#define M5E_OPCODE  2
#define RFID_POWER_PIN 11
#define USB_POWER_PIN 10
#define USB_RESET_PIN 12
#define MONITOR_LED_PIN 13
#define EEPROM_DATA_SEGMENT_START 64
#define USB_RX_PIN 6
#define USB_TX_PIN 7
#define RFID_RX_PIN 4
#define RFID_TX_PIN 5
#define EEPROM_DEVICE_ADDRESS 104
#define RTS_USB_DEVICE 8
#define LEVER_INTERRUPT_PIN 2
#define USB_INTERRUPT_PIN 3
#define USB_TEST_PIN 9
#define USB_CONFIRMATION_TIMEOUT 10000
#define MAX_SETUP_TRIES 10

#define disk1 0x50
#define DISP_FILENAME "disp1.csv"
#define dprint(...) do{if(DEBUG){Serial.print(__VA_ARGS__);}}while(0)
#define dprintln(...) do{if(DEBUG){Serial.println(__VA_ARGS__);}}while(0)
#define d2print(...) do{if(DEBUG2){Serial.print(__VA_ARGS__);}}while(0)
#define d2println(...) do{if(DEBUG2){Serial.println(__VA_ARGS__);}}while(0)

//Set constants.
int readPwr = 3000;
volatile int state2 = LOW;
volatile int state4 = LOW;
NewSoftSerial usb(USB_RX_PIN,USB_TX_PIN);        //Software serial object talks to the usb stick.
NewSoftSerial rfid(RFID_RX_PIN,RFID_TX_PIN,false); //Software serial object talks to the RFID reader.
volatile int eventNum = 0;
volatile unsigned int externalAddress = EEPROM_DATA_SEGMENT_START;

////*********************************************************************////


void setup()
{
  Wire.begin();
  pinMode(MONITOR_LED_PIN, OUTPUT);                 //Allow LED out.
  pinMode(USB_RESET_PIN, OUTPUT);                 //VDrive Reset Pin.
//  pinMode(USB_RX_PIN,INPUT);                    //Software RX
//  pinMode(USB_TX_PIN,OUTPUT);                   //Software TX
//  pinMode(RTS_USB_DEVICE,INPUT);  //RTS from VDrive2
  pinMode(USB_POWER_PIN,OUTPUT); //USB POWER
  pinMode(LEVER_INTERRUPT_PIN,INPUT);
  pinMode(USB_INTERRUPT_PIN,INPUT);
  pinMode(RFID_POWER_PIN,OUTPUT); //RFID Power
  pinMode(USB_TEST_PIN,OUTPUT); //USB Test Pin

  digitalWrite(USB_RESET_PIN, HIGH);
  digitalWrite(USB_POWER_PIN,LOW);  //TURN USB ON.
  digitalWrite(MONITOR_LED_PIN,LOW);  //Turn LED OFF.

//  attachInterrupt(0, stateTest, LOW);   //Set up interrupt for button press.
//  attachInterrupt(1, diskInsert, LOW);   //Set up interrupt for button press.

  Serial.begin(9600);                  //Set up serial connection to computer.
  delay(2000);
  dprintln("Started");

  usb.begin(9600);                     //Set up software serial to the USB stick.
  usb.flush();
  delay(2000);
  resetVNC();
  usb.flush();
  delay(100);
  usb.print("WKD");
  usb.print(13,BYTE);
  usb.flush();
  delay(100);
  usb.print("IPA");
  usb.print(13,BYTE);
  delay(2000);

//  initializeRTC();
  digitalWrite(USB_POWER_PIN,HIGH); //TURN USB OFF.
  digitalWrite(RFID_POWER_PIN,HIGH); //TURN RFID OFF.
}

void loop()
{
  digitalWrite(MONITOR_LED_PIN,HIGH);  //Toggle LED to opposite state for visual reference.

  //If the button has been pressed, initiate communications.
  if(state2 == HIGH)
  {
    digitalWrite(RFID_POWER_PIN,LOW);
    delay(100);
    //Get the RFID tag from the reader.
    byte rfidTag[12];
    retrieveRFIDInfo(rfidTag);
    digitalWrite(RFID_POWER_PIN,HIGH);

    Wire.beginTransmission(EEPROM_DEVICE_ADDRESS);
    Wire.send(0x00);
    Wire.endTransmission();    //These 3 lines clear out the RTC to prep it for communication.

    //Obtain all the time information from the DS1307 chip.
    Wire.requestFrom(EEPROM_DEVICE_ADDRESS,7);
    int second = Wire.receive() & 0x7f;
    int minute = Wire.receive();      
    int hour = Wire.receive() & 0x3f;    
    int day_of_week=Wire.receive();   
    int day = Wire.receive();        
    int month = Wire.receive();       
    int year = Wire.receive();

    //Convert the BCD to base 10.
    minute = ((minute/16) * 10) + (minute % 16);
    day = ((day/16) * 10) + (day % 16);
    month = ((month/16) * 10) + (month % 16);
    year = ((year/16) * 10) + (year % 16);
    second = ((second/16) * 10) + (second % 16);
    hour = ((hour/16) * 10) + (hour % 16);

    //Print the output to the command line for debug.
    dprint("Year: ");  
    dprintln(year);
    dprint("Month: ");  
    dprintln(month);
    dprint("Day: ");  
    dprintln(day);
    dprint("Day of week: ");  
    dprintln(day_of_week);
    dprint("hour: ");  
    dprintln(hour);
    dprint("minute: ");  
    dprintln(minute);
    dprint("second: ");  
    dprintln(second);
    state2 = !state2;        //Change the state back to idle.

    d2print(eventNum);
    d2print(",");
    d2print(day);
    d2print("/");
    d2print(month);
    d2print("/");
    d2print(year);
    d2print(",");
    d2print(hour);
    d2print(":");
    d2print(minute);
    d2print(":");
    d2println(second);

    //Write the data entry to persistent storage.
    writeEEPROM(disk1, externalAddress++,month);
    writeEEPROM(disk1, externalAddress++,day);
    writeEEPROM(disk1, externalAddress++,year);
    writeEEPROM(disk1, externalAddress++,hour);
    writeEEPROM(disk1, externalAddress++,minute);
    writeEEPROM(disk1, externalAddress++,second);
    for(int i = 0; i < 12; i++)
    {
      writeEEPROM(disk1, externalAddress++, rfidTag[i]);
    }
    eventNum++;
    //    writeEEPROM(disk1, 0, eventNum);
  }

  //Interrupt handling for the usb drive interrupt button.
  if(state4 == HIGH)
  {
    state4 = !state4;
    dprintln(eventNum);
//    eventNum = 100;  //Set number of things to print manually.
    resetVNC();
    usb.flush();
    delay(100);
    writeUSBNoParam("WKD");
    writeUSBNoParam("IPA");
    delay(5000);
    writeUSBParam("OPW ", DISP_FILENAME);
    dprintln("Opened File");

    int addressCounter = EEPROM_DATA_SEGMENT_START;
    for(int i = 0; i < eventNum; i++)
    {
      int numChars = 1;
      int x = i;
      while (x>= 10)
      {                
        numChars++;                
        x/=10;    
      }
      char buf3[numChars];
      sprintf(buf3, "%0d", i);
      dprint(numChars);
      dprintln(buf3);
      writeDataToUSBDrive(numChars, buf3);

      //Write the date to disk.
      numChars = 5;
      byte month = readEEPROM(disk1,addressCounter++);
      byte day = readEEPROM(disk1, addressCounter++);
      byte year = readEEPROM(disk1,addressCounter++);
      numChars += addBytes(month);
      numChars += addBytes(day);
      numChars += addBytes(year);
      char buf[numChars];
      sprintf(buf, ",%0d/%0d/20%0d", month,day,year);
      writeDataToUSBDrive(numChars, buf);

      //Write the time to disk.
      numChars = 4;
      byte hour = readEEPROM(disk1,addressCounter++);
      byte minute = readEEPROM(disk1, addressCounter++);
      byte second = readEEPROM(disk1,addressCounter++);
      numChars += addBytes(hour);
      numChars += addBytes(minute);
      numChars += addBytes(second);
      char buf2[numChars];
      sprintf(buf2, ",%0d:%0d:%0d,", hour,minute,second);
      writeDataToUSBDrive(numChars, buf2);

      //Write the RFID tag to disk byte by byte (might try condensing this loop to save time.
      char buf5[26];
      numChars = 25;
      for(int j = 0; j < 12; j++)
      {
        byte tagID = readEEPROM(disk1,addressCounter++);
        buf5[2*j] = getFirstByte(tagID);
        buf5[(2*j)+1] = getSecondByte(tagID);
      }
      buf5[24] = 13;
      buf5[25] = 0;
      writeDataToUSBDrive(numChars, buf5);
    }
    dprintln("Done Writing");
    writeUSBParam("CLF ", DISP_FILENAME);
    dprintln("Closed File.");
    dprintln("DONE");
    digitalWrite(USB_POWER_PIN,HIGH);
  }
  digitalWrite(MONITOR_LED_PIN,LOW);
//  attachInterrupt(0, stateTest, LOW);
//  attachInterrupt(1, diskInsert, LOW);
  sleepNow();
}


byte getFirstByte(byte inByte)
{
  byte first;
  if(inByte>>4 > 9)
  {
    first = 55 + (inByte>>4);
  }
  else
  {
    first = 48 + (inByte>>4);
  }
  return first;
}

byte getSecondByte(byte inByte)
{
  byte second;
  if((inByte & 0x0F) > 9)
  {
    second = 55 + (inByte & 0x0F);
  }
  else
  {
    second = 48 + (inByte & 0x0F);
  }
  return second;
}

void verifyWrite()
{
  unsigned long start2 = millis();
  unsigned long endtime2 = start2 + USB_CONFIRMATION_TIMEOUT;

  while(endtime2 < start2)
  {
    start2 = millis();
    endtime2 = start2 + USB_CONFIRMATION_TIMEOUT;
  }
  byte z = 0;
  delay(10);
  unsigned long comparison = millis();
  while ((comparison < endtime2) && (comparison > start2))
  {
    z = VNC1_Confirmation();
    if(z)
    {
      break;
    }
    comparison = millis();
  }
  if(z)
  {
    return;
  }
  else
  {
    dprintln("Failed");
    digitalWrite(USB_POWER_PIN,HIGH);
    digitalWrite(MONITOR_LED_PIN,LOW);
//    attachInterrupt(0, stateTest, LOW);
//    attachInterrupt(1, diskInsert, LOW);
    sleepNow();
  }
}

void writeUSBNoParam(char* command)
{
  usb.flush();
  delay(100);
  digitalWrite(USB_TEST_PIN,HIGH);
  usb.print(command);
  usb.print(13,BYTE);
  delay(100);
  digitalWrite(USB_TEST_PIN,LOW);
  verifyWrite();
  //  while (VNC1_Confirmation()==0);
}

void writeUSBParam(char* command, char* parameter)
{
  usb.flush();
  delay(100);
  digitalWrite(USB_TEST_PIN,HIGH);
  usb.print(command);
  usb.print(parameter);
  usb.print(13,BYTE);
  delay(100);
  digitalWrite(USB_TEST_PIN,LOW);
  verifyWrite();
  //  while (VNC1_Confirmation()==0);
}

void writeDataToUSBDrive(int length, char* toPrint)
{
  usb.flush();
//  delay(100);
  digitalWrite(USB_TEST_PIN,HIGH);
  usb.print("WRF ");
  usb.print(length, DEC);
  usb.print(13,BYTE);
  usb.print(toPrint);
  delay(25);
  digitalWrite(USB_TEST_PIN,LOW);
  //  verifyWrite();
  unsigned long start = millis();
  unsigned long endtime = start + USB_CONFIRMATION_TIMEOUT;

  while(endtime < start)
  {
    start = millis();
    endtime = start + USB_CONFIRMATION_TIMEOUT;
  }
  while (VNC1_Confirmation()==0 && millis() < endtime && millis() > start);
}

//Set up state machine states on interrupt for soap dispenser lever.
void stateTest()
{
//  detachInterrupt(0);
//  detachInterrupt(1);
  state2 = !state2;
}

//Set up state machine states on interrupt for usb.
void diskInsert()
{
//  detachInterrupt(0);
//  detachInterrupt(1);
  state4 = !state4;
}

//Hard reset for the VNC chip.
void resetVNC()
{
  digitalWrite(USB_POWER_PIN,LOW);
  delay(6000);
  digitalWrite(USB_RESET_PIN, LOW);
  delay(100);
  digitalWrite(USB_RESET_PIN,HIGH);
}

//Discover if VNC is responding or not.
byte VNC1_Confirmation(){  

  byte state=0;
  byte byte_received;
  byte confirm=0;
  unsigned long start = millis();
  unsigned long endtime = start + USB_CONFIRMATION_TIMEOUT;

  while(endtime < start)
  {
    start = millis();
    endtime = start + USB_CONFIRMATION_TIMEOUT;
  }

  while (usb.available()<5 && millis() < endtime && millis() > start) ;

  dprintln(".");

  while (usb.available()){
    byte_received=usb.read();
    dprint(byte_received);
    if(byte_received == 67)
    {
      while(usb.available())
      {
        usb.read();
      }
      usb.flush();
      delay(100);
      return 0;
    }

    if (byte_received==68 && state==0){
      //Received "D"
      state=1;
    }
    else if (byte_received==58 && state==1){
      //Received ":"  
      state=2;
    }
    else if (byte_received==92 && state==2){
      //Received "\"  
      state=3;
    }
    else if (byte_received==62 && state==3){
      //Received ">"
      state=4;
    }
    else if (byte_received==13 && state==4){
      //Received "CR"
      state=0;
      confirm=1;
      break;
    }
    else{
      confirm=0;
      break;
    }
  }  
  dprint("Confirmed = ");
  dprintln(confirm, DEC);
  delay(16);
  return confirm;
}  

//Figure out how many bytes are in a hex string.
int addBytes(int num)
{
  if(num<10)
  {
    return 1;
  }
  else
  {
    return 2;
  }
}

//Put system to sleep.
void sleepNow()         // here we put the arduino to sleep
{
  /* Now is the time to set the sleep mode. In the Atmega8 datasheet
   * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
   * there is a list of sleep modes which explains which clocks and 
   * wake up sources are available in which sleep mode.
   *
   * In the avr/sleep.h file, the call names of these sleep modes are to be found:
   *
   * The 5 different modes are:
   *     SLEEP_MODE_IDLE         -the least power savings 
   *     SLEEP_MODE_ADC
   *     SLEEP_MODE_PWR_SAVE
   *     SLEEP_MODE_STANDBY
   *     SLEEP_MODE_PWR_DOWN     -the most power savings
   *
   * For now, we want as much power savings as possible, so we 
   * choose the according 
   * sleep mode: SLEEP_MODE_PWR_DOWN
   * 
   */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
 
  sleep_enable();          // enables the sleep bit in the mcucr register
  // so sleep is possible. just a safety pin 

  /* Now it is time to enable an interrupt. We do it here so an 
   * accidentally pushed interrupt button doesn't interrupt 
   * our running program. if you want to be able to run 
   * interrupt code besides the sleep function, place it in 
   * setup() for example.
   * 
   * In the function call attachInterrupt(A, B, C)
   * A   can be either 0 or 1 for interrupts on pin 2 or 3.   
   * 
   * B   Name of a function you want to execute at interrupt for A.
   *
   * C   Trigger mode of the interrupt pin. can be:
   *             LOW        a low level triggers
   *             CHANGE     a change in level triggers
   *             RISING     a rising edge of a level triggers
   *             FALLING    a falling edge of a level triggers
   *
   * In all but the IDLE sleep modes only LOW can be used.
   */
  digitalWrite(USB_RESET_PIN,LOW);
//  digitalWrite(USB_RX_PIN,LOW);
//  digitalWrite(USB_TX_PIN,LOW);
  digitalWrite(RFID_RX_PIN,LOW);
  digitalWrite(RFID_TX_PIN,LOW);
  attachInterrupt(0, stateTest, LOW);
  attachInterrupt(1, diskInsert, LOW);
  sleep_mode();            // here the device is actually put to sleep!!
  // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
  sleep_disable();         // first thing after waking from sleep:
  // disable sleep...
  detachInterrupt(0);
  detachInterrupt(1);
  digitalWrite(USB_RESET_PIN,HIGH);
}

void retrieveRFIDInfo(byte* result)
{
  int worked = 1;
  int trials = 0;
  while((worked != 0) && trials < MAX_SETUP_TRIES)
  {
    worked = RFIDSetup();
    trials++;
  }
  if(trials == MAX_SETUP_TRIES - 1)
  {
    return;
  }
  dprintln("Querying!");
  rfid.flush();
  delayMicroseconds(400);
  unsigned long start = millis();
  unsigned long endtime = start + USB_CONFIRMATION_TIMEOUT;

  while(endtime < start)
  {
    start = millis();
    endtime = start + USB_CONFIRMATION_TIMEOUT;
  }
//  Serial.println("*");
  delay(1);
  unsigned long comparison = millis();
  while(((comparison < endtime) && (comparison > start)))
  {
    if(QueryEnvironment(result, 50))
    {
//      Serial.println("+");
      if(DIAGNOSTIC_STATE)
      {
        indicateCorrectRead();
      }
      return;
    }
    comparison = millis();
    delay(100);
  } 
  for(int i = 0; i<12;i++)
  {
    result[i] = i+65;
  }
}

void indicateCorrectRead()
{
//  Serial.println(".");
  digitalWrite(MONITOR_LED_PIN, LOW);
  delay(100);
  digitalWrite(MONITOR_LED_PIN,HIGH);
  delay(100);
  digitalWrite(MONITOR_LED_PIN,LOW);
  delay(100);
  digitalWrite(MONITOR_LED_PIN,HIGH);
}

//This function updates a running CRC value
void CRC_calcCrc8(unsigned int* crc_calc, byte ch) 
{
  byte i, v, xor_flag;

  //Align test bit with leftmost bit of the MsgObj byte
  v = 0x80;
  for (int i = 0; i < 8; i++) 
  {
    if (*crc_calc & 0x8000)
    {
      xor_flag = 1;
    }
    else
    {
      xor_flag = 0;
    }
    *crc_calc = *crc_calc << 1;
    if (ch & v)
    {
      *crc_calc = *crc_calc + 1;
    }
    if (xor_flag)
    {
      *crc_calc = *crc_calc ^ MSG_CCITT_CRC_POLY;
    }
    //Align test bit with next bit of the MsgObj byte.  
    v = v >> 1;
  }
}


boolean validateCRC(byte* buf, int n) {
  //TODO: Fix this (it doesn't work right now)
  if (n < 7)
  {
    return false;
  }
  unsigned int crcReg = MSG_CRC_INIT;
  for (int i = 0; i < n; i++) 
  {
    CRC_calcCrc8(&crcReg, buf[i]);
  }
  //This is the CRC returned from the reader
  unsigned int thiscrc = ((buf[n - 1] & 0x00FF) << 8) | (buf[n - 2] & 0x00FF);
  //printf("%x %x", buf[n -2], buf[n - 1]);
  boolean equal = (thiscrc == crcReg);
  if (!equal) 
  {
    Serial.println("Warning: CRC check failed");
  }
  return equal;
}

void sendMessage(byte command, byte* data, byte length) 
{
  rfid.flush();
  byte bytes[256];
  bytes[0] = 0xFF;
  bytes[1] = length;
  bytes[2] = command;
  for (int i = 3; i < length+5 && (i - 3) < length; i++) 
  {
    bytes[i] = data[i - 3];
  }
  unsigned int crc = CRC_calcCrcMsgObj(length, command, data); 
  byte lo = (byte)(crc & 0x00FF);
  byte hi = (byte)((crc & 0xFF00) >> 8);
  bytes[length+5 - 2] = hi;
  bytes[length+5 - 1] = lo;
  dprint("Sent: ");
  for(int i = 0; i < length+5; i++)
  {
    dprint("x");
    dprint(bytes[i],HEX);
    rfid.print(bytes[i],BYTE);
    delayMicroseconds(10);
  }
  dprintln();
  delayMicroseconds(0);
}

unsigned int CRC_calcCrcMsgObj(byte length, byte opCode, byte* data) 
{
  unsigned int crcReg, i;
  crcReg = MSG_CRC_INIT;
  CRC_calcCrc8(&crcReg, length);
  CRC_calcCrc8(&crcReg, opCode);
  for (int i = 0; i < length; i++) 
  {
    CRC_calcCrc8(&crcReg, data[i]);
  }
  return crcReg;
}



//[SOH 1 byte] [length 1 byte] [op code 1 byte] [status 2 bytes] [data n bytes] [CRC 2 bytes]
int readMessage(byte* data, int length, int timeout) 
{
  memset(data, 0x00, length);//Clear the previous contents from the buffer
  int minlength = 7;
  data[0] = 0xFF;
  int n = 1;
  unsigned long start = millis();
  unsigned long endtime = start + timeout + 50;
  while(endtime < start)
  {
    start = millis();
    endtime = start + timeout + 50;
  }

  while(rfid.available() || ((millis() < endtime) && (millis() > start)))
  {

    int k = rfid.read();
    if(k == 0xFF && n == 1)
    {
      continue;
    }
    if(k != 0xFFFF)
    {
      data[n] = k;
      n += 1;
    }

  }
  //validateCRC(data, n);
  dprintln(n);
  for(int i = 0; i < n; i++)
  {
    dprint("x");
    dprint(data[i],HEX);
  }
  dprintln();
  return n;
}

boolean checkSuccess(byte* buf, int n) 
{
  if (n < 7) 
  {
    return false;
  }
  else
  {
    return ((((buf[3] << 8) & 0xFF00) | (0x00FF & buf[4])) == 0x0000);
  }
}

boolean checkBootFirmwareVersion() 
{
  byte buf[256];
  sendMessage(0x03, NULL, 0);//"Get bootloader firmware version number"
  int n = readMessage(buf, 256, 0);
  return checkSuccess(buf, n);
}

boolean bootFirmware() 
{
  byte buf[256];
  //Boot into Firmware
  dprint("\tBooting into firmware\n");
  sendMessage(0x04, NULL, 0);
  delay(20);
  int n = readMessage(buf, 256, 0);
  if(checkSuccess(buf,n))
  {
    return true;
  }
  // Non-Zero Response will be received if the reader has already booted into firmware
  //   This occurs when you've already powered-up & previously configured the reader.  
  //   Can safely ignore this problem and continue initialization
  if (((((buf[3] << 8) & 0xFF00) | (0x00FF & buf[4])) == 0x0101)) //This actually means "invalid opcode"
  {
    return true;
  }
  return false;
}

boolean ChangeAntennaPorts(byte TXport, byte RXport) 
{
  byte buf[256];
  byte data[2] = {TXport, RXport};
  sendMessage(0x91, data, 2);
  delay(20);
  int n = readMessage(buf, 256, 0);
  return checkSuccess(buf, n);
}

boolean ChangeTXReadPower(unsigned int r) 
{
  byte buf[256];
  unsigned int readPwrF = r;
  byte hi = (readPwrF & 0xFFFF) >> 8;
  byte lo = (readPwrF & 0x00FF);
  byte data[2] = {hi, lo};
  sendMessage(0x92, data, 2);
  delay(20);
  int n = readMessage(buf, 256, 0);
  return checkSuccess(buf, n);
}

//Set protocol to gen2
boolean setProtocol() 
{
  byte buf[256];
  byte data[2] = {0x00, 0x05};
  sendMessage(0x93, data, 2);
  delay(20);
  int n = readMessage(buf, 256, 0);
  return checkSuccess(buf, n);
}

boolean setRegion() 
{
  //Set Region (we're only going to deal with North America)
  byte buf[256];
  byte data[1] = {0x01};
  sendMessage(0x97, data, 1);
  delay(20);
  int n = readMessage(buf, 256, 0);
  return checkSuccess(buf, n);
}




//Get all of the tags in the environment and log them to the logfile
int QueryEnvironment(byte* result, unsigned int timeout) 
{
  memset(result, 0x00, 12);//Clear the previous contents from the buffer
  byte buf[256];
  //Send "Read Tag ID Multiple" Command (opCode 22)
  byte timeoutHi = (timeout & 0xFFFF) >> 8;
  byte timeoutLo = timeout & 0x00FF;
  byte readmultipledata[4] = {0x00, 0x00, timeoutHi, timeoutLo};
  sendMessage(0x22, readmultipledata, 4);
  delay(timeout + 20);//Sleep to give the reader enough time to execute this command and 
  //send the data back
  int n = readMessage(buf, 256, 2);
  if (n < 7) 
  {
    Serial.println("ERROR sending \"Read Tag ID Multiple\"; " + n);
    rfid.flush();
    return 0;
  }
  if ((((((buf[3] << 8) & 0xFF00) | (0x00FF & buf[4])) == 0x0400))) 
  {
    //No tags were found
    Serial.println("ERROR: No tags found!");
    return 0;
  }
  //At least one tag was seen
  sendMessage(0x29, NULL, 0);
  delay(10);
  n = readMessage(buf, 256, 0);
  if (n < 11) 
  {
    Serial.println("ERROR sending Get Tag Buffer command");
    return 0;
  }
  unsigned int ReadIndex = (buf[5] << 8) & 0xFF00 | (buf[6] & 0x00FF);
  unsigned int WriteIndex = (buf[7] << 8) & 0xFF00 | (buf[8] & 0x00FF);
  int numTags = WriteIndex - ReadIndex;
  while (numTags > 0) 
  {
    byte getbufferdata[3] = {0x00, 0x02, 0x00};
    sendMessage(0x29, getbufferdata, 3);
    delay(10);
    n = readMessage(buf, 256, 0);
    if (n < 7) 
    {
      Serial.println("ERROR: Reading buffer failed");
      break;
    }
    if (buf[1] < 4) 
    {
      Serial.println("ERROR: Reading buffer failed");
      break;
    }
    byte num = buf[8];
    for (int i = 0; i < num; i++) 
    {
      int rssi = buf[5 + 4 + i*19];
      //Print the tag id as a hex string
      dprint("TAGID: ");
      d2print("TAGID: ");
      int k = 0;
      for (int j = 4 + i*19 + 5; j < 4 + i*19 + 5 + 12; j++) 
      {
        dprint(buf[j+5],HEX);
        d2print(buf[j+5],HEX);
        result[k] = buf[j+5];
        k++;
      }
      dprintln();
      d2println();
      break;
    }
    dprintln();
    break;
  }
  //Reset and clear the Tag ID Buffer for next Read Tag ID Multiple
  sendMessage(0x2A, NULL, 0);
  delay(10);
  n = readMessage(buf, 256, 0);
  if (!checkSuccess(buf, n)) 
  {
    Serial.println("ERROR clearing buffer for next tag read");
    return 0;
  }
  return 1;
}

// Set up the device.  Return 0 if things go well, and 1 otherwise.
int RFIDSetup() 
{
  dprintln("RFID driver initialising");
  //First connect at 230400 baud first, which will work if the reader
  //has already been initialized
  dprintln("\tAttempting 9600 NOT 230400 bps...");
  //Connect(230400);
  rfid.begin(9600);
  delay(1000);
  if (!checkBootFirmwareVersion())  
  {
    dprintln("\tFailed @ 9600 bps");
    dprintln("\tCould not open serial port at baudrate 230400 bps or 9600 bps");
    rfid.end();
    return 1;
  }
  else 
  {
    dprintln("\tSuccessful @ 9600 bps\n");
  }

  //Now boot into firmware
  dprintln("Trying to boot firmware.");
  if (!bootFirmware()) 
  {
    dprintln("Failed to boot firmware");
    return 1;
  }

  dprintln("Changing antenna ports.");
  if (!ChangeAntennaPorts(1, 1)) 
  {
    dprintln("Failed to change antenna ports");
    return 1;
  }

  dprintln("Changing read power.");
  if (!ChangeTXReadPower(readPwr)) 
  {
    dprintln("Failed to change read power");
    return 1;
  }

  dprintln("Changing protocol.");
  if (!setProtocol()) 
  {
    dprintln("Failed to set protocol to GEN2");
    return 1;
  }

  dprintln("Setting region.");
  if (!setRegion()) 
  {
    dprintln("Failed to set region");
    return 1;
  }

  // Set Power Mode (we'll just use default of "full power mode").
  // Use remaining defaults

  // Start the device thread; spawns a new thread and executes
  // RFIDdriver::Main(), which contains the main loop for the driver.
  //StartThread();

  return 0;
}
void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) 
{
  Wire.beginTransmission(deviceaddress);
  Wire.send((int)(eeaddress >> 8));   // MSB
  Wire.send((int)(eeaddress & 0xFF)); // LSB
  Wire.send(data);
  Wire.endTransmission();
  delay(5);
}

byte readEEPROM(int deviceaddress, unsigned int eeaddress ) 
{
  byte rdata = 0xFF;
  Wire.beginTransmission(deviceaddress);
  Wire.send((int)(eeaddress >> 8));   // MSB
  Wire.send((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(deviceaddress,1);
  if (Wire.available()) rdata = Wire.receive();
  return rdata;
}

void initializeRTC()
{
  // program the time & enable clock
  Wire.beginTransmission(0x68);
  Wire.send(0);
  Wire.send(0x00); //seconds = 0 and start oscillator
  Wire.send(0x26); //minutes = 4
  Wire.send(0x04); //hour = 4
  Wire.send(0x06); //day of week = 2 : Monday
  Wire.send(0x18); //day = the 4th
  Wire.send(0x03); //month = 4 = April
  Wire.send(0x11); //year = 2011
  Wire.endTransmission();
  delay(100);
}












