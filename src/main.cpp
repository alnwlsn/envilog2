// Wilson Electronic Time Capsule (v9) - Alan Wilson 2020
// This uses a BME280 sensor to measure the enviroment, and then stores the data on an on-board flash chip
// A DS3232 Real time clock is used to generate a timestamp and also to run an interrupt
// AVR atmega328p with MiniCore framework is the microcontroller that runs the show.
// Everything is kept at low power/sleep when not used, to give the unit a very long battery life.

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SPIMemory.h>
#include <SparkFunBME280.h>
#include <Wire.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void) {
    MCUSR = 0;
    wdt_disable();
    return;
}

void printBin(byte temp) {
    if (temp < 0b10000000) {
        Serial.print('0');
    }
    if (temp < 0b1000000) {
        Serial.print('0');
    }
    if (temp < 0b100000) {
        Serial.print('0');
    }
    if (temp < 0b10000) {
        Serial.print('0');
    }
    if (temp < 0b1000) {
        Serial.print('0');
    }
    if (temp < 0b100) {
        Serial.print('0');
    }
    if (temp < 0b10) {
        Serial.print('0');
    }
    Serial.print(temp, BIN);
}
void printHex(byte temp) {
    if (temp < 0x10) {
        Serial.print('0');
    }
    Serial.print(temp, HEX);
}

class DS3231 {  //my own custom lib for control of the DS3231
    uint8_t RTC_ADDR = 0x68;

    uint8_t ztimeSecond;
    uint8_t ztimeMinute;
    uint8_t ztimeHour;
    uint8_t ztimeDay;
    uint8_t ztimeMonth;
    uint8_t ztimeYear;
    uint8_t ztimeWeekday;

    uint8_t za1rmb;  //alarm register bits - set first, then set the time, day, month, etc to get this value into the clock
    uint8_t za2rmb;

   public:
    DS3231() {}

    static uint8_t bcd2bin(uint8_t val) { return val - 6 * (val >> 4); }
    static uint8_t bin2bcd(uint8_t val) { return val + 6 * (val / 10); }
    uint8_t read_register(uint8_t reg) {
        Wire.beginTransmission(RTC_ADDR);
        Wire.write((uint8_t)reg);
        Wire.endTransmission();
        Wire.requestFrom(RTC_ADDR, (uint8_t)1);
        return Wire.read();
    }
    void write_register(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(RTC_ADDR);
        Wire.write((uint8_t)reg);
        Wire.write((uint8_t)val);
        Wire.endTransmission();
    }

    void setTimeSecond(uint8_t temp) { write_register(0x00, bin2bcd(temp)); }
    void setTimeMinute(uint8_t temp) { write_register(0x01, bin2bcd(temp)); }
    void setTimeHour(uint8_t temp) { write_register(0x02, bin2bcd(temp)); }
    void setTimeDay(uint8_t temp) { write_register(0x04, bin2bcd(temp)); }
    void setTimeMonth(uint8_t temp) { write_register(0x05, bin2bcd(temp)); }
    void setTimeYear(int temp) { write_register(0x06, bin2bcd(temp)); }
    void validateClock() {
        uint8_t bstat = read_register(0x0F);
        bstat &= ~0x80;
        write_register(0x0F, bstat);
    }  //Clears OSF
    bool lostPower(void) { return (read_register(0x0f) >> 7); }
    void getTime() {
        Wire.beginTransmission(RTC_ADDR);  //set to location 0 (seconds)
        Wire.write((uint8_t)0);
        Wire.endTransmission();
        Wire.requestFrom(RTC_ADDR, (uint8_t)7);
        ztimeSecond = bcd2bin(Wire.read() & 0x7F);
        ztimeMinute = bcd2bin(Wire.read());
        ztimeHour = bcd2bin(Wire.read());
        Wire.read();
        ztimeDay = bcd2bin(Wire.read());
        ztimeMonth = bcd2bin(Wire.read());
        ztimeYear = bcd2bin(Wire.read());
    }
    uint8_t csecond() { return ztimeSecond; }
    uint8_t cminute() { return ztimeMinute; }
    uint8_t chour() { return ztimeHour; }
    uint8_t cday() { return ztimeDay; }
    uint8_t cmonth() { return ztimeMonth; }
    uint8_t cyear() { return ztimeYear; }
    uint8_t cweekday() { return ztimeWeekday; }

    void setAlarm1RMB(uint8_t temp) { za1rmb = temp; }
    void setAlarm1Second(uint8_t temp) { write_register(0x07, ((1 & (za1rmb >> 0)) << 7) + bin2bcd(temp)); }  //set alarm bcd and bit 0 of alarm 1 register mask bit
    void setAlarm1Minute(uint8_t temp) { write_register(0x08, ((1 & (za1rmb >> 1)) << 7) + bin2bcd(temp)); }
    void setAlarm1Hour(uint8_t temp) { write_register(0x09, ((1 & (za1rmb >> 2)) << 7) + bin2bcd(temp)); }
    void setAlarm1Weekday(uint8_t temp) { write_register(0x0a, (0b01000000) | (((1 & (za1rmb >> 3)) << 7) + bin2bcd(temp))); }
    void setAlarm1Day(uint8_t temp) { write_register(0x0a, (~0b01000000) & (((1 & (za1rmb >> 3)) << 7) + bin2bcd(temp))); }  //if day is set, tell alarm to use it, otherwise, use weekday

    void useAlarm1onINT() {
        uint8_t bcont = read_register(0x0e);
        bcont |= 0b00000001;
        write_register(0x0e, bcont);
    }  //sets A1IE
    void dontUseAlarm1onINT() {
        uint8_t bcont = read_register(0x0e);
        bcont &= ~0b00000001;
        write_register(0x0e, bcont);
    }  //clears A1IE
    void resetAlarm1() {
        uint8_t bstat = read_register(0x0f);
        bstat &= ~0b00000001;
        write_register(0x0f, bstat);
    }  //clears alarm 1 flag
    float getTemp() {
        int8_t temp_msb, temp_lsb;
        Wire.beginTransmission(RTC_ADDR);
        Wire.write(0x11);
        Wire.endTransmission();
        Wire.requestFrom(RTC_ADDR, (uint8_t)2);
        temp_msb = Wire.read();
        temp_lsb = (Wire.read() >> 6) & 0x03;
        Wire.endTransmission();
        if (temp_msb & 0b10000000) {  //check if negative number
            temp_msb ^= 0b11111111;
            temp_msb += 0x1;
            return (-1.0 * ((float)temp_msb) + ((float)temp_lsb * 0.25));
        } else {
            return ((float)temp_msb + ((float)temp_lsb * 0.25));
        }
    }
    void printRegs() {
        Serial.print(F("DS3231 registers:\r\n"));
        Serial.print(F("ADR DAT(BIN)\r\n"));
        for (byte j = 0; j <= 0x12; j++) {
            printHex(j);
            Serial.print(' ');
            printBin(read_register(j));
            Serial.println();
        }
        Serial.print(F("\r\n"));
    }
    void printTime() {
        getTime();
        Serial.print(F("Current time: "));
        if (cyear() < 10) {
            Serial.print('0');
        }
        Serial.print(cyear());
        Serial.print('/');
        if (cmonth() < 10) {
            Serial.print('0');
        }
        Serial.print(cmonth());
        Serial.print('/');
        if (cday() < 10) {
            Serial.print('0');
        }
        Serial.print(cday());
        Serial.print(' ');
        if (chour() < 10) {
            Serial.print('0');
        }
        Serial.print(chour());
        Serial.print(':');
        if (cminute() < 10) {
            Serial.print('0');
        }
        Serial.print(cminute());
        Serial.print(':');
        if (csecond() < 10) {
            Serial.print('0');
        }
        Serial.println(csecond());
    }
};

//board IO pins
#define greenLed A0
#define redLed A1
#define yellowLed A2
#define button 3
#define switch1 7
#define switch2 6
#define switch3 5
#define switch4 4

SPIFlash flash;
DS3231 rtc;
BME280 bme;

struct frame_t {  //structure that stores each frame
    unsigned long cycle;
    byte cyear;
    byte cmonth;
    byte cday;
    byte chour;
    byte cminute;
    byte csecond;
    float tempeReading;
    float baromReading;
    float humidReading;
    float ds3132temp;
    unsigned int bvoltadc;
    float avrTemp;
};
frame_t frame;

unsigned long lastindex;   //limit for frame
unsigned long index;       //current index to store
unsigned long icycle = 0;  //current cycle (since atmega has been powered on)

void dumpindex(unsigned long t) {
    if (!flash.readAnything(t * sizeof(frame), frame)) {
        Serial.print(F("Read failed!"));
    }
    Serial.print(t);
    Serial.print('\t');
    Serial.print(frame.cycle);
    Serial.print('\t');
    Serial.print(frame.cyear);
    Serial.print('\t');
    Serial.print(frame.cmonth);
    Serial.print('\t');
    Serial.print(frame.cday);
    Serial.print('\t');
    Serial.print(frame.chour);
    Serial.print('\t');
    Serial.print(frame.cminute);
    Serial.print('\t');
    Serial.print(frame.csecond);
    Serial.print('\t');
    Serial.print(frame.ds3132temp);
    Serial.print('\t');

    Serial.print(frame.tempeReading);
    Serial.print('\t');
    Serial.print(frame.baromReading);
    Serial.print('\t');
    Serial.print(frame.humidReading);
    Serial.print('\t');
    Serial.print(frame.avrTemp);
    Serial.print('\t');
    Serial.print(frame.bvoltadc);
    Serial.print('\t');
    Serial.println();
}
void dumpFlash(unsigned long start) {
    while (Serial.available()) {
        Serial.read();
    }  //empty serial register
    Serial.println();
    Serial.println(F("FLASH MEMORY DATA DUMP FOLLOWS"));
    Serial.println(F("Index\tCycle\tYear\tMonth\tDay\tHour\tMinute\tSecond\tDS3231 Temperature (C)\tTemperature (C)\tPressure (Pa)\tHumidity (%)\tAVR Internal Temp (C)\tBattery (10-bit ADC)"));
    for (unsigned long t = start; t < lastindex; t++) {
        dumpindex(t);
        if (digitalRead(button) == 0) {
            return;
        }
        if (Serial.available()) {
            return;
        }
    }
}

void bmeInit() {
    //SETUP OF THE BME280 SENSOR
    bme.settings.commInterface = I2C_MODE;
    bme.settings.I2CAddress = 0x76;
    bme.settings.runMode = 3;  //3 for normal mode (ON), 0 for sleep mode (OFF)
    bme.settings.tStandby = 0;
    bme.settings.filter = 0;
    bme.settings.tempOverSample = 8;
    bme.settings.pressOverSample = 8;
    bme.settings.humidOverSample = 8;
    if (bme.begin() == 0x60) {
        Serial.println(F("BME280 ready"));
    } else {
        Serial.println(F("BME280 not working"));
    }
}
float GetTemp(void) {  //internal AVR temperature, just for fun
    unsigned int wADC;
    double t;

    // The internal temperature has to be used
    // with the internal reference of 1.1V.
    // Channel 8 can not be selected with
    // the analogRead function yet.

    // Set the internal reference and mux.
    ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
    ADCSRA |= _BV(ADEN);  // enable the ADC

    delay(20);  // wait for voltages to become stable.

    ADCSRA |= _BV(ADSC);  // Start the ADC

    // Detect end-of-conversion
    while (bit_is_set(ADCSRA, ADSC))
        ;

    // Reading register "ADCW" takes care of how to read ADCL and ADCH.
    wADC = ADCW;

    // The offset of 324.31 could be wrong. It is just an indication.
    t = (wADC - 324.31) / 1.22;

    // The returned temperature is in degrees Celsius.
    return (t);
}

void awaken() {
    ADCSRA = 134;
    pinMode(button, INPUT_PULLUP);
    pinMode(switch1, INPUT_PULLUP);
    pinMode(switch2, INPUT_PULLUP);
    pinMode(switch3, INPUT_PULLUP);
    pinMode(switch4, INPUT_PULLUP);
    pinMode(greenLed, OUTPUT);
    pinMode(redLed, OUTPUT);
    pinMode(yellowLed, OUTPUT);
    rtc.resetAlarm1();
    Serial.begin(115200);
    Serial.println();
    flash.powerUp();
    bmeInit();
    delay(10);
}
void readSensors() {
    rtc.printTime();
    frame.ds3132temp = rtc.getTemp();
    frame.cyear = rtc.cyear();
    frame.cmonth = rtc.cmonth();
    frame.cday = rtc.cday();
    frame.chour = rtc.chour();
    frame.cminute = rtc.cminute();
    frame.csecond = rtc.csecond();
    frame.cycle = icycle;
    frame.bvoltadc = 0;  //analogRead(batteryADC);
    Serial.print(F("Cycle: "));
    Serial.println(icycle);
    Serial.print(F("Index: "));
    Serial.println(index);
    Serial.print(F("Battery ADC: "));
    Serial.println(frame.bvoltadc);
    Serial.print(F("DS3231 Temperature (C): "));
    Serial.println(frame.ds3132temp);
    frame.tempeReading = bme.readTempC();
    frame.baromReading = bme.readFloatPressure();
    frame.humidReading = bme.readFloatHumidity();
    Serial.print(F("BME280 Temperature (C): "));
    Serial.println(frame.tempeReading);
    Serial.print(F("BME280 Pressure   (Pa): "));
    Serial.println(frame.baromReading);
    Serial.print(F("BME280 Humidity    (%): "));
    Serial.println(frame.humidReading);
    frame.avrTemp = GetTemp();
    Serial.print(F("AVR Internal Temp  (C): "));
    Serial.println(frame.avrTemp);
}
void saveData() {
    //Write to the flash chip. The retries are likely unnessacary, but I added them anyways.
    byte retry = 10;
    while (retry > 0) {
        retry--;
        if (flash.writeAnything(index * sizeof(frame), frame)) {
            Serial.println(F("Write OK"));
            if (!digitalRead(switch4)) {
                digitalWrite(yellowLed, HIGH);
                delay(50);
                digitalWrite(yellowLed, LOW);
            }
            break;
        } else {
            Serial.println(F("Write FAILED"));
            if (!digitalRead(switch4)) {
                digitalWrite(redLed, HIGH);
                delay(50);
                digitalWrite(redLed, LOW);
            }
        }
        delay(100);
    }
}
void wakeISR() {  //just wakes the chip and disables interrupts until next time
    sleep_disable();
    detachInterrupt(0);
    detachInterrupt(1);
}
void goSleep() {
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, LOW);
    digitalWrite(yellowLed, LOW);
    bme.setMode(0);
    flash.powerDown();
    pinMode(switch1, INPUT);
    pinMode(switch2, INPUT);
    pinMode(switch3, INPUT);
    pinMode(switch4, INPUT);
    pinMode(greenLed, INPUT);
    pinMode(redLed, INPUT);
    pinMode(yellowLed, INPUT);
    Serial.end();
    pinMode(0, INPUT);
    pinMode(1, INPUT);
    delay(10);
    ADCSRA = 0;  //Shutdown procedure from the Nick Gammon fourms
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    noInterrupts();                        //stop any interrups before sleeping
    attachInterrupt(0, wakeISR, FALLING);  //goto 'wake' on interrupt (using clock alarm)
    attachInterrupt(1, wakeISR, FALLING);  //goto 'wake' on interrupt (using button)
    EIFR = bit(INTF0);                     // clear flag for interrupt 0
    interrupts();                          //start interrups and
    sleep_cpu();                           //then immediately sleep
}

void bootMsg() {
    Serial.println(F("TIME CAPSULE LONG DURATION ENVIROMENTAL LOGGER - v9 (2020-12-17) - Alan J. WIlson"));
}
void msg() {  //reads EEPROM stored message
    Serial.print(F("*******\r\n"));
    char rxb;
    int eindex = 0;
    while (1) {
        rxb = EEPROM.read(eindex);
        if (rxb == 13) {
            Serial.write(10);
        }
        if (rxb == '\\') {
            break;
        }
        Serial.write(rxb);
        if (eindex >= 1023) {
            break;
        }
        eindex++;
    }
    Serial.print(F("\r\n*******\r\n"));
}

//***************************Wilson Command Console******************************
bool consoleLock = 0;
#include <string.h>
#define serbuflen 50             //total length of single line of input
#define paramMax 10              //number of parameters that will be split
char serbuf[serbuflen + 1];      //buffer for storing serial input text
unsigned short serbufIndex = 0;  //index pointer for serbuf
char* param[paramMax];           //stores pointers to parameters
short params;                    //stores number of parameters got
void consoleProcess() {          //splits out the parameters
    char* tmp;
    params = -1;
    tmp = strtok(serbuf, " ");
    if (tmp != NULL) {
        params = 0;
        param[params] = tmp;
    } else {
        return;
    }
    while (tmp != NULL) {
        tmp = strtok(NULL, " ");
        if (tmp != NULL) {
            params++;
            if (params >= paramMax) {
                params = paramMax - 1;
            }
            param[params] = tmp;
        }
    }
}
void consoleOperate() {                           //act on the parameters we got
    if (0 == strcmp_P(param[0], PSTR("time"))) {  //sets the RTC
        Serial.print(F("\r\n"));
        if (params <= 1) {
            rtc.printTime();
        } else {
            rtc.setTimeYear(strtol(param[1], NULL, 10));
            rtc.setTimeMonth(strtol(param[2], NULL, 10));
            rtc.setTimeDay(strtol(param[3], NULL, 10));
            rtc.setTimeHour(strtol(param[4], NULL, 10));
            rtc.setTimeMinute(strtol(param[5], NULL, 10));
            rtc.setTimeSecond(strtol(param[6], NULL, 10));
            rtc.resetAlarm1();
            rtc.validateClock();
            rtc.printTime();
        }
        // Serial.print(F("\r\n1st parameter (int/prn in Hex)"));
        //   Serial.println(strtol(param[1], NULL, 16), HEX);
        // Serial.print(F("2nd parameter (int/prn in Dec)"));
        //   Serial.println(strtol(param[2], NULL, 10));
        // Serial.print(F("3rd parameter (int/prn in Bin)"));
        //   Serial.println(strtol(param[3], NULL, 2), BIN);
    }
    if (0 == strcmp_P(param[0], PSTR("exit"))) {
        Serial.print(F("\r\n"));
        consoleLock = 0;
    }
    if (0 == strcmp_P(param[0], PSTR("speed"))) {
        if (params >= 1) {
            Serial.begin(strtol(param[1], NULL, 10));
        }
    }
    if (0 == strcmp_P(param[0], PSTR("dump"))) {
        unsigned long start = 0;
        if (params >= 1) {
            start = strtol(param[1], NULL, 10);
        }
        dumpFlash(start);
    }
    if (0 == strcmp_P(param[0], PSTR("cycle"))) {
        Serial.print(F("\r\n"));
        Serial.println(icycle);
    }
    if (0 == strcmp_P(param[0], PSTR("index"))) {
        Serial.print(F("\r\n"));
        Serial.println(index);
    }
    if (0 == strcmp_P(param[0], PSTR("erase"))) {
        Serial.print(F("\r\n"));
        flash.eraseChip();
        index = 0;
        icycle = 0;
    }
    if (0 == strcmp_P(param[0], PSTR("save"))) {
        Serial.print(F("\r\n"));
        readSensors();
        saveData();
        index++;
        icycle++;
        digitalWrite(redLed, HIGH);
    }
    if (0 == strcmp_P(param[0], PSTR("reset"))) {
        Serial.print(F("\r\n"));
        wdt_enable(WDTO_15MS);
        for (;;) {
        }
    }
    if (0 == strcmp_P(param[0], PSTR("new"))) {
        Serial.print(F("\r\n"));
        char rxb = 0;
        int eindex = 0;
        while (1) {
            if (Serial.available()) {
                rxb = Serial.read();
                if (rxb == 13) {
                    Serial.write(10);
                }
                EEPROM.write(eindex, rxb);
                if (rxb == '\\') {
                    break;
                }
                Serial.write(rxb);
                if (eindex >= 1023) {
                    break;
                }
                eindex++;
            }
        }
    }
    if (0 == strcmp_P(param[0], PSTR("msg"))) {
        Serial.print(F("\r\n"));
        msg();
    }
}
void consoleRun() {
    while (Serial.available()) {
        serbuf[serbufIndex] = Serial.read();
        if (serbuf[serbufIndex] == 13) {
            serbuf[serbufIndex] = 0;
            serbufIndex = 0;
            consoleProcess();  //split the parameters
            if (params >= 0) {
                consoleOperate();  //operate the console given parameters
                                   //for(int y=0; y<=params; y++){printf("%d - %s\r\n",y,param[y]);}
            }
            Serial.print(F("\r\n>"));
        } else if (serbuf[serbufIndex] == 8 || serbuf[serbufIndex] == 127) {
            if (serbufIndex != 0) {
                serbufIndex--;
                Serial.write(8);
                Serial.write(32);
                Serial.write(8);
            }
        } else {
            Serial.write(serbuf[serbufIndex]);
            serbufIndex++;
            if (serbufIndex >= serbuflen) {
                serbufIndex = serbuflen;
            }
        }
    }
}

void setup() {
    awaken();
    bootMsg();
    msg();
    Wire.begin();
    SPI.begin();

    //don't remember why this is needed, but I'm leaving it
    SPI.beginTransaction(SPISettings(100000L, MSBFIRST, SPI_MODE0));  //start spi and set spi clock speed
    delay(10);
    pinMode(10, OUTPUT);
    digitalWrite(10, LOW);
    SPI.transfer(0xAB);
    digitalWrite(10, HIGH);
    delay(10);  //power up flash chip with raw command (in case is sleeping)
    SPI.endTransaction();

    flash.begin(MB(4));  //start flash
    //flash.eraseChip();

    digitalWrite(yellowLed, HIGH);
    delay(200);
    digitalWrite(greenLed, HIGH);
    delay(200);
    digitalWrite(redLed, HIGH);
    delay(200);
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, LOW);
    digitalWrite(yellowLed, LOW);  //3 led test pattern

    //rtc.printRegs();
    rtc.printTime();
    lastindex = (flash.getCapacity() / sizeof(frame)) - 1;
    Serial.print(F("Flash capacity: "));
    Serial.print(flash.getCapacity());
    Serial.print(F(" bytes, "));
    Serial.print(lastindex + 1);
    Serial.println(F(" frames"));
    Serial.print(F("Frame size: "));
    Serial.println((int)sizeof(frame));
    Serial.print(F("Next ram index: "));
    while (1) {
        if (!flash.readAnything(index * sizeof(frame), frame)) {
            Serial.println(F("Read failed!"));
            break;
        }
        if (frame.cycle == 4294967295) {
            break;
        }
        index++;
    }
    Serial.println(index);

    Serial.println(F("Checking for Command Console on 9600"));
    Serial.flush();

    Serial.begin(9600);
    delay(200);
    Serial.print(F("\r\nPress Enter to enter Command Console\r\n"));

    unsigned long limM = millis();
    while (millis() - limM <= 5000) {
        if (Serial.available()) {
            if (Serial.read() == 13) {
                bootMsg();
                msg();
                Serial.print(F("Welcome\r\n>"));
                digitalWrite(redLed, HIGH);
                consoleLock = 1;
                while (consoleLock) {
                    consoleRun();
                }
                digitalWrite(redLed, LOW);
            }
        }
    }
    Serial.print(F("Exit\r\n"));
    delay(100);
    Serial.begin(115200);
    Serial.println();

    //ALARM 1 REGISTER MASK BITS - 0b1110 trips timer when seconds match (timer trips once/minute). 0b1100 trips timer when minutes and seconds match (once/hour)
    rtc.setAlarm1RMB(0b1100);
    rtc.setAlarm1Day(27);
    rtc.setAlarm1Hour(11);
    rtc.setAlarm1Minute(0);
    rtc.setAlarm1Second(0);
    rtc.useAlarm1onINT();
    rtc.resetAlarm1();
}

void loop() {
    awaken();
    if (!digitalRead(switch4)) {
        digitalWrite(greenLed, HIGH);
    }
    readSensors();
    saveData();
    index++;
    icycle++;
    goSleep();
}