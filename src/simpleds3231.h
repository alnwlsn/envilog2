class DS3231{
  uint8_t RTC_ADDR=0x68;
  
  uint8_t ztimeSecond;
  uint8_t ztimeMinute;
  uint8_t ztimeHour;
  uint8_t ztimeDay;
  uint8_t ztimeMonth;
  uint8_t  ztimeYear;
  uint8_t ztimeWeekday;

  uint8_t za1rmb; //alarm register bits - set first, then set the time, day, month, etc to get this value into the clock
  uint8_t za2rmb;

  public:
  DS3231(){ }
  
  static uint8_t bcd2bin (uint8_t val) { return val - 6 * (val >> 4); }
  static uint8_t bin2bcd (uint8_t val) { return val + 6 * (val / 10); }
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

  void setTimeSecond(uint8_t temp){write_register(0x00, bin2bcd(temp));} 
  void setTimeMinute(uint8_t temp){write_register(0x01, bin2bcd(temp));} 
  void setTimeHour  (uint8_t temp){write_register(0x02, bin2bcd(temp));}
  void setTimeDay   (uint8_t temp){write_register(0x04, bin2bcd(temp));}
  void setTimeMonth (uint8_t temp){write_register(0x05, bin2bcd(temp));}
  void setTimeYear  (int  temp){write_register(0x06, bin2bcd(temp));}
  void validateClock(){ uint8_t bstat = read_register(0x0F); bstat &= ~0x80; write_register(0x0F, bstat); } //Clears OSF
  bool lostPower(void){ return (read_register(0x0f) >> 7);}
  void getTime(){
    Wire.beginTransmission(RTC_ADDR); //set to location 0 (seconds)
    Wire.write((uint8_t)0);  
    Wire.endTransmission();
    Wire.requestFrom(RTC_ADDR, 7);
    ztimeSecond = bcd2bin(Wire.read() & 0x7F);
    ztimeMinute = bcd2bin(Wire.read());
    ztimeHour = bcd2bin(Wire.read());
    Wire.read();
    ztimeDay = bcd2bin(Wire.read());
    ztimeMonth = bcd2bin(Wire.read());
    ztimeYear = bcd2bin(Wire.read());
  }
  uint8_t csecond(){return ztimeSecond;}
  uint8_t cminute(){return ztimeMinute;}
  uint8_t chour  (){return ztimeHour;}
  uint8_t cday   (){return ztimeDay;}
  uint8_t cmonth (){return ztimeMonth;}
  uint8_t cyear  (){return ztimeYear;}
  uint8_t cweekday(){return ztimeWeekday;}
  
  void setAlarm1RMB(uint8_t temp){za1rmb=temp;}
  void setAlarm1Second (uint8_t temp){write_register(0x07, ((1&(za1rmb>>0))<<7)+bin2bcd(temp)); } //set alarm bcd and bit 0 of alarm 1 register mask bit
  void setAlarm1Minute (uint8_t temp){write_register(0x08, ((1&(za1rmb>>1))<<7)+bin2bcd(temp)); }
  void setAlarm1Hour   (uint8_t temp){write_register(0x09, ((1&(za1rmb>>2))<<7)+bin2bcd(temp)); }
  void setAlarm1Weekday(uint8_t temp){write_register(0x0a,  0b01000000|((1&(za1rmb>>3))<<7)+bin2bcd(temp)); } 
  void setAlarm1Day    (uint8_t temp){write_register(0x0a, ~0b01000000&((1&(za1rmb>>3))<<7)+bin2bcd(temp)); } //if day is set, tell alarm to use it, otherwise, use weekday
  
  void useAlarm1onINT()    { uint8_t bcont = read_register(0x0e); bcont |= 0b00000001; write_register(0x0e, bcont); } //sets A1IE
  void dontUseAlarm1onINT(){ uint8_t bcont = read_register(0x0e); bcont &=~0b00000001; write_register(0x0e, bcont); } //clears A1IE
  void resetAlarm1()       { uint8_t bstat = read_register(0x0f); bstat &=~0b00000001; write_register(0x0f, bstat); } //clears alarm 1 flag
  float getTemp(){
    int8_t temp_msb, temp_lsb;
    Wire.beginTransmission(RTC_ADDR);
    Wire.write(0x11);
    Wire.endTransmission();
    Wire.requestFrom(RTC_ADDR, 2);
    temp_msb =  Wire.read();
    temp_lsb = (Wire.read() >> 6) & 0x03;
    Wire.endTransmission();
    if(temp_msb & 0b10000000) {     //check if negative number
      temp_msb  ^= 0b11111111;
      temp_msb  += 0x1;
      return (-1.0 * ((float)temp_msb) + ((float)temp_lsb * 0.25));
    }
    else {
      return ((float)temp_msb + ((float)temp_lsb * 0.25));
    }
}
 
};