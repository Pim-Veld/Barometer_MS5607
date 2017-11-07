// HIS = High Impedance State
// DC = Don't Care
// *#* Indicates statements with potential hang-ups: lines 34, 45, 55,69, 82, 

#define sda A4 // I2C data line (pull-up resistor = 4k7)
#define scl A5 // I2C clock line (pull-up resistor = 4k7)

// Variables for MS5607-02BA01 Micro Altimeter Module =====
byte ms5607_addr_wr=0xEC; // I2C-address MS5607 to write
byte ms5607_addr_rd=0xED; // I2C-address MS5607 to read
byte counter, counter_print_lines;
unsigned long praw, traw;
float pressure, pressure_min, pressure_max;
float pressure_height_adjust = 1.89375; // height correction for 14,3+0,85m above AHN2
float pressure_calibrate = -2.19; // adjustment to KNMI Hoogeveen
float temp, temp_min, temp_max;
unsigned long rm, rl; // Output of function uslmply

// Variables for Real Time Clock Module DS3231 ============
byte ds3231_addr_wr=0xD0; // I2C-address DS3231 to write
byte ds3231_addr_rd=0xD1; // I2C-address DS3231 to read
boolean dst = false; // set true during Daylight Saving Time period
byte yearh, monthh, dateh, dayh, hoursh, minutesh, secondsh; // hexadecimal
byte month, date, hours, minutes, seconds;
byte dlz; // date of the last sunday of the month
unsigned int year;
char* day[]={"zo","ma","di","wo","do","vr","za"};

// General subroutines for I2C communication =============
void i2c_init(){
  // send clockpulses as long as sda = LOW
  // Entry: scl DC, sda DC
  // Exit: scl HIS, sda HIGH
  pinMode(sda, INPUT); // set sda HIS = release sda
  while (digitalRead(sda)==LOW){ // toggle scl till sda = HIGH *#*
    pinMode(scl, OUTPUT); // pull scl LOW
    pinMode(scl, INPUT); // set scl HIS --> scl = HIGH
  }
  // i2c_stop(); // Isn't necessary
}
void i2c_start(){
  // Entry: scl DC, sda DC
  // Exit: scl LOW, sda LOW
  pinMode(sda, INPUT); // set sda HIS = release sda
  pinMode(scl, INPUT); // set scl HIS --> scl = HIGH
  while (digitalRead(sda)==LOW){} // wait till sda = HIGH *#*
  pinMode(sda, OUTPUT); // pull sda LOW
  pinMode(scl, OUTPUT); // pull scl LOW
}
void i2c_stop(){
  // Entry: scl LOW, sda DC.
  // Exit: scl HIS, sda HIS.
  pinMode(sda, OUTPUT); // assure that sda is LOW
  pinMode(scl, INPUT); // set scl HIS --> scl = HIGH
  pinMode(sda, INPUT); // set sda HIS --> release sda
  while (digitalRead(sda)==LOW){} // wait till sda = HIGH *#*
}
void i2c_send(byte data_out){
  // Entry: scl LOW, sda DC.
  // Return: scl LOW, sda HIS.
  byte i;
  for(i=7;i<8;i=i-1){
    if(bitRead(data_out,i)){pinMode(sda, INPUT);}
    else{pinMode(sda, OUTPUT);}
    pinMode(scl, INPUT);
    pinMode(scl, OUTPUT);
  }
  pinMode(sda, INPUT); // Release sda to enable slave to send acknowledge
  pinMode(scl, INPUT);
  while(digitalRead(sda)!=LOW){} // Wait for acknowledge from slave *#*
  pinMode(scl, OUTPUT);
  // while(digitalRead(sda)==LOW){} // Wait while sda is LOW
  // with line above active, read coefficients stalls after two
}
byte i2c_receive(byte ackn){
  // Entry: scl LOW, sda DC
  // Exit: scl LOW, sda LOW
  // If ackn == 0 don't let master send acknowledge
  byte data_in=0;
  byte i;
  pinMode(sda, INPUT); // release sda for slave
  pinMode(scl, INPUT);
  while(digitalRead(scl)==LOW){} // wait while slave holds scl LOW *#*
  if(digitalRead(sda)==HIGH){bitSet(data_in,7);}
  pinMode(scl, OUTPUT);
  for(i=6;i<7;i--){
    pinMode(scl, INPUT);
    if(digitalRead(sda)==HIGH){bitSet(data_in,i);}
    pinMode(scl, OUTPUT);
  }
  if(ackn==0){pinMode(sda, INPUT);} // No acknowledge from Master
  else{pinMode(sda, OUTPUT);} // Acknowledge from Master
  pinMode(scl, INPUT);
  pinMode(scl, OUTPUT);
  return(data_in);
}

// Special I2C subroutines for MS5607 ====================

void ms5607_reset(){
  // write 0x1E to MS5607.
  i2c_start();
  i2c_send(ms5607_addr_wr);
  i2c_send(0x1E); // send RESET command
  i2c_stop();
  delay(2); // Is absolutely necessary
}
unsigned int read_coefficient(byte coef_addr){
  byte msb, lsb;
  unsigned int msblsb;
  i2c_start();
  i2c_send(ms5607_addr_wr); // send address ms5607 write
  i2c_send(coef_addr); // send Coefficient address
  i2c_start(); // Repeated start
  i2c_send(ms5607_addr_rd); // send address ms5607 read
  msb = i2c_receive(1); // read Coefficient ms-byte
  lsb = i2c_receive(0); // read Coefficient ls-byte
  i2c_stop();
  msblsb = msb; // combine the 2 bytes into one word
  msblsb = msblsb << 8;
  msblsb = msblsb | lsb;
  return(msblsb);
}
void read_coefficients(){
unsigned int c;
  c = read_coefficient(0xA2);
  Serial.print("C1 = ");
  Serial.print(c);
  if (c==45942){Serial.print(" ok");}
  Serial.println();
  c = read_coefficient(0xA4);
  Serial.print("C2 = ");
  Serial.print(c);
  if (c==42128){Serial.print(" ok");}
  Serial.println();
  c = read_coefficient(0xA6);
  Serial.print("C3 = ");
  Serial.print(c);
  if (c==29053){Serial.print(" ok");}
  Serial.println();
  c = read_coefficient(0xA8);
  Serial.print("C4 = ");
  Serial.print(c);
  if (c==27210){Serial.print(" ok");}
  Serial.println();
  c = read_coefficient(0xAA);
  Serial.print("C5 = ");
  Serial.print(c);
  if (c==32736){Serial.print(" ok");}
  Serial.println();
  c = read_coefficient(0xAC);
  Serial.print("C6 = ");
  Serial.print(c);
  if (c==28022){Serial.print(" ok");}
  Serial.println();
  Serial.println();
}
void start_pconv(){
  i2c_start();
  i2c_send(ms5607_addr_wr); // send address ms5607 write
  i2c_send(0x48); // start pressure conversion 4096
  i2c_stop();
}
void start_tconv(){
  i2c_start();
  i2c_send(ms5607_addr_wr); // send address ms5607 write
  i2c_send(0x58); // start temperature conversion 4096
  i2c_stop();
}
unsigned long read_adc(){
  byte byte2, byte1, byte0;
  unsigned long adc_out;
  i2c_start();
  i2c_send(ms5607_addr_wr); // send address ms5607 write
  i2c_send(0X00); // send ADC read command
  i2c_start(); // repeated start
  i2c_send(ms5607_addr_rd); // send address ms5607 read
  byte2 = i2c_receive(1); // read adc byte2 and aknowledge
  byte1 = i2c_receive(1); // read adc byte1 and aknowledge
  byte0 = i2c_receive(0); // read adc byte0 without aknowledge
  i2c_stop();
  adc_out = byte2; // combine the 3 bytes into one unsigned long
  adc_out = adc_out << 8;
  adc_out = adc_out | byte1;
  adc_out = adc_out << 8;
  adc_out = adc_out | byte0;
  return adc_out;
}
void read_praw(){
  start_pconv();
  delay(9);
  praw = read_adc();
}
void read_traw(){
  start_tconv();
  delay(9);
  traw = read_adc();
}

// Special I2C subroutines for DS3231 ====================

void set_time_and_date(){
  while(digitalRead(10)==HIGH){} // Wait till button is pressed
  i2c_start();
  i2c_send(ds3231_addr_wr); // send address ds3231 write
  i2c_send(0x00); // send address register 00
  i2c_send(0x00); // R00: set seconds = 00 and reset
  i2c_send(0x18); // R01: set minutes = 18
  i2c_send(0x01); // R02: set hours = 01 (>>> in gmt+1 <<<)
  i2c_send(0x03); // R03: set day = 03 (sunday = 01)
  i2c_send(0x30); // R04: set date = 30
  i2c_send(0x12); // R05: set month = 12
  i2c_send(0x14); // R06: set year = 14
  i2c_stop;
}
void read_time_and_date(){
  i2c_start();
  i2c_send(ds3231_addr_wr); // send address ds3231 write
  i2c_send(0x00); // send address register 00
  i2c_start(); // repeated start
  i2c_send(ds3231_addr_rd); // send address ds3231 read
  secondsh = i2c_receive(1); // read register 0x00
  minutesh = i2c_receive(1); // read register 0x01
  hoursh = i2c_receive(1); // read register 0x02
  dayh = i2c_receive(1); // read register 0x03
  dateh = i2c_receive(1); // read register 0x04
  monthh = i2c_receive(1); // read register 0x05
  yearh = i2c_receive(0); // read register 0x06
  i2c_stop();
}

// Data processing subroutines for MS5607 ==================

void uslmply(unsigned long opr1, unsigned long opr2){
  // multiplies two unsigned long variables
  // rm.rl = opr1 x opr2
  // the product is stored in 2 unsigned long variables
  // rm contains the ms 32 bits of the product
  // rl contains the ls 32 bits of the product
  unsigned int a1, a0, b1, b0;
  unsigned long a1b1, a1b0, a0b1, a0b0;
  unsigned int a1b1x, a1b0x; // de "x" stands for least significant 16 bits
  unsigned int a0b1x, a0b0x; // de "x" stands for least significant 16 bits
  unsigned long r3, r2, r1, r0;
  unsigned int carry;
  
  a0 = opr1 & 0xFFFF; // Split operands in 16-bits
  a1 = opr1 >> 16;
  b0 = opr2 & 0xFFFF;
  b1 = opr2 >> 16;
  
  a0b0 = a0; // Partial products: 32-bits
  a0b0 = a0b0 * b0;
  a0b1 = a0;
  a0b1 = a0b1 * b1;
  a1b0 = a1;
  a1b0 = a1b0 * b0;
  a1b1 = a1;
  a1b1 = a1b1 * b1;
    
  a0b0x = a0b0 & 0xFFFF; // Split partial products in 16-bits
  a0b1x = a0b1 & 0xFFFF;
  a1b0x = a1b0 & 0xFFFF;
  a1b1x = a1b1 & 0xFFFF;
  
  a0b0 = a0b0 >> 16;
  a0b1 = a0b1 >> 16;
  a1b0 = a1b0 >> 16;
  a1b1 = a1b1 >> 16;
  
  // Add split up partial products: 32-bits partial quotients
  r0 = a0b0x;
  r1 = a0b0;
  r1 = r1 + a0b1x + a1b0x;
  r2 = a0b1;
  r2 = r2 + a1b0 + a1b1x;
  r3 = a1b1;
  
  // Add carry of partial quotients to more significant partial quotient.
  // r0 is <= 0xFFFF and therefore has no carry.
  carry = r1 >> 16;
  r2 = r2 + carry;
  carry = r2 >> 16;
  r3 = r3 + carry;
  
  // Strip the carried-over values from the contributing partial quotients
  // r0 did not contribute
  r1 = r1 & 0xFFFF;
  r2 = r2 & 0xFFFF;
  // r3 did not contribute
  
  // Store the ms 32 bits in rm
  rm = r3;
  rm = rm << 16;
  rm = rm | r2;
  
  // Store the ls 32 bits in rl
  rl = r1;
  rl = rl << 16;
  rl = rl | r0;
}
void apply_corrections(){
  
    // === Calculate the corrected temperature TEMP ===
    
    // Constants and variables for temperature correction:
  unsigned long c5_2p8 = 8380416; // 32736 x 256.
  byte c6_b1 = 109; // 109*256 + 118 = 28022.
  byte c6_b0 = 118;
  long dt;
  unsigned long dt_abs;
  boolean dt_ispos;
  unsigned long dt_abs_c6_b1, dt_abs_c6_b0;
  unsigned long dt_abs_c6_b0_2m8, dt_abs_c6_2m8, dt_abs_c6_2m23;
  
    // Calculate dt (absolute value and sign)
    // dt = traw - (256 x c5)
  if(traw >= c5_2p8){
    dt_ispos = true;
    dt_abs = traw - c5_2p8;
    dt = dt_abs; 
  }
  else{
    dt_ispos = false;
    dt_abs = c5_2p8 - traw;
    dt = 0 - dt_abs;
  }
    // Calculate dt x c6 / 2^8;
  dt_abs_c6_b1 = dt_abs * c6_b1; // dt_abs x msB of c6.
  dt_abs_c6_b0 = dt_abs * c6_b0; // dt_abs x lsB of c6.
  dt_abs_c6_b0_2m8 = dt_abs_c6_b0 >> 8;
  dt_abs_c6_2m8 = dt_abs_c6_b1 + dt_abs_c6_b0_2m8;
    // Calculate dt x c6 / 2^23
  dt_abs_c6_2m23 = dt_abs_c6_2m8 >> 15;
  // Calculate Temp
   if(dt_ispos){
    temp = 20.0 + dt_abs_c6_2m23/100.0;
  }
  else{
    temp = 20.0 - dt_abs_c6_2m23/100.0;
  }
  
  // === Calculate the corrected pressure ===
  
  // Constants and variables for pressure correction:
  unsigned long c1_2p23_msl = 89; // ms part of 45942 x 2^23
  unsigned long c1_2p23_lsl = 3137339392; // ls part of 45942 x 2^23
  unsigned long c2_2p23_msl = 82; // ms part of 42128 x 2^23
  unsigned long c2_2p23_lsl = 1207959552; // ls part of 42128 x 2^23
  unsigned long c3 = 29053;
  unsigned int c4 = 27210;
  unsigned long off_abs_msl, off_abs_lsl;
  boolean off_ispos;
  unsigned long sens_abs_msl, sens_abs_lsl;
  boolean sens_ispos;
  unsigned long pressure_msl, pressure_lsl;
  // byte iscarry, isborrow;
  unsigned long carry, work1, work2;
  boolean isgreater;
  boolean flag1, flag2, flag3, flag4, flag5;
  
  // Calculate off
  // off = [(c2 * 2^23) + (c4 x dt)] / 2^6
  // Only off < 0 if dt<0 AND (c2*2^23)<(c4xdt)
  
  uslmply (c4, dt_abs); // rm,rl = c4 x dt_abs
  
  // If (c4 x dt_abs) > (C2 * 2^23) then isgreater = true
  isgreater = false;
  if(rm==c2_2p23_msl)if(rl>c2_2p23_lsl) isgreater = true;
  else if(rm>c2_2p23_msl) isgreater = true;
  
  // Set flags if bit_31 of least significant long = 1
  flag1 = false;
  if((c2_2p23_lsl&0x80000000) > 0) flag1 = true;
  flag2 = false;
  if((rl&0x80000000)>0) flag2 = true;
  
  // Set flags if addition of lowest 31 bits generates a carry
  flag3 = false;
  work1 = c2_2p23_lsl & 0x7FFFFFFF;
  work1 = work1 + (rl & 0x7FFFFFFF);
  if((work1&0x80000000)>0) flag3 = true;
  
  // Set flags if subtraction of lowest 31 bits generates a borrow
  flag4 = false;
  work1 = c2_2p23_lsl & 0x7FFFFFFF;
  work1 = work1 - (rl & 0x7FFFFFFF);
  if((work1&0x80000000)>0) flag4 = true;
  flag5 = false;
  work1 = rl & 0x7FFFFFFF;
  work1 = work1 - (c2_2p23_lsl & 0x7FFFFFFF);
  if((work1&0x80000000)>0) flag5 = true;
  
  // If dT >= 0 then add both terms
  if(dt_ispos){
    off_ispos = true;
    off_abs_msl = c2_2p23_msl + rm;
    off_abs_lsl = c2_2p23_lsl + rl;
    // Add carry
    if ((flag1 && flag2)||(flag3 && (flag1 || flag2))){
      off_abs_msl = off_abs_msl + 1;
    }
  }
  
  // If dT < 0 then subtract the smaller from the larger term.
  if(dt_ispos==false){
    if(isgreater){
      off_ispos = false;
      off_abs_msl = rm - c2_2p23_msl;
      off_abs_lsl = rl - c2_2p23_lsl;
      // Subtract borrow
      if((!flag2 && flag1)||(!flag2 && flag5)||(flag1 && flag5)){
        off_abs_msl = off_abs_msl - 1;
      }
    }
    else{
      off_ispos = true;
      off_abs_msl = c2_2p23_msl - rm;
      off_abs_lsl = c2_2p23_lsl - rl;
      // Subtract borrow
      if((!flag1 && flag2)||(!flag1 && flag4)||(flag2 && flag4)){
        off_abs_msl = off_abs_msl - 1;
      }
    }
  }
  // OFF = OFF / 2^6
  carry = off_abs_msl & 0x0000003F;
  carry = carry << 26;
  off_abs_msl = off_abs_msl >> 6;
  off_abs_lsl = off_abs_lsl >> 6;
  off_abs_lsl = off_abs_lsl | carry;
  
  // Calculate sens
  // sens = [(c1 x 2^23) + (c3 x dt)] / 2^7
  // Only sens < 0 if dt<0 AND (c1*2^23)<(c3xdT)
  
  uslmply (c3, dt_abs); // rm,rl = c3 x dt_abs
  
  // If (c3 x dt_abs) > (c1 x 2^23) then isgreater = true
  isgreater = false;
  if(rm==c1_2p23_msl)if(rl>c1_2p23_lsl) isgreater = true;
  else if(rm>c1_2p23_msl) isgreater = true;
  
  // Set flags if bit_31 of least significant long = 1
  flag1 = false;
  if((c1_2p23_lsl&0x80000000) > 0) flag1 = true;
  flag2 = false;
  if((rl&0x80000000)>0) flag2 = true;
  
  // Set flags if addition of lowest 31 bits generates a carry
  flag3 = false;
  work1 = c1_2p23_lsl & 0x7FFFFFFF;
  work1 = work1 + (rl & 0x7FFFFFFF);
  if((work1&0x80000000)>0) flag3 = true;
  
  // Set flags if subtraction of lowest 31 bits generates a borrow
  flag4 = false;
  work1 = c1_2p23_lsl & 0x7FFFFFFF;
  work1 = work1 - (rl & 0x7FFFFFFF);
  if((work1&0x80000000)>0) flag4 = true;
  flag5 = false;
  work1 = rl & 0x7FFFFFFF;
  work1 = work1 - (c1_2p23_lsl & 0x7FFFFFFF);
  if((work1&0x80000000)>0) flag5 = true;
  
  // If dt >= 0 then add both terms
  if(dt_ispos){
    sens_ispos = true;
    sens_abs_msl = c1_2p23_msl + rm;
    sens_abs_lsl = c1_2p23_lsl + rl;
    // Add carry
    if ((flag1 && flag2)||(flag3 && (flag1 || flag2))){
      sens_abs_msl = sens_abs_msl + 1;
    }
  }
  
  // If dt < 0 then subtract the smaller term from the larger
  if(dt_ispos==false){
    if(isgreater){
      sens_ispos = false;
      sens_abs_msl = rm - c1_2p23_msl;
      sens_abs_lsl = rl - c1_2p23_lsl;
      // Subtract borrow
      if((!flag2 && flag1)||(!flag2 && flag5)||(flag1 && flag5)){
        sens_abs_msl = sens_abs_msl - 1;
      }
    }
    else{
      sens_ispos = true;
      sens_abs_msl = c1_2p23_msl - rm;
      sens_abs_lsl = c1_2p23_lsl - rl;
      // Subtract borrow
      if((!flag1 && flag2)||(!flag1 && flag4)||(flag2 && flag4)){
        sens_abs_msl = sens_abs_msl - 1;
      }
    }
  }
  // sens = sens / 2^7
  carry = sens_abs_msl & 0x0000007F;
  carry = carry << 25;
  sens_abs_msl = sens_abs_msl >> 7;
  sens_abs_lsl = sens_abs_lsl >> 7;
  sens_abs_lsl = sens_abs_lsl | carry;
  
  // Calculation of the corrected pressure
  // pressure = {[praw x sens / 2^21] - off} / 2^15 in 0,01 mBar
  // pressure = {[praw x (sens / 2^2) / 2^19] - off} / 2^15
  // pressure = {minuend - subtrahend} / 2^15
  // minuend = praw x (sens / 2^2) / 2^19
  // subtrahend = off

  // sens = sens / 2^2
  // After the division sens_abs_msl should be zero
  carry = sens_abs_msl & 0x00000003;
  carry = carry << 30;
  sens_abs_msl = sens_abs_msl >> 2;
  sens_abs_lsl = sens_abs_lsl >> 2;
  sens_abs_lsl = sens_abs_lsl | carry;
  
  // rm,rl = praw x sens_abs
  uslmply (praw, sens_abs_lsl);
  
  // rm,rl = rm,rl / 2^19
  carry = rm & 0x0007FFFF;
  carry = carry << 13;
  rm = rm >> 19;
  rl = rl >> 19;
  rl = rl | carry;
  
  // Now rm,rl = minuend
  // and off = subtrahend
  // Calculation of minuend - subtrahend:
  
  // Set flags if bit_31 of least significant long = 1
  flag1 = false;
  if((rl&0x80000000) > 0) flag1 = true;
  flag2 = false;
  if((off_abs_lsl&0x80000000)>0) flag2 = true;
  
  // Set flags if addition of lowest 31 bits generates a carry
  flag3 = false;
  work1 = rl & 0x7FFFFFFF;
  work1 = work1 + (off_abs_lsl & 0x7FFFFFFF);
  if((work1&0x80000000)>0) flag3 = true;
  
  // Set flags if subtraction of lowest 31 bits generates a borrow
  flag4 = false;
  work1 = rl & 0x7FFFFFFF;
  work1 = work1 - (off_abs_lsl & 0x7FFFFFFF);
  if((work1&0x80000000)>0) flag4 = true;
  flag5 = false;
  work1 = off_abs_lsl & 0x7FFFFFFF;
  work1 = work1 - (rl & 0x7FFFFFFF);
  if((work1&0x80000000)>0) flag5 = true;
  
  // Reminder: barometric pressure is always >= 0
  if(sens_ispos && off_ispos){
    pressure_msl = rm - off_abs_msl;
    pressure_lsl = rl - off_abs_lsl;
    // Subtract borrow
    if((!flag1 && flag2)||(!flag1 && flag4)||(flag2 && flag4)){
      pressure_msl = pressure_msl - 1;
    }
  }
  if(sens_ispos && !off_ispos){
    pressure_msl = rm + off_abs_msl;
    pressure_lsl = rl + off_abs_lsl;
    // Add carry
    if ((flag1 && flag2)||(flag3 && (flag1 || flag2))){
      pressure_msl = pressure_msl + 1;
    }
  }
  if(!sens_ispos && !off_ispos){
    pressure_msl = off_abs_msl - rm;
    pressure_lsl = off_abs_lsl - rl;
    // Subtract borrow
    if((!flag2 && flag1)||(!flag2 && flag5)||(flag1 && flag5)){
      pressure_msl = pressure_msl - 1;
    }
  }
  
  // Divide pressure by 2^15
  carry = pressure_msl & 0x00007FFF;
  carry = carry << 17;
  pressure_msl = pressure_msl >> 15;
  pressure_lsl = pressure_lsl >> 15;
  pressure_lsl = pressure_lsl | carry;
  // pressure_msl should be zero [120000 < 2^32 -1]
  
  pressure = pressure_lsl / 100.0;

} // End of "apply_corrections"

void filter_pressure(){ // Average over last 10 readings
  byte i;
  static float pressure_hist[10];
  for(i=9;i>0;i=i-1){
    pressure_hist[i] = pressure_hist[i-1];
  }
  pressure_hist[0] = pressure;
  for(i=1;i<10;i=i+1){
    pressure = pressure + pressure_hist[i];
  }
  pressure = pressure / 10.0;
}

void determine_extremes(){
  if(counter>9){ // normal flow of execution
    if(pressure < pressure_min) pressure_min = pressure;
    if(pressure > pressure_max) pressure_max = pressure;
    if(temp < temp_min) temp_min = temp;
    if(temp > temp_max) temp_max = temp;
  }
  else{ // at startup reset extremes
    pressure_min = 1200.00; // 1200,00 mBar
    pressure_max = 10.00; // 10,00 mBar
    temp_min = 85.00; // 85,00 °C
    temp_max = -40.00; // -40,00 °C
  }
}

// Data processing subroutines for DS3231 ==================

void convert_time_and_date(){ // convert from hex to decimal
  seconds=((secondsh&0xF0)>>4)*10;
  seconds=seconds+(secondsh&0x0F);
  minutes=((minutesh&0xF0)>>4)*10;
  minutes=minutes+(minutesh&0x0F);
  hours=((hoursh&0x30)>>4)*10;
  hours=hours+(hoursh&0x0F);
  date=((dateh&0xF0)>>4)*10;
  date=date+(dateh&0x0F);
  month=monthh&0x0F;
  if((monthh&0x10)!=0){month=month+10;}
  year=2000+((yearh&0xF0)>>4)*10;
  year=year+(yearh&0x0F);
}

void convert_to_dst(){
     if((month<3)||(month>10)) return; // no dst in these months
     if((month>3)&&(month<10)) goto dst; // these months are in the dst-period
     // Now we only have to deal with March and October
     // Calculate the date of the last sunday of the current month (dlz)
     // Can only be 25, 26, 27, 28, 29, 30, 31 March or October
     dlz=date+7-dayh; // dlz = date of the NEXT sunday (is probably not the LAST sunday)
     while(dlz<25)dlz=dlz+7; // dlz = date of the last sunday of the current month
     if ((month==3)&&(date<dlz)) return; // no dst before the last sunday in March
     if ((month==10)&&(date>dlz)) return; // no dst after the last sunday in October
     if ((month==3)&&(date==dlz)&&(hours<2)) return; // no dst before March dlz 02:00 hours
     if ((month==10)&&(date==dlz)&&(hours>1)) return; // no dst after October dlz 02:00 hours
dst: hours=hours+1;
     if(hours<24) return; // We didn't enter into the following day
     hours = 0;
     dayh=dayh+1;
     if(dayh==8) dayh=1;
     date=date+1;
     if(date<31) return; // We didn't enter into the following month
     // Now date = 31 or 32 and month= 3 or 4 or 5 or 6 or 7 or 8 or 9 or 10
     if((date==31)&&((month==3)||(month==5)||(month==7)||(month==8)||(month==10))) return;
     // Now date=31 and month= 4 or 6 or 9, or date =32
     date=1;
     month=month+1;
}

// Output subroutines =====================================

void print_header(){
  Serial.println();
  Serial.print(day[dayh-1]);
  Serial.print(" ");
  Serial.print((dateh&0xF0)>>4);
  Serial.print(dateh&0x0F);
  Serial.print("-");
  Serial.print((monthh&0x10)>>4);
  Serial.print(monthh&0x0F);
  Serial.print("-");
  Serial.println(year);
  Serial.println("Time     Pmin    Press   Pmax    Tmin  Temp  Tmax");
}
void print_pressure_and_temp(){
  counter_print_lines = counter_print_lines + 1;
  if(counter_print_lines==1){print_header();}
  Serial.print((hoursh&0x30)>>4);
  Serial.print(hoursh&0x0F);
  Serial.print(":");
  Serial.print((minutesh&0xF0)>>4);
  Serial.print(minutesh&0x0F);
  Serial.print(":");
  Serial.print((secondsh&0xF0)>>4);
  Serial.print(secondsh&0x0F);
  Serial.print(" ");
  Serial.print(pressure_min);
  Serial.print(" ");
  Serial.print(pressure);
  Serial.print(" ");
  Serial.print(pressure_max);
  Serial.print(" ");
  Serial.print(temp_min);
  Serial.print(" ");
  Serial.print(temp);
  Serial.print(" ");
  Serial.println(temp_max);
  if(counter_print_lines==10){counter_print_lines = 0;}
}

// Setup and Loop ==========================================

void setup(){
  pinMode(13,OUTPUT); 
  digitalWrite(13,LOW); // Switch LED13 off
  pinMode(10,INPUT_PULLUP); // Pushbutton
  Serial.begin(9600);
  Serial.println("Barometric Pressure MS5607");
  i2c_init();
  ms5607_reset();
//  set_time_and_date(); // Enable when necessary;
  counter = 0; // used to determine the startup phase: if counter<10
  counter_print_lines = 0; // used to control printing of the headings
  Serial.println("Setup ok!");
  read_coefficients(); // Read coefficients from ms5607 and print them
  Serial.print("Pressure height adjustment = ");
  if(pressure_height_adjust>0) Serial.print('+');
  Serial.print(pressure_height_adjust,5);
  Serial.println(" hPa (equ to +14,3+0,85 m above AHN2)");
  Serial.print("Pressure calibration = ");
  if(pressure_calibrate>0) Serial.print('+');
  Serial.print(pressure_calibrate,5);
  Serial.println(" hPa (equ to measurement at KNMI Hoogeveen)");
  Serial.println();
}
void loop(){
  if(counter<10){counter = counter + 1;}
  read_praw();
  read_traw();
  apply_corrections();
  pressure = pressure + pressure_height_adjust; // adjust to sea level
  pressure = pressure + pressure_calibrate; // adjust to KNMI Hoogeveen
  filter_pressure();
  determine_extremes();
  read_time_and_date();
  convert_time_and_date();
  convert_to_dst(); // In dst-period convert to daylight saving time
  if (counter==10){ // normal flow of execution
    print_pressure_and_temp();
    delay(600000); // delay after startup fase
  }
  else{ // during startup
    Serial.print(counter);
    delay(1000); // delay during startup fase
  }
}
