#include <OneWire.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

/*
  omgbio PCR multi temperature probe reader 
  for Dallas Semiconductor DS18x2x family probes.
  
  License: GPLv3
*/

#define DEBUG (1)

struct device {
  int index;
  byte addr[8];
  int model_s;
  int is_ds18x2x;
};

OneWire ds(6);

struct device devices[8];
int device_count = 0;

void debug_byte(byte b) {
  if(DEBUG) {
    Serial.print(b, HEX);
  }
}

void debug_float(float f) {
  if(DEBUG) {
    Serial.print(f);
  }
}

void debug_int(int i) {
  if(DEBUG) {
    Serial.print(i);
  }
}

void debug(char* str) {
  if(DEBUG) {
    Serial.print(str); 
  }
}

void find_devices() {
  byte addr[8];
  int i, j;

  ds.reset_search();
  delay(250);
  i = 0;
  while(ds.search(addr)) {  
    
    devices[i].index = i;
    debug("Found device: ");
    
    if(OneWire::crc8(addr, 7) != addr[7]) {
      debug("Invalid checksum! Skipping.\n");
      continue; 
    }
    
    for(j=0; j < 8; j++) {
      devices[i].addr[j] = addr[j];
      debug("0x");
      debug_byte(addr[j]);
      if(j < 7) {
        debug(":"); 
      }
    }
    debug(" Type: ");

    devices[i].is_ds18x2x = 1;
    
    // the first ROM byte indicates which chip
    switch (addr[0]) {
      case 0x10:
        debug("DS18S20 or DS1820");
        devices[i].model_s = 1;
        break;
      case 0x28:
        debug("DS18B20");
        devices[i].model_s = 0;
        break;
      case 0x22:
        debug("DS1822");
        devices[i].model_s = 0;
        break;
      default:
        debug("Not a DS18x2x family device.");
        devices[i].is_ds18x2x = 0;
        return;
    } 
    
    debug("\n");
    i++;
  }
  

  
  device_count = i;
}

int heat_ctrl_pin = 12;
int cold_ctrl_pin = 10;

// PID stuff
double pid_setpoint, pid_input, pid_output;
PID myPID(&pid_input, &pid_output, &pid_setpoint, 1000, 200, 400, DIRECT);
int pid_window_size = 5000;
unsigned long pid_window_start_time;

// PID AutoTune stuff
double aTuneStep = pid_window_size, aTuneNoise = 1;
unsigned int aTuneLookBack = 10;
PID_ATune aTune(&pid_input, &pid_output);
byte ATuneModeRemember = 0;
byte modeIndex = 0;
double kp = 2, ki = 0.5, kd = 2;
bool tuning = false;

void setup(void) {
  // PCR is:
  //  Denaturation: 94-98
  //  Annealing: 50-65
  //  Extension: 72
  
  Serial.begin(9600);

  pinMode(heat_ctrl_pin, OUTPUT);
  pinMode(cold_ctrl_pin, OUTPUT);

  delay(2000); // Wait a bit so we can show the serial console

  // Temperature sensor stuff
  find_devices();

  // PID stuff
  pid_window_start_time = millis();
  pid_setpoint = 72.0;
  myPID.SetOutputLimits(0, pid_window_size);
  myPID.SetMode(AUTOMATIC); // this turns the PID on
  
}

void AutoTuneHelper(boolean start) {
  if(start) {
    ATuneModeRemember = myPID.GetMode();
    myPID.SetMode(MANUAL);
  } else {
    modeIndex = ATuneModeRemember;
    myPID.SetMode(modeIndex);
  } 
}

void toggleAutoTune() {
  if(!tuning) {
    // begin autotune
    AutoTuneHelper(true);
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    tuning = true;
    Serial.write("AutoTune started!");
  } else { // cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
    Serial.write("AutoTune canceled");
  }
}


/*
  This function based on the temperature reading example
  from the OneWire Arduino library example
*/
float read_temperature(struct device* dev) {
  int i;
  byte present = 0;
  byte data[12];
  float celsius;
  
  ds.reset();
  ds.select(dev->addr);
  ds.write(0x44, 1); // start conversion, with parasite power on at the end
  
  delay(1000); // maybe 750ms is enough, maybe not
  
  // we might do a ds.depower() here, but the reset will take care of it.
  present = ds.reset();
  
  ds.select(dev->addr);
  ds.write(0xBE);         // Read Scratchpad

  // read 9 bytes;
  for ( i = 0; i < 9; i++) {
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (dev->model_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
//  fahrenheit = celsius * 1.8 + 32.0;
/*
  Serial.print("D:");
  Serial.print(dev->index);
  Serial.print(":");
  Serial.print(celsius);
  Serial.print("\n");
*/
  return celsius;
}

int toggle(int* state, int pin) {
  if(*state > 0) {
    *state = 0;
    digitalWrite(pin, LOW);
  } else {
    *state = 1; 
    digitalWrite(pin, HIGH);
  }
}


byte in;
int heating = 0;
int cooling = 0;
unsigned long timenow = 0;
int last_print = 0;

void loop(void) {
  
  int i;
  for(i=0; i < device_count; i++) {
    pid_input = read_temperature(&(devices[i]));
  }
 

  if (Serial.available() > 0) {
    in = Serial.read();
    switch(in) {
      case 'a':
        toggleAutoTune();
        break;
      case 'h':
        toggle(&heating, heat_ctrl_pin);
        break;
      case 'c':
        toggle(&cooling, cold_ctrl_pin);
        break;
    }
  }

  timenow = millis();
 

  if(tuning) {
    byte val = (aTune.Runtime());

    if(val != 0) {
      tuning = false;
    }

    if(!tuning) { 
      // We're done, set the tuning parameters
      kp = aTune.GetKp();
      ki = aTune.GetKi();
      kd = aTune.GetKd();
      myPID.SetTunings(kp, ki, kd);
      AutoTuneHelper(false);
      Serial.print("Tuning complete!\n");
      Serial.print("  kp: "); Serial.print(kp); Serial.print("\n");
      Serial.print("  ki: "); Serial.print(ki); Serial.print("\n");
      Serial.print("  kd: "); Serial.print(kd); Serial.print("\n");
    }
  } else { // not tuning, so run normal PID control
    myPID.Compute();
  }
  
  // PID stuff
  if(timenow - pid_window_start_time > pid_window_size) { //time to shift the Relay Window
    pid_window_start_time += pid_window_size;
  }
  
  if(pid_output > timenow - pid_window_start_time) {
    digitalWrite(heat_ctrl_pin, HIGH);
  } else {
    digitalWrite(heat_ctrl_pin, LOW);
  }  
  
  if(timenow - last_print > 1000) {
    Serial.print("Temp: ");
    Serial.print(pid_input);
    Serial.print(" | pid_output: ");
    Serial.print(pid_output);
    Serial.print("\n");
    last_print = timenow;
  }
  
}
