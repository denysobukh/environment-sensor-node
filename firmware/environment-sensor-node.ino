
#undef DEBUG
// uncomment this for deboug output to Serial port
//#define DEBUG 1

#include <SPI.h>
#include <LowPower.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

#include <RH_RF95.h>

#define DHT_PIN         5     // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)

#define POWER_DHT_PIN   6
#define POWER_BPM_PIN   7
#define POWER_VMETR_PIN 4
#define V_PIN           3
#define LED_PIN         8


#define HEADER_FROM     0x0A    
#define HEADER_TO       0x0B

// Hardware configuration

//we don't use this library directly
//DHT lib code is pasted directly to this code with <code>delay</code> replaced by <code>LowPower.powerDown</code>
//DHT dht(DHT_PIN, DHTTYPE);
Adafruit_BMP085 bmp;
RH_RF95 rf95(SS, 3, hardware_spi);

// this is Message structure, which should be expected on receiver side
struct Message {
   int32_t pressure;
   int32_t humidity;
   int32_t temperature;
   int32_t voltage;
} msg;

boolean pressure_sensor_ok = false;
boolean radio_ok = false;
uint8_t packet_id = 0;

void setup(void) {

  #ifdef DEBUG
  Serial.begin(57600);  
  Serial.flush();
  Serial.println("-> Setup");
  Serial.flush();
  #endif 

  
  pinMode(POWER_DHT_PIN, OUTPUT);
  pinMode(POWER_BPM_PIN, OUTPUT);
  pinMode(POWER_VMETR_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  analogReference(INTERNAL);
  digitalWrite(POWER_BPM_PIN, HIGH);

  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  
  #ifdef DEBUG
  Serial.print("Init BMP: ");
  Serial.flush();
  #endif
   
  if (bmp.begin(BMP085_ULTRAHIGHRES)) {
    pressure_sensor_ok = true;
    #ifdef DEBUG
    Serial.println("OK.");
    Serial.flush();
    #endif
  } else {
    #ifdef DEBUG
    Serial.println("failed.");
    Serial.flush();
    #endif
  }

  digitalWrite(POWER_BPM_PIN, LOW);
  // disabling pull up 
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);
  

  #ifdef DEBUG
  Serial.print("Init radio: "); 
  Serial.flush();
  #endif 

  //  Init radio 
  if (rf95.init()) {
    radio_ok = true;
   
    rf95.setFrequency(868.45);
    //For RFM95/96/97/98 LORA with useRFO false, 
    // valid values are from +5 to +23.
    rf95.setTxPower(5);
   
    rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
    rf95.setPreambleLength(8);
  
    rf95.setThisAddress(HEADER_FROM);
    rf95.setHeaderFrom(HEADER_FROM);
    rf95.setHeaderTo(HEADER_TO);

    #ifdef DEBUG
    Serial.println("OK."); 
    Serial.flush();
    #endif 
    //rf95.setModeTx();  
    rf95.sleep();
  } else {
    #ifdef DEBUG
    Serial.println("failed."); 
    Serial.flush();
    #endif 
  }
   
    
  LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
}

uint8_t loopCount = 14;

void loop(void){
  
  // We only do something every 7 sleep cycles (15 * 8s = 2m)
  if(loopCount == 14)
  {

    digitalWrite(POWER_VMETR_PIN, HIGH);
    
    #ifdef DEBUG
    Serial.println("-> Waking up");
    Serial.flush();
    #endif

    readBMP();
    readDHT();

    
   
    uint32_t u = readUBat(V_PIN);
    msg.voltage = u; 

    
    digitalWrite(POWER_VMETR_PIN, LOW);
            
    #ifdef DEBUG
    Serial.print("U = "); 
    Serial.print(msg.voltage); 
    Serial.println(" V"); 
    Serial.flush();
    #endif


    sendData();
    
    loopCount = 0; 

    #ifdef DEBUG
    Serial.println("-> Going sleep");
    Serial.flush(); 
    blinkLong(1);
    #endif  
  }
  
  loopCount++;
  
  // Using Low-Power library to put the MCU to Sleep
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
}


#define      P_16   0xA001     //!< polynomial
//variant for RAM
uint16_t crc16(uint8_t *buf, uint16_t num)
{
 uint8_t i;
 uint16_t crc = 0xffff;

 while(num--)
 {
  crc ^= *buf++;
  i = 8;
  do
  {
   if (crc & 1)
    crc = (crc >> 1) ^ P_16;
   else
    crc >>= 1;
  } while(--i);
 }
 return(crc);
}


void sendData() {

    #ifdef DEBUG
    Serial.println("Powering up radio");
    Serial.flush(); 
    #endif

    //Power up the radio    
                 
    rf95.setHeaderId(packet_id++);
    rf95.send((uint8_t*) &msg, sizeof(msg));
    rf95.waitPacketSent(500);
    rf95.sleep();


    #ifdef DEBUG
    Serial.println("Transfer done");
    Serial.flush();
    #endif
 
}

void readBMP() {

    if(pressure_sensor_ok) {
      #ifdef DEBUG
      Serial.print("Reading BMP: "); 
      Serial.flush();
      #endif

      // enabling pull up 
      digitalWrite(SDA, HIGH);
      digitalWrite(SCL, HIGH);
      // power up
      delayMicroseconds(25);
      digitalWrite(POWER_BPM_PIN, HIGH);      
      delay(3);
      //LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
  
      msg.pressure = bmp.readPressure();
  
      #ifdef DEBUG
      Serial.print("P = ");
      Serial.print(msg.pressure);
      Serial.println(" Pa");
      Serial.flush();
      #endif
      digitalWrite(POWER_BPM_PIN, LOW);
      
      // disabling pull up 
      digitalWrite(SDA, LOW);
      digitalWrite(SCL, LOW);
    } else {
      #ifdef DEBUG
        Serial.println("BMP is down. Skipping."); 
        Serial.flush();
      #endif
    }

}

    
/*  
   
 [ reference voltatge / adc resulution ] 
 1.1 / 1024 = 
 = 0.00107421875     
 
 H = Vout / Vin = Z2 / (Z1 + Z2)
 = 100k / 562k + 100k = 0.151057401812689
 
 = 6.6 ( 7.454545454545455 experimental ) 
 = 0.00708984375 ( 0.719730941704036 )
 voltage = ((float)analogRead(V_PIN)) * 0.00708984375;


 22pF charge time = 13 μs

*/ 
uint32_t readUBat(uint8_t pin)
{ 
  delay(1);
  uint32_t sum = 0;
  for (int i=0; i < 4; i++) {
    int in = analogRead(pin); 
    sum += in;     
    #ifdef DEBUG
        int v = (in * 719730) / 1000000;
        Serial.print("U = ");         
        Serial.println(v); 
        Serial.flush();
    #endif
  }
  sum = sum >> 2;
  uint32_t u = (sum * 719730) / 1000000;
  return u;                             
}



uint8_t _pin;
uint32_t _maxcycles = microsecondsToClockCycles(1000);
 
#ifdef __AVR
  // Use direct GPIO access on an 8-bit AVR so keep track of the port and bitmask
  // for the digital pin connected to the DHT.  Other platforms will use digitalRead.
  uint8_t _bit, _port;
#endif


class InterruptLock {
  public:
   InterruptLock() {
    noInterrupts();
   }
   ~InterruptLock() {
    interrupts();
   }

};

uint32_t expectPulse(bool level) {
  uint32_t count = 0;
  // On AVR platforms use direct GPIO port access as it's much faster and better
  // for catching pulses that are 10's of microseconds in length:
  #ifdef __AVR
    uint8_t portState = level ? _bit : 0;
    while ((*portInputRegister(_port) & _bit) == portState) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }
  // Otherwise fall back to using digitalRead (this seems to be necessary on ESP8266
  // right now, perhaps bugs in direct port access functions?).
  #else
    while (digitalRead(_pin) == level) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }
  #endif

  return count;
}

void readDHT() {

  #ifdef DEBUG
  Serial.print("Reading DHT: "); 
  Serial.flush();
  #endif
  
  digitalWrite(POWER_DHT_PIN, HIGH);
  LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
  
  uint8_t _pin = DHT_PIN;
  bool _lastresult;
  
  #ifdef __AVR
    _bit = digitalPinToBitMask(_pin);
    _port = digitalPinToPort(_pin);
  #endif
  pinMode(_pin, INPUT_PULLUP);


  uint8_t data[5];
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  digitalWrite(_pin, HIGH);
  //delay(250);
  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);

  // First set data line low for 20 milliseconds.
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);
  //LowPower.powerDown(SLEEP_30MS, ADC_OFF, BOD_OFF);

  uint32_t cycles[80];
  {
    // Turn off interrupts temporarily because the next sections are timing critical
    // and we don't want any interruptions.
    InterruptLock lock;

    // End the start signal by setting data line high for 40 microseconds.
    digitalWrite(_pin, HIGH);
    delayMicroseconds(40);

    // Now start reading the data line to get the value from the DHT sensor.
    pinMode(_pin, INPUT_PULLUP);
    delayMicroseconds(10);  // Delay a bit to let sensor pull data line low.

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == 0) {
      blinkError(4, 1, false);
      _lastresult = false;
      return _lastresult;
    }
    if (expectPulse(HIGH) == 0) {
      blinkError(4, 2, false);
      _lastresult = false;
      return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i=0; i<80; i+=2) {
      cycles[i]   = expectPulse(LOW);
      cycles[i+1] = expectPulse(HIGH);
    }
  } // Timing critical code is now complete.


// save power, pull up off
  digitalWrite(_pin, LOW);

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i=0; i<40; ++i) {
    uint32_t lowCycles  = cycles[2*i];
    uint32_t highCycles = cycles[2*i+1];
    if ((lowCycles == 0) || (highCycles == 0)) {
      blinkError(4, 3, false);
      _lastresult = false;
      return _lastresult;
    }
    data[i/8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i/8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  // Check we read 40 bits and that the checksum matches.
  if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    blinkError(4, 4, false);
  }


  int32_t t, h;


      t = data[2] & 0x7F;
      t *= 256;
      t += data[3];
 //     t *= 0.1;
      if (data[2] & 0x80) {
        t *= -1;
      }


      h = data[0];
      h *= 256;
      h += data[1];
//      h *= 0.1;

  msg.humidity = h;
  msg.temperature = t;
  
  digitalWrite(POWER_DHT_PIN, LOW);

  #ifdef DEBUG
  if (isnan(msg.humidity) || isnan(msg.temperature)) {
    Serial.println("Failed to read from DHT sensor!");
  } else { 
    Serial.print("T = ");
    Serial.print(msg.temperature / 10.0);
    Serial.print(" ℃, ");

    Serial.print("H = ");
    Serial.print(msg.humidity / 10.0);
    Serial.println(" %");
  }
  Serial.flush();
  #endif
 
}



void blinkLong(byte e) {
    for(byte i = 0; i<e; i++) {
      digitalWrite(LED_PIN, 1);
      LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
      digitalWrite(LED_PIN, 0);
      LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
    }
    LowPower.powerDown(SLEEP_500MS, ADC_OFF, BOD_OFF);
}


void blinkError(byte e, byte f, bool halt) {

  digitalWrite(POWER_DHT_PIN, 0);
  digitalWrite(POWER_BPM_PIN, 0);
  digitalWrite(POWER_VMETR_PIN, 0);
  
  do {
    for(byte i = 0; i<e; i++) {
      digitalWrite(LED_PIN, 1);
      LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
      digitalWrite(LED_PIN, 0);
      LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    }
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    for(byte i = 0; i<f; i++) {
      digitalWrite(LED_PIN, 1);
      LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
      digitalWrite(LED_PIN, 0);
      LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
    }
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
  } while (halt);
}
