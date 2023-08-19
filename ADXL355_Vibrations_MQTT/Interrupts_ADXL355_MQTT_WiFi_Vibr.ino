#include "ADXL355.h"
#include "pindef.h"
#include "DebugUtils.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

String dataString = "";
int param;

//WiFi
const char *ssid = "ZONG4G-30FC";     //Pixel XL    Pixel   IoT           Advice     ZONG4G-30FC   SSMA1234
const char *password = "30903090";    //Hotcoffee77 hello123 NED#iot8520   Advice123  30903090 bismillah786

const char *mqtt_server = "advice.innovaprime.io"; // 192.168.8.100  192.168.0.2  //broker.emqx.io    192.168.8.100    advice.innovaprime.io //IP Address where mosquitto broker is installed
const char *topic = "motor/vibrations";
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

//Getting epoch Date & Time From NTP Server
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
// Variable to save current epoch time
unsigned long epochTime;  //unsigned long epochTime;
// Function that gets current epoch time
unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}

#define ODR_Freq 2000  // ODR Frequency  30   10    5
#define data_collection_time_sec   60  // 60 seconds required for 1/60hz gap between FFT Frequencies  prev: 60   1490 RPM

#define FFT_N       (4096)
const int samples_to_read1 = ODR_Freq *  data_collection_time_sec;
const int samples_to_read = 120000;

bool drdy = false;
bool init_intr = true;
unsigned long tempx; unsigned long tempy; unsigned long tempz;
float x;  float y; float z;

int counter=0;
float *magx;
float *freqx;
float *array_int_x; float *array_int_y; float *array_int_z;
float *fft_inputx;
float *fft_outputx;
char * out;

unsigned long timeBegin;
unsigned long timeEnd;
unsigned long duration;

const unsigned long eventInterval = 300000;  //5 mins: 300000  120000
unsigned long previousTime = 0;
unsigned long currentTime;

const unsigned long fourMinutes = 6 * 40 * 1000UL;  //240 secs
const unsigned long fiveMinutes = 5 * 60 * 1000UL;
const unsigned long elevenMinutes =  6 * 110 * 1000UL;  //11 mins delay
const unsigned long nineMinutes =  6 * 90 * 1000UL;  //9 mins delay 540 secs
const unsigned long oneMinute =  6 * 10 * 1000UL;
  
static unsigned long lastSampleTime = 0 - fourMinutes;  // initialize such that a reading is due the first time through loop()

String Vibr_Sensor = "ADXL355Z";
String Vibr_Id = "EVAL359685WE45";  //WROVER-D0WDQ6

ADXL355 adxl355 = ADXL355(ADXL355_CS);

#define slope (float)-9.05
#define bias (float)1852.0   //1852.0
//float bias=1852.0, slope=-9.05;
float res1 = 0.0f; //prev: float res1=0.0f
float temp1;
float incr = 0.23;

uint32_t  high = 0.0;
uint32_t  low = 0.0;
uint32_t  res;
uint32_t  res2;

int meter_counter = 0;

float temperatureRaw() {
  high = adxl355.read8(REG_TEMP2);
  low = adxl355.read8(REG_TEMP1);
  res = ((high & 0x00001111) << 8); //| low;
  res1 = (((float)res - bias) / slope) - 171.0; // res1=((((float)res – 1852.0)) / (-9.05)) + 25.0; -174.3   res1=(((float)res-bias)/slope)+25.0;
  /*res1=res1+incr;
    incr++;
    if(incr==0.35)
     incr=0.23;*/
  //res1=((((float)res – 1852.0)) / (-9.05)) + 25.0;
  return res1;
  // return res;
}

//setup is run only once
void setup() {

       Serial.begin(115200);
       adxl355.init(RATE_2000);        
       adxl355.isDeviceRecognized();
       setup_wifi();
       client.setServer(mqtt_server, 1883);
       client.setCallback(callback);
       timeClient.begin();
       timeClient.setTimeOffset(18000); //change it to 3600 for GMT+1 required for Grafana

       array_int_x = (float *) ps_malloc(samples_to_read * sizeof(float)); //Create an float array of n_elements 
       array_int_y = (float *) ps_malloc(samples_to_read * sizeof(float)); //Create an float array of n_elements 
       array_int_z = (float *) ps_malloc(samples_to_read * sizeof(float)); //Create an float array of n_elements     
       
       timeBegin = millis();
       attachInterrupt(digitalPinToInterrupt(DRDY), readings_ISR, RISING); 
}

void setup_wifi() {
      delay(10);
      // We start by connecting to a WiFi network
      Serial.println();
      Serial.print("Connecting to ");
      Serial.println(ssid);
    
      WiFi.begin(ssid, password);
    
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
    
      Serial.println("");
      Serial.println("Connected to the WiFi Network");
      Serial.println("IP address: ");  //Assigns IP address to ESP32. 10.57.138.13  Try to assign static IP address to ESP32 as we did in company
      Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connecti
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      //Subscribe
      //client.subscribe("esp32/output");  //ESP32 subscribing to esp32/output so that it can receive o/p from NodeRED Client
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}

float vibr_time;
int meter_setup = 1;
void loop() {
          
          if (!client.connected())
          {
            reconnect(); 
          }
          client.loop();
          
          //[D][WiFiClient.cpp:514] connected(): Disconnected: RES: 0, ERR: 128
          //Attempting MQTT connection...connected
          if(counter > 120000 - 1){
             StaticJsonDocument<256> doc; //a lill more than 256 bytes in the stack
             long int t1 = millis();
             for(int i=0; i < 120000; i++){  //our processing  sending to cloud  //2 timestamps:  data sending which will live read the data, data reading vibrations
                epochTime = getTime();//it comes in seconds 
                //doc["counter"] = i;
                temp1 = temperatureRaw();
                doc["sensor"] = Vibr_Sensor;
                doc["device_id"] = Vibr_Id;
                doc["timestamp"] = epochTime;
                doc["temp"] = temp1;
                doc["vibr_x"] = array_int_x[i];
                doc["vibr_y"] = array_int_y[i];
                doc["vibr_z"] = array_int_z[i];
                //temp1 += 0.1;
                              
                char out[256];
                int b = serializeJson(doc, out);
                client.publish(topic, out);
                delayMicroseconds(1);
                
             }//for loop
                    
             long int t2 = millis();
             Serial.print("Time taken by the task: "); Serial.print(t2-t1); Serial.println(" milliseconds");
             
             Serial.println("--------------------------------------------------");
             free(array_int_x); free(array_int_y); free(array_int_z);  
             array_int_x = (float *) ps_malloc(samples_to_read * sizeof(float));  
             array_int_y = (float *) ps_malloc(samples_to_read * sizeof(float));   
             array_int_z = (float *) ps_malloc(samples_to_read * sizeof(float));   
             counter=0;
        }
      
        unsigned long now = millis();              //36-43 51-58   15-22   22-26
        if (now - lastSampleTime >= fourMinutes) //7 mins to collect and print data(in future our processing): 7+4mins delay = 11mins delay + 1mins delay to collect the readings 
                                                 //it'll take to collect readings
        {
           lastSampleTime += fourMinutes;
           // add code to take temperature reading here
           Serial.println("reattaching");
           timeBegin = millis();
           attachInterrupt(digitalPinToInterrupt(DRDY), readings_ISR, RISING);
        }
        //Serial.println("Hello");
      
}


//But it should check after every five mins whether data is available or not and than read the regs
void readings_ISR() {
            char data[9];       
            digitalWrite(adxl355._ss, LOW);//Enable SPI
            adxl355.vspi->transfer(0x11);        
            for (int i = 0; i < 9; i++) {
              data[i] = adxl355.vspi->transfer(0x00);
            }
            digitalWrite(adxl355._ss, HIGH);//Disable SPI
            tempx = (data[0]) << 12 | (data[1]) << 4 | (data[2]) >> 4;
            tempy = (data[3]) << 12 | (data[4]) << 4 | (data[5]) >> 4;
            tempz = (data[6]) << 12 | (data[7]) << 4 | (data[8]) >> 4;
            array_int_x[counter] = adxl355.twosCompliment(tempx) * 0.0000039;
            array_int_y[counter] = adxl355.twosCompliment(tempy) * 0.0000039;
            array_int_z[counter] = adxl355.twosCompliment(tempz) * 0.0000039; 
            
            counter=counter+1; 
            if( counter > 120000 - 1){
               //Before sending data to cloud, first check FFT's accuracy on MATLAB
               //Print how much PSRAM available
               detachInterrupt(digitalPinToInterrupt(DRDY));
            }          
}
