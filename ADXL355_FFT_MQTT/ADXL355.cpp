
#include <Arduino.h>
#include <SPI.h>
#include "ADXL355.h"
#include "arduinoFFT.h"
#include "DebugUtils.h"

//FFT
/*#define SAMPLES 128             //Must be a power of 2
#define SAMPLING_FREQUENCY 1000 //Hz, must be less than 10000 due to ADC
arduinoFFT FFT = arduinoFFT();
*/

ADXL355::ADXL355(uint8_t ss) : _ss(ss)
{
    pinMode(_ss, OUTPUT);
    digitalWrite(_ss, HIGH);
};

ADXL355::~ADXL355(){
}

//I have also used VSPI in Arduino Programme
void ADXL355::init(uint8_t samplerate_value){ 
    vspi = new SPIClass(VSPI);
    vspi -> beginTransaction(SPISettings(_spiClk, MSBFIRST, SPI_MODE0)); //For ADXL345, it was SPI_MODE3
    vspi -> begin();
    write8(RESET_REG, RESET_CMD);
    write8(DATASYNC_REG, INTERNAL_SYNC);
    write8(I2C_INT_RANGE_REG, RANGE_2G);
    write8(FILTER_REG,LPF_CORNERFREQ);    //RATE_4000   previously: FILTER_DISABLE     LPF_CORNERFREQ
    setSamplerate(samplerate_value);
}

void ADXL355::setSamplerate(uint8_t samplerate_value){
    write8(POWER_CTL_REG,STANDBY);
    write8(SAMPLERATE_REG,samplerate_value);
    write8(POWER_CTL_REG,MEASURE_MODE);
}

bool ADXL355::isDeviceRecognized()
{
    uint8_t test = read8(0x01);  //0x01 is the Device Address
    if (test != 0x1D)
    {
        DEBUG_PRINTF("Connected device does not appear to be an ADXL355: %02x\n\n", test);
    }
    DEBUG_PRINTF("ADXL355 connected with address: %02x\n\n", test);
    return (test == 0x1D);
}

//To read and write from SPI u first need to enable it by writing LOW
uint8_t ADXL355::read8(uint8_t reg)
{
    uint8_t data = 0;
    digitalWrite(_ss, LOW);//Enable SPI
    uint8_t dataToSend = (reg << 1) | READ_BYTE;
    vspi->transfer(dataToSend);
    data = vspi->transfer(0x00);
    digitalWrite(_ss, HIGH);//Disable SPI
    return data;
}

void ADXL355::write8(uint8_t reg, uint8_t val){
    digitalWrite(_ss, LOW);//Enable SPI
    uint8_t dataToSend = (reg << 1) | WRITE_BYTE;
    vspi->transfer(dataToSend);
    vspi->transfer(val);
    digitalWrite(_ss, HIGH);//Disable SPI
}


//SOURCE: https://www.cpp.edu/~zaliyazici/sp2002/ece204/ece204-3.pdf
//To get 1’s complement of a binary number, simply invert the given number.
//To get 2’s complement of binary number is 1’s complement of given number plus 1 to the least significant bit (LSB).
//For example 2’s complement of binary number 10010 is (01101) + 1 = 01110.
//There are various uses of 2’s complement of Binary numbers,
//Mainly in signed Binary number representation and various arithmetic operations for Binary numbers,e.g., additions, subtractions, 
//Signed binary numbers means that both positive and negative numbers may be represented.
//The most significant bit(MSB) represents the sign. The most significant bit position is 0 for all positive values
//The most significant bit(MSB) position is 1 for all negative values. The rest of the bits represent the absolute value
//+15 = 01111
long ADXL355::twosCompliment(unsigned long value)
{
  if (value & (1 << (20 - 1)))
    value = value - (1 << 20);
  return value;
}

/*
double xg_Imag[SAMPLES];
double yg_Imag[SAMPLES];
double zg_Imag[SAMPLES];
unsigned long tempx[SAMPLES], tempy[SAMPLES], tempz[SAMPLES];
double x[SAMPLES], y[SAMPLES], z[SAMPLES];
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03


void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
   for (uint16_t i = 0; i < bufferSize; i++)
   {
      double abscissa;
       //Print abscissa value 
      switch (scaleType)
      {
         case SCL_INDEX:
         abscissa = (i * 1.0);
      break;
      case SCL_TIME:
         abscissa = ((i * 1.0) / SAMPLING_FREQUENCY);
      break;
      case SCL_FREQUENCY:
         abscissa = ((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES);
      break;
   }
   Serial.print(abscissa, 6);  //Print either time, frequency or just the index  
   if(scaleType== SCL_FREQUENCY)
      Serial.print("Hz");
      Serial.print(" ");
      Serial.println(vData[i], 4);  //Print data upto 4 data points
   }
   Serial.println();
}  
*/






/*
#define FFT_WIN_TYP_HAMMING 0x01 
unsigned long microseconds;
unsigned int sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));  
*/


//I think this function is to collect samples from adxl355

//void ADXL355::printRaw(){ 
    //char data[9];
    //digitalWrite(_ss, LOW);//Enable SPI
    //vspi->transfer(0x11);         //Send address of LSB of x. Address is auto-increased after each reading.
    //for(int i = 0;i < 9; i++){
     //   data[i] = vspi->transfer(0x00);
   // }
    //digitalWrite(_ss, HIGH);//Disable SPI
    //for(uint16_t i=0; i<SAMPLES; i++){
            //microseconds = micros();    //Overflows after around 70 minutes!
            //tempx[i] = (data[0])<<12 | (data[1])<<4 | (data[2])>>4;
            //tempy[i] = (data[3])<<12 | (data[4])<<4 | (data[5])>>4;
           // tempz[i] = (data[6])<<12 | (data[7])<<4 | (data[8])>>4;

            // Multiply with 0.0000156*9.81 to get mm/s^2
            //x[i] = twosCompliment(tempx[i])*0.0000039;  //For +-2g
            //y[i] = twosCompliment(tempy[i])*0.0000039;
         //   z[i] = twosCompliment(tempz[i])*0.0000039;
            //xg_Imag[i] = 0;
            //yg_Imag[i] = 0;
       //     zg_Imag[i] = 0;
        
           // Serial.printf("x:%.2f  y:%.2f  z:%.2f \n", x, y, z);
   // }
      
    //Printing acceleration values
    //for(uint16_t i=0; i<(SAMPLES >> 1); i++){
    //     Serial.print(" X: "); Serial.print(x[i]); Serial.print(" Y: "); Serial.print(y[i]); Serial.print(" Z: "); Serial.println(z[i]);
    //}

      
      //PrintVector(x, SAMPLES, SCL_TIME);  //printing the original data
      //PrintVector(y, SAMPLES, SCL_TIME);  //printing the original data
      //PrintVector(z, SAMPLES, SCL_TIME);  //printing the original data
      

      
      //FFT.DCRemoval(x, SAMPLES);
      //FFT.DCRemoval(y, SAMPLES);
      //FFT.DCRemoval(z, SAMPLES);
      
      //FFT.Windowing(x, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      //PrintVector(x, SAMPLES, SCL_TIME);
      //FFT.Windowing(y, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      //FFT.Windowing(z, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);

      //FFT.DCRemoval(x, SAMPLES);
      //FFT.DCRemoval(y, SAMPLES);
      //FFT.DCRemoval(z, SAMPLES);
    
      //FFT.Compute(x, xg_Imag, SAMPLES, FFT_FORWARD);
      //FFT.Compute(y, yg_Imag, SAMPLES, FFT_FORWARD);
      //FFT.Compute(z, zg_Imag, SAMPLES, FFT_FORWARD);
      
      //FFT.ComplexToMagnitude(x, xg_Imag, SAMPLES);
      //FFT.ComplexToMagnitude(y, yg_Imag, SAMPLES);
      //FFT.ComplexToMagnitude(z, zg_Imag, SAMPLES);
      //PrintVector(z, (SAMPLES >> 1), SCL_FREQUENCY);

      //DCRemoval1(x, SAMPLES);
      //DCRemoval1(y, SAMPLES);
      //DCRemoval1(z, SAMPLES);

            
      //Windowing1(x, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      //Windowing1(y, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      //Windowing1(z, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
 
      //DCRemoval1(x, SAMPLES);
      //DCRemoval1(y, SAMPLES);
      //DCRemoval1(z, SAMPLES);
    
      //Compute1(x, xg_Imag, SAMPLES, FFT_FORWARD);
      //Compute1(y, yg_Imag, SAMPLES, FFT_FORWARD);
      //Compute1(z, zg_Imag, SAMPLES, FFT_FORWARD);
      
      //PrintVector(x, SAMPLES, SCL_TIME);
      //ComplexToMagnitude1(x, xg_Imag, SAMPLES);
      //ComplexToMagnitude1(y, yg_Imag, SAMPLES);
      //ComplexToMagnitude1(z, zg_Imag, SAMPLES);
      //PrintVector(z, (SAMPLES >> 1), SCL_FREQUENCY);
     

      //PrintVector(x, (SAMPLES >> 1), SCL_FREQUENCY);
      
      
      //FFT Results
    //for(uint16_t i=0; i < SAMPLES; i++){
      //   Serial.print(" X: "); Serial.println(x[i]); Serial.print(" Y: "); Serial.print(y[i]); Serial.print(" Z: "); Serial.println(z[i]);
   // }
      
//}




#define SAMPLES1 128             //Must be a power of 2
#define SAMPLING_FREQUENCY1 1000 //Hz, must be less than 10000 due to ADC
//arduinoFFT FFT12 = arduinoFFT();

double xg_Imag1[SAMPLES1];
double yg_Imag1[SAMPLES1];
double zg_Imag1[SAMPLES1];
unsigned long tempx1[SAMPLES1], tempy1[SAMPLES1], tempz1[SAMPLES1];
double x1[SAMPLES1], y12[SAMPLES1], z1[SAMPLES1];  //For ArduinoFFT, it is double. For FFT.h, it is float

//I think this function is to save sample to buffer
void ADXL355::getSampletoBuffer(byte *sampleBuffer, int& previousTime, int sampleIndex){
    // 1 sample (individual measurement) contains 14 bytes,
    // 3 bytes for each axis (X,Y,Z) so total = 9 bytes for x,y and z , 2 bytes for measurement time stamps and 3 bytes for sampleIndex
    byte sampleData[14];
    memset(sampleData, 0, 14);
    int currentTime = micros();
    digitalWrite(_ss, LOW);         //Enable SPI
    vspi->transfer(0x11);       //Send address of LSB of x. Address is auto-increased after each reading.
    for(int i = 0; i < 9; ++i) {
        sampleData[i] = vspi->transfer(0x00); //For 3 bytes of data from x,y and z
    }
    digitalWrite(_ss, HIGH);        // Disable SPI

    int timedelta = currentTime - previousTime;
    sampleData[9] = timedelta >> 8; //I think to shift data to particular bit they are shifting data becuz every data is of 8 bytes
    sampleData[10] = timedelta;
    sampleData[11] = sampleIndex >> 16;
    sampleData[12] = sampleIndex >> 8;
    sampleData[13] = sampleIndex;
    previousTime = currentTime;
    int newSampleMemoryAddress = 14 * sampleIndex;
    memcpy(sampleBuffer, sampleData, 14);  //saving sampleData to sampleBuffer
    //memcpy(sampleBuffer+(newSampleMemoryAddress), sampleData, 14);       //Append new measurement to existing buffer
};


//To print the buffer in which samples were saved
void ADXL355::printBuffer(byte* data, int bufsize){
    for(int i = 0; i < bufsize; i+=14) {
        unsigned long tempx = (data[i+0])<<12 | (data[i+1])<<4 | (data[i+2])>>4;
        unsigned long tempy = (data[i+3])<<12 | (data[i+4])<<4 | (data[i+5])>>4;
        unsigned long tempz = (data[i+6])<<12 | (data[i+7])<<4 | (data[i+8])>>4;
        // Multiply with 0.0000156*9.81 to get mm/s^2
        float x = twosCompliment(tempx)*0.0000156*9.81;   // 1/256000 = 0.0000039 for +-2g  &  0.0000156 for +-8g
        float y = twosCompliment(tempy)*0.0000156*9.81;
        float z = twosCompliment(tempz)*0.0000156*9.81;
        int timedelta = data[i+9] << 8 | data[i+10];
        long sampleIndex = data[i+11] << 16 | data[i+12] << 8 | data[i+13];
        Serial.printf("x:%.2f  y:%.2f  z:%.2f delta:%d \t i: %d \n", x, y, z, timedelta, sampleIndex);
    }
}

struct Sample ADXL355::getSample(int& previousTime, int sampleIndex){
    Sample sample;
    int currentTime = micros();
    digitalWrite(_ss, LOW);//Enable SPI
    vspi->transfer(0x11);         //Send address of LSB of x. Address is auto-increased after each reading.
    for(int i = 0;i < 9; i++){
        sample.data[i] = vspi->transfer(0x00);
    }
    digitalWrite(_ss, HIGH);//Disable SPI
    sample.timedelta = currentTime - previousTime;
    previousTime = currentTime;
    sample.sampleIndex = sampleIndex;
    return sample;
}

void ADXL355::printSampleArray(struct Sample * sampleArray, int arraysize){
    for(int i = 0; i < arraysize; i+=1) {
        unsigned long tempx = (sampleArray[i].data[0])<<12 | (sampleArray[i].data[1])<<4 | (sampleArray[i].data[2])>>4;
        unsigned long tempy = (sampleArray[i].data[3])<<12 | (sampleArray[i].data[4])<<4 | (sampleArray[i].data[5])>>4;
        unsigned long tempz = (sampleArray[i].data[6])<<12 | (sampleArray[i].data[7])<<4 | (sampleArray[i].data[8])>>4;
        float x = twosCompliment(tempx)*0.0000156*9.81;   
        float y = twosCompliment(tempy)*0.0000156*9.81;
        float z = twosCompliment(tempz)*0.0000156*9.81;
        Serial.printf("x:%.2f  y:%.2f  z:%.2f delta:%d \t i: %d \n", x, y, z, sampleArray[i].timedelta, sampleArray[i].sampleIndex);
    }
}
