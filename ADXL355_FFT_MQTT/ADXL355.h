//#pragma once
#include <SPI.h>
#include <Arduino.h>
//#include "arduinoFFT.h"

//FFT
/*#define SAMPLES 128             //Must be a power of 2
#define SAMPLING_FREQUENCY 1000 //Hz, must be less than 10000 due to ADC
arduinoFFT FFT = arduinoFFT();
*/


#define READ_BYTE     (0x01)
#define WRITE_BYTE    (0x00)

// https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_355.pdf
#define RESET_REG           (0x2F)
#define RESET_CMD           (0x52)
#define DATASYNC_REG        (0x2B)
#define INTERNAL_SYNC       (0x00)
#define I2C_INT_RANGE_REG   (0x2C)   //For setting range 2g,4g etc..
#define RANGE_8G            (0xC3)   //For
#define FILTER_REG          (0x28)
#define FILTER_DISABLE      (0x00)
#define LPF_CORNERFREQ      (0x02)   //At 1000 Hz ODR, we can set LPF to 250 Hz    0b0010
const int RANGE_2G = 0x01;
const int RANGE_4G = 0x02;
//const int RANGE_8G = 0x03;

#define POWER_CTL_REG       (0x2D)
#define STANDBY             (0x07)
#define MEASURE_MODE        (0x00)

#define SAMPLERATE_REG      (0x28)

#define REG_TEMP2      (0x06)
#define REG_TEMP1      (0x07)

#define RATE_62_5     (0x06)
#define RATE_125      (0x05)
#define RATE_250      (0x04)
#define RATE_500      (0x03)
#define RATE_1000     (0x02)
#define RATE_2000     (0x01)
#define RATE_4000     (0x00)

#define  XDATA3      (0x08)
#define  XDATA2      (0x09)
#define  XDATA1      (0x0A)
#define  YDATA3      (0x0B)
#define  YDATA2      (0x0C)
#define  YDATA1      (0x0D)
#define  ZDATA3      (0x0E)
#define  ZDATA2      (0x0F)
#define  ZDATA1      (0x10)

#define NELEMS sizeof a / sizeof a[0]

#define MHZ 1000000

struct Sample {
    char data [9];
    int timedelta;
    int sampleIndex;
};

class ADXL355
{   
    
    private:
        static const int _spiClk = 10*MHZ; //SPI Clock Speed set to 10 MHz
    public:
        uint8_t _ss;
        SPIClass * vspi = NULL;
        int axisRegs[9] = {XDATA1, XDATA2, XDATA3, YDATA1, YDATA2, YDATA3, ZDATA1, ZDATA2, ZDATA3}; 
        ADXL355(uint8_t ss);
        ~ADXL355();
        void init(uint8_t samplerate_value);
        bool isDeviceRecognized();
        void setSamplerate(uint8_t samplerate_value);
        // Reads data and prints it to the serial
        void printRaw();
        // Reads one sample from the sensor
        void getSampletoBuffer(byte *sampleBuffer, int& previousTime, int sampleIndex);
        void printBuffer(byte* data, int bufsize);
        // Reads one sample from the sensor and saves it to Sample struct. Not used in this application
        struct Sample getSample(int& previousTime, int sampleIndex);
        // Prints sample struct array
        void printSampleArray(struct Sample * sampleArray, int arraysize);
        long twosCompliment(unsigned long value);
        uint8_t read8(uint8_t reg);
        void write8(uint8_t reg, uint8_t value);
    private:
};        
