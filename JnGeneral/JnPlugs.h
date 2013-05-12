// This part of the JeeNode General, a single sketch for all your measurement needs!
//
// 2013-05-11 <kleptog@svana.org> Martijn van Oosterhout
// http://opensource.org/licenses/mit-license.php

enum { DEVICE_LDR, DEVICE_SHT11, DEVICE_ADC, DEVICE_COMPASS, DEVICE_PRESSURE };

#include <PortsSHT11.h>

// spend a little time in power down mode while the SHT11 does a measurement
static void shtDelay () {
    Sleepy::loseSomeTime(32); // must wait at least 20 ms
}

static void *sht11_create(byte port)
{
    SHT11 *sht11 = new SHT11(port);
    return sht11;
}

static byte sht11_measure(void *data, measurement_t *measurements)
{
    SHT11 &sht11 = *(SHT11*)data;
    sht11.measure(SHT11::HUMI, shtDelay);    // TODO: check for CRC errors?
    sht11.measure(SHT11::TEMP, shtDelay);
    float h, t;
    sht11.calculate(h, t);
    measurement_t humi = h + 0.5, temp = 10 * t + 0.5;

    measurements[0] = humi;
    measurements[1] = temp;

    return 0;
}

static void sht11_destroy(void *data)
{
    SHT11 *sht11 = (SHT11*)data;
    delete sht11;
}

#define PLUG_SHT11_INFO { sht11_create, sht11_measure, sht11_destroy }

static void *ldr_create(byte port)
{
    Port *ldr = new Port(port);
    return (void*)ldr;
}

static byte ldr_measure(void *data, measurement_t *measurements)
{
    Port &ldr = *(Port*)data;
    ldr.digiWrite2(1);  // enable AIO pull-up
    byte light = ~ ldr.anaRead() >> 2;
    ldr.digiWrite2(0);  // disable pull-up to reduce current draw

    measurements[0] = light;
    return 0;
}

static void ldr_destroy(void *data)
{
    Port *ldr = (Port*)data;
    delete ldr;
}

#define PLUG_LDR_INFO { ldr_create, ldr_measure, ldr_destroy }

static void *adc_create(byte port)
{
    Port *adc = new Port(port);
    return (void*)adc;
}

static byte adc_measure(void *data, measurement_t *measurements)
{
    Port &adc = *(Port*)data;
    adc.mode2(INPUT);  // enable input
    adc.anaRead();  // throw away first reading
    measurement_t value = map(adc.anaRead(), 0, 1023, 0, 3300);  // Map to millivolts

    measurements[0] = value;
    return 0;
}

static void adc_destroy(void *data)
{
    Port *adc = (Port*)data;
    delete adc;
}

#define PLUG_ADC_INFO { adc_create, adc_measure, adc_destroy }

#include <PortsBMP085.h>

static void *pressure_create(byte port)
{
    PortI2C *i2c = new PortI2C(port);
    BMP085 *pressure = new BMP085(*i2c, 3); // ultra high resolution
    if(!pressure->isPresent()) {
        delete pressure;
        delete i2c;
        return NULL;
    }
    pressure->getCalibData();
    return (void*)pressure;
}

static byte pressure_measure(void *data, measurement_t *measurements)
{
    BMP085 &pressure = *(BMP085*)data;

    pressure.startMeas(BMP085::TEMP);
    Sleepy::loseSomeTime(16);
    pressure.getResult(BMP085::TEMP);

    pressure.startMeas(BMP085::PRES);
    Sleepy::loseSomeTime(32);
    pressure.getResult(BMP085::PRES);

    int16_t temp;
    int32_t pres;
    pressure.calculate(temp, pres);

    measurements[0] = temp;
    measurements[1] = pres / 100;  // convert to hPa to fit in 64k
    return 0;
}

static void pressure_destroy(void *data)
{
    BMP085 *pressure = (BMP085*)data;
//    const PortI2C *i2c = &pressure->port;
    delete pressure;
//    delete i2c;  //FIXME: leak I2C structure
}

#define PLUG_PRESSURE_INFO { pressure_create, pressure_measure, pressure_destroy }

static void *compass_create(byte port)
{
    PortI2C *i2c = new PortI2C(port);
    DeviceI2C *compass = new DeviceI2C(*i2c, 0x1E);
    if(!compass->isPresent()) {
        delete compass;
        delete i2c;
        return NULL;
    }
    compass->send();
    compass->write(0x00);
    compass->write(0x70);  // MSB->LSB 0111.0000 -> 8x average, 15 Hz output rate, normal flow
    compass->stop();
    return (void*)compass;
}

static byte compass_measure(void *data, measurement_t *measurements)
{
    DeviceI2C &compass = *(DeviceI2C*)data;

    compass.send();
    compass.write(0x01);
    compass.write(0x20);   // MSB->LSB 0010.0000 -> Gain = 1024, rest cleared bits
    compass.stop();

    compass.send();
    compass.write(0x02);    // Mode register
    compass.write(0x00);    // Single measurement
    Sleepy::loseSomeTime(10);

    compass.receive();

    byte outputData[6];
    for (byte i=0; i<6; i++){
        outputData[i] = compass.read(i == 5);
    }
    compass.stop();

//    for (int i=0; i<6; i++){
//        printInt(outputData[i]);
//        Serial.print(",");
//    }
    int16_t x=outputData[0] << 8 | outputData[1]; //Combine MSB and LSB of X Data output register
    int16_t z=outputData[2] << 8 | outputData[3]; //Combine MSB and LSB of Z Data output register
    int16_t y=outputData[4] << 8 | outputData[5]; //Combine MSB and LSB of Y Data output register

    measurements[0] = x;
    measurements[1] = y;
    measurements[2] = z;
    return 0;
}

static void compass_destroy(void *data)
{
    DeviceI2C *compass = (DeviceI2C*)data;
//    const PortI2C *i2c = &compass->port;
    delete compass;
//    delete i2c;  //FIXME: leak I2C structure
}

#define PLUG_COMPASS_INFO { compass_create, compass_measure, compass_destroy }

static byte init_measure(Plug &plug)
{
    if(plug.data)
        return 1;
    Device *dev = getDevice(plug.device_id);
    if(!dev) {
        debug("Unknown device");
        return 0;
    }

    Serial.print("Initialising: ");
    Serial.println(dev->name);
    serialFlush();
    plug.data = dev->info.create(plug.port);
    if(!plug.data) {
        debug("measure aborted");
        return 0;
    }

    return 1;
}

static byte measure(Plug &plug, measurement_t measurements[4])
{
    if(!plug.data) {
        debug("not initialised");
        return 0;
    }

    Device *dev = getDevice(plug.device_id);
    if(!dev) {
        debug("Unknown device");
        return 0;
    }
    int res = dev->info.measure(plug.data, measurements);
    if(res) {
        debug("measure failed?");
        printInt(res);
    }
    return 1;
}

static byte end_measure(Plug &plug)
{
    if(!plug.data)
        return 1;

    Device *dev = getDevice(plug.device_id);
    Serial.print("Finalising: ");
    Serial.println(dev->name);
    serialFlush();
    dev->info.destroy(plug.data);

    plug.data = NULL;

    return 0;
}
