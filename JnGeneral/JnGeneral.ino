// This is the JeeNode General, a single sketch for all your measurement needs!
//
// This idea is that you can upload this sketch, configure the JeeNode using
// the serial port and the measurements will be made and transmitted to the
// base station with sufficient information that the base station will know
// what to do with them.
//
// One sketch to rule them all...
//
// 2013-05-11 <kleptog@svana.org> Martijn van Oosterhout
// http://opensource.org/licenses/mit-license.php


#include <JeeLib.h>
#include <avr/sleep.h>
#include <util/crc16.h>
#include <avr/eeprom.h>
#include <util/atomic.h>

#include "JnGeneral.h"

#define MAX_PLUGS 8

// set the sync mode to 2 if the fuses are still the Arduino default
// mode 3 (full powerdown) can only be used with 258 CK startup fuses
#define RADIO_SYNC_MODE 2

// The scheduler makes it easy to perform various tasks at various times:

enum { TASK_ANNOUNCE=0, TASK_MEASURE, TASK_REPORT, TASK_END };
static word schedbuf[TASK_END];
Scheduler scheduler (schedbuf, TASK_END);

#include "JnGRF12.h"
#include "JnPlugs.h"

#define JG_EEPROM_ADDR ((uint8_t*) 0x80)

// Lame floating point format, only powers of ten = mant*10^exp
// 4 bits mantissa (0..15), 4 bits exponent (-8..7)
#define SCALE(mant,exp) (((exp&0xF) << 4) | (mant&0xF))

enum { UNIT_HUMIDITY, UNIT_TEMP, UNIT_VOLT, UNIT_OHM, UNIT_SCALAR, UNIT_TESLA, UNIT_PASCAL };

static Unit units[] = {
    { UNIT_HUMIDITY, "Relative humidity", "%" },
    { UNIT_TEMP,     "Temperature", "C" },
    { UNIT_VOLT,     "Voltage", "V" },
    { UNIT_OHM,      "Resistance", "ohm" },
    { UNIT_SCALAR,   "Scalar", "" },
    { UNIT_TESLA,    "Magnetic field", "T" }, // 10^-4 T = 1 gauss
    { UNIT_PASCAL,   "Pressure", "Pa" },
};

static int numUnits = sizeof(units) / sizeof(units[0]);

static Unit *getUnit(byte unit_id)
{
    for(byte i=0; i<numUnits; i++) {
        if (units[i].unit_id == unit_id)
            return &units[i];
    }
    return NULL;
}

static Device devices[] = {
    { DEVICE_LDR, 1, 0, { { UNIT_SCALAR, SCALE(1,0), 7 }  }, "LDR sensor", PLUG_LDR_INFO },
    { DEVICE_SHT11, 2, 0, { { UNIT_HUMIDITY, SCALE(1, 0), 7 }, { UNIT_TEMP, SCALE(1, -1), -12 } }, "SHT11 sensor", PLUG_SHT11_INFO },
    { DEVICE_VOLT, 1, 0, { { UNIT_VOLT, SCALE(1,-2), 9 } }, "Voltage divider", PLUG_ADC_INFO },
    { DEVICE_COMPASS, 3, 1, { { UNIT_TESLA, SCALE(1,-7), -12 }, { UNIT_TESLA, SCALE(1,-7), -12 }, { UNIT_TESLA, SCALE(1,-7), -12 } }, "Compass board", PLUG_COMPASS_INFO },
    { DEVICE_PRESSURE, 2, 1, { { UNIT_TEMP, SCALE(1, -1), -12 }, { UNIT_PASCAL, SCALE(1,2), 12 } }, "Pressure board", PLUG_PRESSURE_INFO },
};

static int numDevices = sizeof(devices) / sizeof(devices[0]);

static Device *getDevice(byte device_id)
{
    for(byte i=0; i<numDevices; i++) {
        if (devices[i].device_id == device_id)
            return &devices[i];
    }
    return NULL;
}

static Plug plugs[MAX_PLUGS];
static int8_t numPlugs;

// Other variables used in various places in the code:
#define CMDBUFFER_SIZE 32
char cmdbuffer[CMDBUFFER_SIZE];
int8_t cmdbufferpos;

// Buffer for outgoing measurement reports
#define DATABUFFER_SIZE 60
byte databuffer[DATABUFFER_SIZE];
int8_t databufferpos;

// has to be defined because we're using the watchdog for low-power waiting
ISR(WDT_vect) { Sleepy::watchdogEvent(); }

static void serialFlush () {
    #if ARDUINO >= 100
        Serial.flush();
    #endif
    delay(2); // make sure tx buf is empty before going back to sleep
}

static void debug(char *msg) {
    Serial.println(msg);
    serialFlush();
}

int parseInt(int *value)
{
    while( cmdbuffer[cmdbufferpos] == ' ' )
        cmdbufferpos++;
    int8_t sign = 1, len = 0;
    int val = 0;
    if( cmdbuffer[cmdbufferpos] == '-' ) {
        sign = -1;
        cmdbufferpos++;
    }
    while( cmdbuffer[cmdbufferpos] >= '0' && cmdbuffer[cmdbufferpos] <= '9' ) {
        len++;
        val = val * 10 + cmdbuffer[cmdbufferpos] - '0';
        cmdbufferpos++;
    }
    val = val * sign;

    if( len ) {
        *value = val;
        return 1;
    }
    return 0;
}

void processCommand()
{
    cmdbufferpos = 1;   // Is now as indication of where we are with parsing
    int values[2];
    switch(cmdbuffer[0])
    {
        case 'h':
            showHelp();
            break;

        // The RF12 commands
        case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
            if (parseInt(&values[0])) {
                rf12config.nodeId = (bandToFreq(values[0]) << 6) + (rf12config.nodeId & 0x3F);
                saveRF12Config();
                Serial.println(rf12config.msg);
            }
            break;
        case 'n': // set node ID
            if (parseInt(&values[0]))
                values[0] &= 0x1F;
            else
                values[0] = cmdbuffer[cmdbufferpos] & 0x1F;
            if (values[0]) {
                rf12config.nodeId = (rf12config.nodeId & 0xE0) + (values[0] & 0x1F);
                saveRF12Config();
                Serial.println(rf12config.msg);
            }
            break;
        case 'g': // set group
            if (parseInt(&values[0])) {
                rf12config.group = values[0];
                saveRF12Config();
                Serial.println(rf12config.msg);
            }
            break;

        case 'l': // list devices
            showDevices();
            break;

        case 'p': // print config
            showConfig();
            break;

        case 'm': // measure
            if (parseInt(&values[0]))
                doMeasure(values[0]);
            else
                doMeasure(1);
            break;

        case 'a': // add device
            if (parseInt(&values[0]) && parseInt(&values[1])) {
                byte port = values[0];
                if(port < 1 || port > 4) {
                    Serial.println("Bad port number (1..4)");
                    break;
                }
                Device *dev = getDevice(values[1]);
                if(!dev) {
                    Serial.println("Invalid device number");
                    break;
                }

                byte i=0;
                for(i=0; i<numPlugs; i++) {
                    if( plugs[i].port == port ) {
                        Serial.println("Port already used");
                        break;
                    }
                }
                if(i==numPlugs) {
                    if(numPlugs == MAX_PLUGS) {
                        Serial.println("Too many plugs defined");
                        break;
                    }
                    memset(&plugs[numPlugs], 0, sizeof(Plug));
                    plugs[numPlugs].port = port;
                    plugs[numPlugs].device_id = dev->device_id;
                    plugs[numPlugs].period = SCALE(6,2);  // 600 * tenths of seconds = 1 minute
                    for(byte k=0; k<dev->num_measurements; k++) {
                        plugs[numPlugs].measurements[k].scale = dev->measurements[k].scale;
                        plugs[numPlugs].measurements[k].width = dev->measurements[k].width;
                    }
                    numPlugs++;
                    Serial.println("Device added.");
                    init_measure(plugs[numPlugs-1]);
                }
                saveConfig();
                scheduler.timer(TASK_ANNOUNCE, 0);
            }
            break;

        case 'd': // delete device
            if (parseInt(&values[0]) && parseInt(&values[1])) {
                byte port = values[0];
                byte device_id = values[1];
                for(byte i=0; i<numPlugs; i++) {
                    if( plugs[i].port == port & plugs[i].device_id == device_id) {
                        end_measure(plugs[i]);
                        memmove(&plugs[i], &plugs[i+1], sizeof(Plug)*(numPlugs-1-i));
                        numPlugs--;
                        Serial.println("Device removed.");
                        break;
                    }
                }
                saveConfig();
                scheduler.timer(TASK_ANNOUNCE, 0);
            }
            break;

        case 'c': // clear port
            if (parseInt(&values[0])) {
                byte port = values[0];
                for(byte i=0; i<numPlugs; i++) {
                    if( plugs[i].port == port ) {
                        end_measure(plugs[i]);
                        memmove(&plugs[i], &plugs[i+1], sizeof(Plug)*(numPlugs-1-i));
                        numPlugs--;
                        i--;
                        Serial.println("Device removed.");
                    }
                }
                saveConfig();
                scheduler.timer(TASK_ANNOUNCE, 0);
            }
            break;

        default:
            Serial.println("Unknown command.");
            break;
    }
}

void handleInput(char c)
{
    if(c == '\b' || c == '\x7f') {
        if(cmdbufferpos > 0) {
            cmdbuffer[cmdbufferpos--] = 0;
            Serial.print("\b \b");
        }
        return;
    }
    if(c >= 0x20 && c < 0x7f) {
        if(cmdbufferpos < CMDBUFFER_SIZE-1) {
            cmdbuffer[cmdbufferpos++] = c;
            cmdbuffer[cmdbufferpos] = 0;
            Serial.print(cmdbuffer+cmdbufferpos-1);
        }
        return;
    }
    if(c == '\r' || c == '\n') {
        cmdbuffer[cmdbufferpos] = 0;
        Serial.println("");
        processCommand();
        cmdbufferpos = 0;
        cmdbuffer[0] = 0;
        prompt();
        return;
    }
    printInt(c);
}

static void printInt(int32_t val) {
    char buf[2];
    if (val < 0) {
        Serial.print("-");
        val = -val;
    }
    if (val >= 10)
        printInt(val / 10);
    buf[0] = '0' + val % 10;
    buf[1] = 0;
    Serial.print(buf);
}

static void printScale(byte scale) {
    printInt(scale & 0xF);
    Serial.print("e");
    int exp = scale >> 4;
    if(exp >= 8) exp -= 16;
    printInt(exp);
}


static void showHelp()
{
    Serial.print(F("Commands:\r\n"
                 "  h                   Help\r\n"
                 "  a <port> <device>   Add device to port\r\n"
                 "  d <port> <device>   Remove device from port\r\n"
                 "  c <port>            Clear all devices from port\r\n"
                 "  l                   List defined devices\r\n"
                 "  p                   Print current configuration\r\n"
                 "  m                   Do test measurements\r\n"
                 "RF12 configuration:\r\n"
                 "  b <band>            set band: 4 = 433, 8 = 868, 9 = 915\r\n"
                 "  g <group>           RF12 group id (1..255)\r\n"
                 "  n <node>            RF12 node id (A..Z or 1..26)\r\n"
                ));
}

static void showDevices()
{
    Serial.print(F("Defined devices:\r\n"));
    for(byte i=0; i<numDevices; i++) {
        printInt(i);
        Serial.print(" ");
        Serial.print(devices[i].name);
        Serial.print(", measures: ");
        for(byte j=0; j < devices[i].num_measurements; j++) {
            Unit *unit = getUnit(devices[i].measurements[j].unit);
            if(!unit)
                Serial.print("(Unknown)");
            else
                Serial.print(unit->descr);
            Serial.print(", ");
        }
        Serial.print("\r\n");
    }
}

static void showConfig()
{
    Serial.print(F("Current configuration:\r\n"));
    for(byte i=0; i<numPlugs; i++) {
        Serial.print("port ");
        printInt(plugs[i].port);
        Serial.print(" ");
        Device *dev = getDevice(plugs[i].device_id);
        if(!dev) {
            Serial.println("(Unknown)");
            continue;
        }
        Serial.print(dev->name);
        Serial.print(" measuring: ");
        for(byte j=0; j<dev->num_measurements; j++) {
            Unit *unit = getUnit(dev->measurements[j].unit);
            if(!unit)
                Serial.print("(Unknown)");
            else {
                Serial.print(unit->descr);
                Serial.print(" (");
                printScale(plugs[i].measurements[j].scale);
                Serial.print(" ");
                Serial.print(unit->unitname);
                Serial.print(") ");
            }
            Serial.print(", ");
        }

        Serial.print("\r\n");
    }
    Serial.println(rf12config.msg);
}

static int saveConfig()
{
    byte buffer[128];
    byte *ptr = buffer;

    *ptr++ = 'J';
    *ptr++ = 'G';
    ptr++;  // space for total length

    for(byte i=0; i<numPlugs; i++) {
        Device *dev = getDevice(plugs[i].device_id);
        if(!dev) {
            continue;
        }
        byte *start = ptr++;   // space for length for this plug
        *ptr++ = plugs[i].port;
        *ptr++ = plugs[i].device_id;
        *ptr++ = plugs[i].period;
        for(byte j=0; j<dev->num_measurements; j++)
            *ptr++ = plugs[i].measurements[j].scale;

        *start = ptr-start;
    }
    buffer[2] = ptr-buffer+2;  // Add two for CRC

    word crc = ~0;
    for (byte i = 0; i < ptr-buffer; ++i)
        crc = _crc16_update(crc, buffer[i]);

    *ptr++ = crc & 0xFF;
    *ptr++ = crc >> 8;

    // save to EEPROM
    for (byte i = 0; i < ptr-buffer; ++i) {
        byte b = buffer[i];
        eeprom_write_byte(JG_EEPROM_ADDR + i, b);
    }
}

static int loadConfig()
{
    if(eeprom_read_byte(JG_EEPROM_ADDR) != 'J' || eeprom_read_byte(JG_EEPROM_ADDR + 1) != 'G')
        return 0;
    byte len = eeprom_read_byte(JG_EEPROM_ADDR + 2);
    if(len>=128)
        return 0;
    byte buffer[128];
    /// check CRC
    uint16_t crc = ~0;
    for (uint8_t i = 0; i < len; ++i) {
        buffer[i] = eeprom_read_byte(JG_EEPROM_ADDR + i);
        crc = _crc16_update(crc, buffer[i]);
    }
    if (crc != 0)
        return 0;

    byte *ptr = buffer;

    byte nodes = 0;
    ptr = buffer+3;
    while(ptr < buffer+len-2) {
        byte nodelen = *ptr;
        if(nodelen < 4 || nodelen > 8) {
            Serial.println("nodelen fail");
            return 0;
        }

        memset(&plugs[nodes], 0, sizeof(plugs[nodes]));
        plugs[nodes].port = ptr[1];
        plugs[nodes].device_id = ptr[2];
        plugs[nodes].period = ptr[3];

        Device *dev = getDevice(plugs[nodes].device_id);
        if(!dev) {
            // Ignore unknown devices
            continue;
        }
        for(byte j=0; j<dev->num_measurements; j++) {
            plugs[nodes].measurements[j].scale = ptr[4+j];
            plugs[nodes].measurements[j].width = dev->measurements[j].width;
        }
        ptr += nodelen;
        nodes++;
    }
    numPlugs = nodes;

    return 1;
}

static void sendAnnouncement()
{
    // Sends the announcement message and updates the id fields so
    // measurements can be understood.  In total about 15 measurements
    // allowed.
    byte buffer[66];
    byte *ptr = buffer;

    *ptr++ = 'C';  // Config message
    byte id = 0;

    if(numPlugs == 0)
        return;

    for(byte i=0; i<numPlugs; i++) {
        Device *dev = getDevice(plugs[i].device_id);
        if(!dev) {
            continue;
        }
        plugs[i].id = id;
        // For each measurement, the unit, the scale, the period and the bitwidth
        for(byte j=0; j<dev->num_measurements; j++) {
            *ptr++ = dev->measurements[i].unit;
            *ptr++ = plugs[i].measurements[j].scale;
            *ptr++ = plugs[i].period;
            *ptr++ = plugs[i].measurements[j].width;
            id++;
        }
    }

    rf12_sleep(RF12_WAKEUP);
    rf12_sendNow(0, buffer, ptr-buffer);
    rf12_sendWait(RADIO_SYNC_MODE);
    rf12_sleep(RF12_SLEEP);

//    for (byte i = 0; i < ptr-buffer; ++i) {
//        byte b = buffer[i];
//        printInt(b);
//        Serial.print(",");
//    }
//    Serial.println("");
}

// Takes the given measurements for a plug and buffer them for transmission
static void bufferMeasurements(Plug &plug, const measurement_t (&measurements)[4])
{
    // Now add this information to the databuffer
    byte pos = databufferpos;
    Device *dev = getDevice(plug.device_id);
    databuffer[pos++] = plug.id | ((dev->num_measurements-1) << 5);  // start id + number of values
    int32_t bitbuffer = 0;
    byte bitbuffercount = 0;
    for(byte j=0; j<dev->num_measurements; j++) {
        int8_t width = plug.measurements[j].width;
        // Clamp based on width
        int32_t low, high, mask;
        byte width_val;
        if(width > 0) {
            low = 0;
            high = (1 << width)-1;
            mask = high;
            width_val = width;
        } else {
            high = (1 << (-width-1))-1;
            low = ~high;
            mask = (1<<(-width))-1;
            width_val = -width;
        }
        int32_t m = measurements[j];
        if(m < low) m = low;
        if(m > high) m = high;

        // Chop
        m &= mask;

        bitbuffer <<= width_val;
        bitbuffer |= m;
        bitbuffercount += width_val;

//        Serial.print("(");
//        printInt(bitbuffer);
//        Serial.print(")");
        while(bitbuffercount >= 8) {
            databuffer[pos++] = bitbuffer >> (bitbuffercount-8);
            bitbuffercount -= 8;
        }
        bitbuffer &= ((1 << bitbuffercount)-1);  // Not really necessary
    }
    // Push remainder
    if(bitbuffercount) {
        bitbuffer <<= (8-bitbuffercount);
        databuffer[pos++] = bitbuffer;
    }

//    Serial.print("=> ");
//    for(byte j=databufferpos; j<pos; j++) {
//        printInt(databuffer[j]);
//        Serial.print(",");
//    }

    databufferpos = pos;
}

// Takes all the measurements form a single plug and, if requested, buffer for transmission
static int measurePlug(Plug &plug, byte record)
{
    printInt(plug.port);
    Serial.print(": ");

    Device *dev = getDevice(plug.device_id);
    measurement_t measurements[4];
    serialFlush();
    if(!measure(plug, measurements)) {
        Serial.println("(measurement failed)");
        return 1;
    }

    // Print measurements
    for(byte j=0; j<dev->num_measurements; j++) {
        printInt(measurements[j]);
        Serial.print(", ");
    }
    Serial.print("\r\n");
    serialFlush();

    if(record) {
        bufferMeasurements(plug, measurements);
        if(scheduler.idle(TASK_REPORT))
            scheduler.timer(TASK_REPORT, 1);
    }
    return 0;
}

// Measures all defined plugs
static int doMeasure(int count)
{
    Serial.print("Doing measurements:\r\n");
    for(byte n=0; n<count; n++) {
        for(byte i=0; i<numPlugs; i++) {
            measurePlug(plugs[i], n == 0);
        }
        if( (n+1) < count )
            Sleepy::loseSomeTime(1000);
    }
}

static void prompt()
{
    Serial.print("jn-g> ");
    serialFlush();
}

void setup () {
    Serial.begin(57600);
    Serial.println("\r\n[JeeNode General]");
    loadRF12Config();
    loadConfig();

    for(byte i=0; i<numPlugs; i++) {
        init_measure(plugs[i]);
    }
    showHelp();
    showConfig();
    showDevices();
    prompt();

    rf12_config(0);         // Setup RF
    rf12_sleep(RF12_SLEEP); // and power down

    scheduler.timer(TASK_ANNOUNCE, 0);    // start the announcements
}

static byte readSerial = 0;

void loop () {
    // pollWaiting goes to low-power mode but also disables the serial port.
    // So we listen to the serial port for the first minute after boot.  If
    // we don't receive something in that time assume noone is there and
    // stop monitoring.
    byte event;
    if( readSerial || millis() < 60000 ) {
        while (Serial.available())
        {
            readSerial = 1;
            handleInput(Serial.read());
            serialFlush();
        }

        event = scheduler.poll();
    } else {
        event = scheduler.pollWaiting();
    }
    switch (event) {
        case TASK_ANNOUNCE:
            // Repeat every minute for the first five minutes, then once per hour
            scheduler.timer(TASK_ANNOUNCE, millis() < 300000 ? 600 : 36000);

            sendAnnouncement();
            break;

#if 0
        case MEASURE:
            // reschedule these measurements periodically
            scheduler.timer(MEASURE, MEASURE_PERIOD);

            doMeasure();

            // every so often, a report needs to be sent out
            if (++reportCount >= REPORT_EVERY) {
                reportCount = 0;
                scheduler.timer(REPORT, 0);
            }
            break;

        case REPORT:
            doReport();
            break;
#endif
    }
}
