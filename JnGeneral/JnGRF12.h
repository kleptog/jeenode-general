// This part of the JeeNode General, a single sketch for all your measurement needs!
//
// RF12 configuration setup code (largely from RF12demo.ino)
//
// 2013-05-11 <kleptog@svana.org> Martijn van Oosterhout
// http://opensource.org/licenses/mit-license.php


#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

typedef struct {
  byte nodeId;
  byte group;
  char msg[RF12_EEPROM_SIZE-4];
  word crc;
} RF12Config;

static RF12Config rf12config;

static byte bandToFreq (byte band) {
  return band == 8 ? RF12_868MHZ : band == 9 ? RF12_915MHZ : RF12_433MHZ;
}

static int loadRF12Config () {
    /// check CRC
    uint16_t crc = ~0;
    for (uint8_t i = 0; i < RF12_EEPROM_SIZE; ++i)
        crc = _crc16_update(crc, eeprom_read_byte(RF12_EEPROM_ADDR + i));
    if (crc != 0)
        return 0;

    // Successful CRC, so load config
    for (uint8_t i = 0; i < RF12_EEPROM_SIZE; ++i)
        ((byte*) &rf12config)[i] = eeprom_read_byte(RF12_EEPROM_ADDR + i);
    return 1;
}

static void addCh (char* msg, char c) {
  byte n = strlen(msg);
  msg[n] = c;
}

static void addInt (char* msg, word v) {
  if (v >= 10)
    addInt(msg, v / 10);
  addCh(msg, '0' + v % 10);
}

static void saveRF12Config () {
  // set up a nice config string to be shown on startup
  memset(rf12config.msg, 0, sizeof rf12config.msg);
  strcpy(rf12config.msg, " ");

  byte id = rf12config.nodeId & 0x1F;
  addCh(rf12config.msg, '@' + id);
  strcat(rf12config.msg, " i");
  addInt(rf12config.msg, id);
  if (rf12config.nodeId & COLLECT)
    addCh(rf12config.msg, '*');

  strcat(rf12config.msg, " g");
  addInt(rf12config.msg, rf12config.group);

  strcat(rf12config.msg, " @ ");
  static word bands[4] = { 315, 433, 868, 915 };
  word band = rf12config.nodeId >> 6;
  addInt(rf12config.msg, bands[band]);
  strcat(rf12config.msg, " MHz ");

  rf12config.crc = ~0;
  for (byte i = 0; i < sizeof rf12config - 2; ++i)
    rf12config.crc = _crc16_update(rf12config.crc, ((byte*) &rf12config)[i]);

  // save to EEPROM
  for (byte i = 0; i < sizeof rf12config; ++i) {
    byte b = ((byte*) &rf12config)[i];
    eeprom_write_byte(RF12_EEPROM_ADDR + i, b);
  }

  if (!loadRF12Config())
    Serial.println("config save failed");
}


