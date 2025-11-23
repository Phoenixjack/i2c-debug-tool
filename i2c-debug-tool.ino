#include <Wire.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>

const uint32_t SERIAL_BAUD = 115200;
const uint8_t  I2C_SDA_PIN = SDA; // adjust if needed
const uint8_t  I2C_SCL_PIN = SCL; // adjust if needed

#define MAX_NAME_LEN 32
#define EEPROM_MAX_DEVICES 8
#define EEPROM_BASE_ADDR 0

struct I2C_DeviceInfo {
  uint8_t address;           // 7-bit I2C address
  char    name[MAX_NAME_LEN];
  uint8_t chip_id_reg;       // 0xFF = no ID register check
  uint8_t expected_chip_id;  // expected value after mask
  uint8_t id_mask;           // bits to compare (0xFF = exact)
} PROGMEM;

const I2C_DeviceInfo i2c_devices[] PROGMEM = {
  // Same address, same WHO_AM_I register, different IDs
  { 0x68, "MPU-6050",              0x75, 0x68, 0xFF },
  { 0x68, "MPU-9250 / ICM-20948",  0x75, 0x71, 0xFF },

  // BME/BMP variants (same addr + reg + ID)
  { 0x76, "BME280 / BMP280 (def)", 0xD0, 0x60, 0xFF },
  { 0x77, "BME280 / BMP280 (alt)", 0xD0, 0x60, 0xFF },

  // Devices with no chip-ID check: use chip_id_reg = 0xFF
  { 0x23, "BH1750 Light Sensor",   0xFF, 0x00, 0x00 },
  { 0x29, "TSL2561 / APDS-9960",   0xFF, 0x00, 0x00 },
  { 0x40, "HTU21D / Si7021",       0xFF, 0x00, 0x00 },
  { 0x3C, "OLED (SSD1306)",        0xFF, 0x00, 0x00 },
  { 0x3D, "OLED (SSD1306)",        0xFF, 0x00, 0x00 },
};

const size_t NUM_I2C_DEVICES = sizeof(i2c_devices) / sizeof(i2c_devices[0]);
const size_t EEPROM_ENTRY_SIZE = sizeof(I2C_DeviceInfo);

// --------- Utility: hex parsing / printing ----------

uint16_t parseHex(const String &sRaw, bool &ok) {
  String s = sRaw;
  s.trim();
  s.toUpperCase();
  if (s.startsWith("0X")) s.remove(0, 2);
  if (s.length() == 0) {
    ok = false;
    return 0;
  }
  char *endPtr = nullptr;
  uint16_t v = (uint16_t) strtoul(s.c_str(), &endPtr, 16);
  ok = (*endPtr == '\0');
  return v;
}

void printHexByte(uint8_t v) {
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

void printHex16(uint16_t v) {
  if (v < 0x10)      Serial.print("000");
  else if (v < 0x100) Serial.print("00");
  else if (v < 0x1000) Serial.print('0');
  Serial.print(v, HEX);
}

String readLineBlocking() {
  String line;
  while (true) {
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        line.trim();
        return line;
      }
      if (isPrintable(c)) {
        line += c;
        Serial.print(c);
      }
    }
    delay(1);
  }
}

bool parseHexPrompt(const __FlashStringHelper *prompt, uint8_t &valueOut, uint8_t defaultVal, bool allowBlank) {
  while (true) {
    Serial.print(prompt);
    String resp = readLineBlocking();
    Serial.println();
    if (resp.length() == 0 && allowBlank) {
      valueOut = defaultVal;
      return true;
    }

    bool ok = false;
    uint16_t v = parseHex(resp, ok);
    if (ok && v <= 0xFF) {
      valueOut = (uint8_t)v;
      return true;
    }

    Serial.println(F("  Invalid hex value. Try again."));
  }
}

// --------- I2C helpers ----------

bool i2cWriteRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  uint8_t err = Wire.endTransmission();
  return (err == 0);
}

bool i2cReadRegister(uint8_t addr, uint8_t reg, uint8_t &value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission(false); // repeated START
  if (err != 0) return false;

  uint8_t toRead = 1;
  uint8_t got = Wire.requestFrom((int)addr, (int)toRead);
  if (got != toRead) return false;

  value = Wire.read();
  return true;
}

bool i2cReadRange(uint8_t addr, uint8_t startReg, uint8_t endReg) {
  if (endReg < startReg) return false;

  uint8_t reg = startReg;
  while (reg <= endReg) {
    uint8_t chunkStart = reg;
    uint8_t remaining  = endReg - reg + 1;
    uint8_t chunkLen   = remaining > 16 ? 16 : remaining;

    // Set register pointer
    Wire.beginTransmission(addr);
    Wire.write(reg);
    uint8_t err = Wire.endTransmission(false); // repeated START
    if (err != 0) {
      Serial.print(F("I2C error setting reg "));
      printHexByte(reg);
      Serial.print(F(" err="));
      Serial.println(err);
      return false;
    }

    uint8_t got = Wire.requestFrom((int)addr, (int)chunkLen);
    if (got != chunkLen) {
      Serial.print(F("I2C short read at reg "));
      printHexByte(reg);
      Serial.print(F(" got="));
      Serial.println(got);
      return false;
    }

    // Print line header
    printHexByte(chunkStart);
    Serial.print(F(": "));

    for (uint8_t i = 0; i < chunkLen; i++) {
      uint8_t b = Wire.read();
      printHexByte(b);
      if (i != chunkLen - 1) Serial.print(' ');
      reg++;
    }
    Serial.println();
  }
  return true;
}

// --------- EEPROM-backed device storage ----------

bool eepromSlotUsed(const I2C_DeviceInfo &info) {
  return info.address <= 0x7F && info.name[0] != 0xFF && info.name[0] != '\0';
}

void loadEepromDevice(size_t slot, I2C_DeviceInfo &info) {
  int base = EEPROM_BASE_ADDR + slot * EEPROM_ENTRY_SIZE;
  EEPROM.get(base, info);
}

void saveEepromDevice(size_t slot, const I2C_DeviceInfo &info) {
  int base = EEPROM_BASE_ADDR + slot * EEPROM_ENTRY_SIZE;
  EEPROM.put(base, info);
}

int findFreeEepromSlot() {
  for (size_t i = 0; i < EEPROM_MAX_DEVICES; i++) {
    I2C_DeviceInfo info;
    loadEepromDevice(i, info);
    if (!eepromSlotUsed(info)) return (int)i;
  }
  return -1;
}

// --------- Device identification ----------

void appendName(String &dest, const String &name) {
  if (dest.length() > 0) dest += F(", ");
  dest += name;
}

void considerDeviceMatch(const I2C_DeviceInfo &info, uint8_t addr, String &confirmedNames, String &possibleNames, bool &hasDatabaseMatch) {
  if (info.address != addr) return;

  hasDatabaseMatch = true;
  String name(info.name);

  if (info.chip_id_reg == 0xFF) {
    appendName(possibleNames, name);
    return;
  }

  uint8_t chipId = 0;
  bool ok = i2cReadRegister(addr, info.chip_id_reg, chipId);
  if (ok && ((chipId & info.id_mask) == (info.expected_chip_id & info.id_mask))) {
    appendName(confirmedNames, name);
  } else {
    appendName(possibleNames, name);
  }
}

bool promptAddDeviceToEeprom(uint8_t addr) {
  Serial.print(F("  No match for 0x"));
  printHexByte(addr);
  Serial.print(F(". Add to EEPROM? [y/N]: "));
  String resp = readLineBlocking();
  resp.toLowerCase();
  if (!(resp == "y" || resp == "yes")) {
    Serial.println();
    return false;
  }

  int slot = findFreeEepromSlot();
  if (slot < 0) {
    Serial.println(F("\n  EEPROM is full. Delete an entry to add more."));
    return false;
  }

  I2C_DeviceInfo info;
  memset(&info, 0xFF, sizeof(info));
  info.address = addr;
  info.id_mask = 0xFF;

  Serial.println();
  Serial.print(F("  Enter device name (max 31 chars): "));
  String name = readLineBlocking();
  if (name.length() == 0) {
    name = F("Custom 0x");
    char buf[3];
    snprintf(buf, sizeof(buf), "%02X", addr);
    name += buf;
  }
  name.remove(MAX_NAME_LEN - 1); // ensure space for null terminator
  name.toCharArray(info.name, MAX_NAME_LEN);

  Serial.println();
  Serial.println(F("  CHIPID setup:"));
  Serial.println(F("    - Enter FF if no chip-ID check is available"));
  uint8_t chipReg = 0xFF;
  parseHexPrompt(F("    Chip ID register (hex): "), chipReg, 0xFF, false);
  info.chip_id_reg = chipReg;

  uint8_t chipVal = 0x00;
  parseHexPrompt(F("    Expected chip ID value (hex): "), chipVal, 0x00, true);
  info.expected_chip_id = chipVal;

  uint8_t maskVal = 0xFF;
  parseHexPrompt(F("    ID mask (hex, blank for FF): "), maskVal, 0xFF, true);
  info.id_mask = maskVal;

  saveEepromDevice((size_t)slot, info);
  Serial.print(F("\n  Saved to EEPROM slot "));
  Serial.println(slot);
  return true;
}

void describeI2CAddress(uint8_t addr) {
  String confirmedNames;
  String possibleNames;
  bool   hasDatabaseMatch = false;

  for (size_t i = 0; i < NUM_I2C_DEVICES; i++) {
    I2C_DeviceInfo info;
    memcpy_P(&info, &i2c_devices[i], sizeof(I2C_DeviceInfo));
    considerDeviceMatch(info, addr, confirmedNames, possibleNames, hasDatabaseMatch);
  }

  for (size_t i = 0; i < EEPROM_MAX_DEVICES; i++) {
    I2C_DeviceInfo info;
    loadEepromDevice(i, info);
    if (!eepromSlotUsed(info)) continue;
    considerDeviceMatch(info, addr, confirmedNames, possibleNames, hasDatabaseMatch);
  }

  Serial.print(F("  Found: 0x"));
  printHexByte(addr);

  if (hasDatabaseMatch && confirmedNames.length() > 0) {
    Serial.print(F(". CONFIRMED: "));
    Serial.println(confirmedNames);
  } else if (hasDatabaseMatch) {
    Serial.print(F(". ChipID not matched. Possible "));
    Serial.println(possibleNames);
  } else {
    Serial.println(F(". No match in database or EEPROM"));
    promptAddDeviceToEeprom(addr);
  }
}

// --------- Commands ----------

void cmdHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  scan"));
  Serial.println(F("  dump <addr> [start] [end]"));
  Serial.println(F("  read <addr> <reg>"));
  Serial.println(F("  write <addr> <reg> <val>"));
  Serial.println(F("All values are hex, e.g. 0x68 00 7F"));
}

void cmdScan() {
  Serial.println(F("I2C scan (7-bit addresses):"));

  for (uint8_t addr = 1; addr < 0x80; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      describeI2CAddress(addr);
    }
  }
  Serial.println(F("Scan complete."));
}

void cmdDump(const String *tokens, int ntokens) {
  if (ntokens < 2) {
    Serial.println(F("Usage: dump <addr> [start] [end]"));
    return;
  }

  bool ok;
  uint16_t addr16 = parseHex(tokens[1], ok);
  if (!ok || addr16 > 0x7F) {
    Serial.println(F("Invalid addr"));
    return;
  }
  uint8_t addr = (uint8_t)addr16;

  uint8_t start = 0x00;
  uint8_t end   = 0xFF;

  if (ntokens >= 3) {
    uint16_t s16 = parseHex(tokens[2], ok);
    if (!ok || s16 > 0xFF) {
      Serial.println(F("Invalid start reg"));
      return;
    }
    start = (uint8_t)s16;
  }
  if (ntokens >= 4) {
    uint16_t e16 = parseHex(tokens[3], ok);
    if (!ok || e16 > 0xFF) {
      Serial.println(F("Invalid end reg"));
      return;
    }
    end = (uint8_t)e16;
  }

  Serial.print(F("Dump addr 0x"));
  printHexByte(addr);
  Serial.print(F(" regs 0x"));
  printHexByte(start);
  Serial.print(F(" to 0x"));
  printHexByte(end);
  Serial.println();

  if (!i2cReadRange(addr, start, end)) {
    Serial.println(F("Dump failed."));
  }
}

void cmdRead(const String *tokens, int ntokens) {
  if (ntokens < 3) {
    Serial.println(F("Usage: read <addr> <reg>"));
    return;
  }

  bool ok;
  uint16_t addr16 = parseHex(tokens[1], ok);
  if (!ok || addr16 > 0x7F) {
    Serial.println(F("Invalid addr"));
    return;
  }
  uint16_t reg16 = parseHex(tokens[2], ok);
  if (!ok || reg16 > 0xFF) {
    Serial.println(F("Invalid reg"));
    return;
  }

  uint8_t addr = (uint8_t)addr16;
  uint8_t reg  = (uint8_t)reg16;
  uint8_t value;

  if (i2cReadRegister(addr, reg, value)) {
    Serial.print(F("0x"));
    printHexByte(addr);
    Serial.print(F("[0x"));
    printHexByte(reg);
    Serial.print(F("] = 0x"));
    printHexByte(value);
    Serial.println();
  } else {
    Serial.println(F("Read failed"));
  }
}

void cmdWrite(const String *tokens, int ntokens) {
  if (ntokens < 4) {
    Serial.println(F("Usage: write <addr> <reg> <val>"));
    return;
  }

  bool ok;
  uint16_t addr16 = parseHex(tokens[1], ok);
  if (!ok || addr16 > 0x7F) {
    Serial.println(F("Invalid addr"));
    return;
  }
  uint16_t reg16 = parseHex(tokens[2], ok);
  if (!ok || reg16 > 0xFF) {
    Serial.println(F("Invalid reg"));
    return;
  }
  uint16_t val16 = parseHex(tokens[3], ok);
  if (!ok || val16 > 0xFF) {
    Serial.println(F("Invalid val"));
    return;
  }

  uint8_t addr = (uint8_t)addr16;
  uint8_t reg  = (uint8_t)reg16;
  uint8_t val  = (uint8_t)val16;

  if (i2cWriteRegister(addr, reg, val)) {
    Serial.print(F("Wrote 0x"));
    printHexByte(val);
    Serial.print(F(" to 0x"));
    printHexByte(addr);
    Serial.print(F("[0x"));
    printHexByte(reg);
    Serial.println(F("] OK"));
  } else {
    Serial.println(F("Write failed"));
  }
}

// --------- Command parsing ----------

String inputLine;

void handleLine(const String &lineRaw) {
  String line = lineRaw;
  line.trim();
  if (line.length() == 0) return;

  // Split into tokens
  const int MAX_TOKENS = 8;
  String tokens[MAX_TOKENS];
  int ntokens = 0;

  int from = 0;
  while (from < line.length() && ntokens < MAX_TOKENS) {
    int idx = line.indexOf(' ', from);
    if (idx < 0) idx = line.length();
    String tok = line.substring(from, idx);
    tok.trim();
    if (tok.length() > 0) {
      tokens[ntokens++] = tok;
    }
    from = idx + 1;
  }

  if (ntokens == 0) return;

  String cmd = tokens[0];
  cmd.toLowerCase();

  if (cmd == "help" || cmd == "?") {
    cmdHelp();
  } else if (cmd == "scan") {
    cmdScan();
  } else if (cmd == "dump") {
    cmdDump(tokens, ntokens);
  } else if (cmd == "read") {
    cmdRead(tokens, ntokens);
  } else if (cmd == "write") {
    cmdWrite(tokens, ntokens);
  } else {
    Serial.print(F("Unknown cmd: "));
    Serial.println(cmd);
    cmdHelp();
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {;} // on some boards
  Wire.begin();
  // Optionally: Wire.setClock(100000); // 100 kHz

  Serial.println(F("I2C Debug Shell Ready"));
  Serial.println(F("Type 'help' for commands."));
  Serial.print(F("> "));
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') {
      // ignore
    } else if (c == '\n') {
      Serial.println();
      handleLine(inputLine);
      inputLine = "";
      Serial.print(F("> "));
    } else {
      // Basic line editing: backspace
      if ((c == 8 || c == 127) && inputLine.length() > 0) {
        inputLine.remove(inputLine.length() - 1);
        Serial.print("\b \b");
      } else if (isPrintable(c)) {
        inputLine += c;
        Serial.print(c);
      }
    }
  }
}
