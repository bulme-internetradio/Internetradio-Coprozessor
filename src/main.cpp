// Libraries
#include <Arduino.h>

#include <ooPinChangeInt.h>
#include <AdaEncoder.h>

#include <SPI.h>
#include <Wire.h>

#include <radio.h>
#include <SI4703.h>
#include <RDSParser.h>

#define PIN_POWER_CTL   4   // Pi POWER
#define PIN_LCD_PWM     9   // PWM (LCD)
#define PIN_AMP_ENABLE  8   // not-SHUTDOWN (Amplifier)
#define PIN_ATT_SS      SS  // Attenuator SPI Chip Select

#define PIN_ROT_0_SW   A2   // Rotary Encoder 0 Button
#define PIN_ROT_1_SW    5   // Rotary Encoder 1 Button

#define PIN_FM_RST     A3   // Si4703 Reset Pin

// Zeit, die nach einem Abschaltbefehl gewartet wird, bevor die 5V Versorgung wirklich getrennt wird
#define POWER_OFF_WAIT_TIME 15000

typedef String (*commandAction)(String); // Typ der Befehlsfunktion

// Liste der Befehle
enum Commands {
  c_notfound,       // 0
  c_change_att,     // 1
  c_set_amp_power,  // 2
  c_set_lcd_pwm,    // ...
  c_power_off,
  c_fm_init,
  c_fm_deinit,
  c_fm_scan_up,
  c_fm_scan_down,
  c_fm_set_freq,
  c_fm_set_mono,
  c_fm_set_vol,
  c_count
};

// Befehlsarray
commandAction commandMap[c_count];
void setupCommandMap();

// Liste der Ereignisse
enum Events {
  e_cmd_response,
  e_rot0_dir1,
  e_rot0_dir2,
  e_rot0_btn,
  e_rot1_btn,
  e_change_att,
  e_fm_scan_complete,
  e_fm_rds_name,
  e_fm_rds_text
};

// Funktion, die eine Ereignisnachricht versendet
void printEvent(Events event, String args = "") {
  Serial.print(event);
  Serial.print("+");
  Serial.println(args);
}

// Drehimpulsgeber
int8_t clicks = 0;
AdaEncoder encoder_rot0 = AdaEncoder('a', A0, A1);
AdaEncoder encoder_rot1 = AdaEncoder('b', 7, 6);

// Aktuelle Lautstärkedämpfung (min: 0, max: 127)
uint8_t currentAttenuation = 10;

// Funktion um Lautstärkedämpfung an den Attenuator zu übertragen
void setAttenuation(const uint8_t v) {
  currentAttenuation = v;
  digitalWrite(SS, LOW);
  delay(1);
  SPI.transfer(0x00); // Channel 0
  SPI.transfer(v);
  delay(1);
  digitalWrite(SS, HIGH);

  delay(1);

  digitalWrite(SS, LOW);
  delay(1);
  SPI.transfer(0x01); // Channel 1
  SPI.transfer(v);
  delay(1);
  digitalWrite(SS, HIGH);
}

// Standby-Modus verlassen
void powerOn() {
  digitalWrite(PIN_POWER_CTL, HIGH); // 5V Versorgung einschalten
  delay(100); // Warten, damit die Komponenten Zeit zum Initialisieren haben
  digitalWrite(PIN_LCD_PWM, HIGH); // Bildschirmbeleuchtung einschalten
  digitalWrite(PIN_AMP_ENABLE, HIGH); // Verstärker einschalten
}

// Standby-Modus aktivieren
void powerOff() {
  digitalWrite(PIN_AMP_ENABLE, LOW); // Verstärker ausschalten
  digitalWrite(PIN_LCD_PWM, LOW); // Bildschirmbeleuchtung ausschalten
  delay(100); // Warten, damit der Verstärker popfrei abschalten kann
  digitalWrite(PIN_POWER_CTL, LOW); // 5V Versorgung abschalten
}

// FM-Modul
bool is_fm_initialized = false;
SI4703 radio = SI4703(PIN_FM_RST);
RDSParser rds;

void RDS_process(uint16_t block1, uint16_t block2, uint16_t block3, uint16_t block4) {
  rds.processData(block1, block2, block3, block4);
}

// RDS Daten verarbeiten
void fmServiceNameCallback(char *name)
{
  for (uint8_t i = 0; i < 8; i++) {
    if (name[i] == '\n') name[i] = '\r';
  }
  printEvent(e_fm_rds_name, name);
}

void fmTextCallback(char *text)
{
  for (uint8_t i = 0; i < 8; i++) {
    if (text[i] == '\n') text[i] = '\r';
  }
  printEvent(e_fm_rds_text, text);
}

// Diese Funktion wird einmal nach dem Reset aufgerufen
void setup() {
  // Pins
  pinMode(PIN_POWER_CTL, OUTPUT);
  digitalWrite(PIN_POWER_CTL, HIGH);

  pinMode(PIN_LCD_PWM, OUTPUT);
  digitalWrite(PIN_LCD_PWM, HIGH);

  pinMode(PIN_AMP_ENABLE, OUTPUT);
  digitalWrite(PIN_AMP_ENABLE, HIGH);

  pinMode(PIN_ROT_0_SW, INPUT_PULLUP);
  pinMode(PIN_ROT_1_SW, INPUT_PULLUP);

  // Attenuator SPI konfiguration
  pinMode(PIN_ATT_SS, OUTPUT);
  digitalWrite(PIN_ATT_SS, HIGH);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE0);
  SPI.begin();

  // Befehlsarray füllen
  setupCommandMap();

  // Serial
  Serial.begin(9600);
}

// Vorheriger Zustand Drehknopftaster, damit sie
// auf eine fallende Flanke "triggern"
bool rot_0_pressed = false;
bool rot_1_pressed = false;

// Diese Funktion wird kontinuierlich aufgerufen
void loop() {
  // Drehimpulsgeber auslesen
  AdaEncoder *thisEncoder = NULL;
  thisEncoder = AdaEncoder::genie();
  if (thisEncoder != NULL) {
    clicks = thisEncoder->getClicks();
    if (thisEncoder->getID() == 'a') {
      if (clicks > 0) {
        printEvent(e_rot0_dir1);
      }
      if (clicks < 0) {
        printEvent(e_rot0_dir2);
      }
    } else {
      if (clicks > 0) {
        setAttenuation(currentAttenuation + 1);
        printEvent(e_change_att, String(currentAttenuation));
      }
      if (clicks < 0) {
        setAttenuation(currentAttenuation - 1);
        printEvent(e_change_att, String(currentAttenuation));
      }
    }
  }
  // Drehknopftaster auslesen
  if (digitalRead(PIN_ROT_0_SW) == LOW) {
    if (!rot_0_pressed) {
      rot_0_pressed = true;
      printEvent(e_rot0_btn);
    }
  } else {
    rot_0_pressed = false;
  }
  if (digitalRead(PIN_ROT_1_SW) == LOW) {
    if (!rot_1_pressed) {
      rot_1_pressed = true;
      printEvent(e_rot1_btn);
    }
  } else {
    rot_1_pressed = false;
  }
  // FM-RDS-Daten empfangen
  if (is_fm_initialized) {
    radio.checkRDS();
  }
  // Befehle empfangen und verarbeiten
  if (Serial.available() > 0) { // Wenn etwas im Serial-Buffer ist
    int cmd = Serial.readStringUntil('+').toInt(); // Befehl (Teil vor +) in int verwandeln
    String args = Serial.readStringUntil('\n'); // Argumente (Teil nach +) lesen
    if (cmd < 0 || cmd >= c_count) cmd = 0; // Bei unbekanntem Befehl, Fehler zurückgeben
    String res = (*commandMap[cmd])(args); // Funktion des Befehls aus Array lesen und ausführen
    printEvent(e_cmd_response, res); // Antwort des Befehls zurückgeben
  }
}

// Dieser Befehl wird für unbekannte Befehle ausgeführt
// Parameter: keine
// Antwort: immer "notfound"
String cmd_notfound(String args) {
  return "notfound";
}

// Dieser Befehl kann die Lautstärkedämpfung anpassen
// Parameter: Zahl zwischen 0 und 127 -> Attenuation level
// Antwort: Die übergebene Zahl als Bestätigung zurück
String cmd_change_att(String args) {
  uint16_t val = args.toInt(); // Argument als Zahl lesen
  uint8_t att = val;
  setAttenuation(att);
  return String(att);
}

// Dieser Befehl kann den Verstärker ein/ausschalten
// Parameter: 0 für aus, 1 für ein
// Antwort: "off" wenn nun ausgeschalten,
//          "on" wenn nun eingeschalten
String cmd_set_amp_power(String args) {
  if (args.charAt(0) == '0') { // Wenn Argument dem Charakter '0' entspricht
    digitalWrite(PIN_AMP_ENABLE, LOW);
    return "off";
  } else {
    digitalWrite(PIN_AMP_ENABLE, HIGH);
    return "on";
  }
}

// Dieser Befehl kann die Displayhelligkeit kontrollieren
// Parameter: Zahl zwischen 0 und 255 -> Displayhelligkeit
// Antwort: Die übergebene Zahl als Bestätigung zurück
String cmd_set_lcd_pwm(String args) {
  uint16_t val = args.toInt();
  uint8_t pwmVal = val;
  analogWrite(PIN_LCD_PWM, pwmVal); // Über PWM die Helligkeit einstellen
  return String(pwmVal);
}

// Dieser Befehl aktiviert den Standby-Modus
// Parameter: keine
// Antwort: keine
String cmd_power_off(String args) {
  delay(POWER_OFF_WAIT_TIME);
  powerOff();
  return "";
}

// Dieser Befehl initialisiert das FM-Modul
// Parameter: keine
// Antwort: ok
String cmd_fm_init(String args) {
  if (is_fm_initialized) {
    radio.setMute(false);
    return "ok";
  }
  is_fm_initialized = true;
  radio.init();
  radio.setBand(RADIO_BAND_FM);
  radio.setMono(false);
  radio.setMute(false);
  radio.setVolume(5);
  radio.attachReceiveRDS(RDS_process);
  rds.attachServicenNameCallback(fmServiceNameCallback);
  rds.attachTextCallback(fmTextCallback);
  return "ok";
}

// Dieser Befehl deinitialisiert das FM-Modul
// Parameter: keine
// Antwort: ok (oder "noinit" wenn nicht initialisiert)
String cmd_fm_deinit(String args) {
  if (!is_fm_initialized) return "noinit";
  is_fm_initialized = false;
  radio.setMute(true);
  return "ok";
}

// Dieser Befehl sucht den nächsthöheren FM-Sender
// Parameter: keine
// Antwort: Frequenz in Mhz * 100 (oder "noinit" wenn nicht initialisiert)
String cmd_fm_scan_up(String args) {
  if (!is_fm_initialized) return "noinit";
  radio.seekUp();
  // printEvent(e_fm_scan_complete, String(radio.getFrequency()));
  return String(radio.getFrequency());
}

// Dieser Befehl sucht den nächstniedrigeren FM-Sender
// Parameter: keine
// Antwort: Frequenz in Mhz * 100 (oder "noinit" wenn nicht initialisiert)
String cmd_fm_scan_down(String args) {
  if (!is_fm_initialized) return "noinit";
  radio.seekDown();
  // printEvent(e_fm_scan_complete, String(radio.getFrequency()));
  return String(radio.getFrequency());
}

// Dieser Befehl legt die FM-Frequenz absolut fest
// Parameter: Frequenz in Mhz * 100
// Antwort: Frequenz in Mhz * 100 (oder "noinit" wenn nicht initialisiert)
String cmd_fm_set_freq(String args) {
  if (!is_fm_initialized) return "noinit";
  uint16_t val = args.toInt();
  radio.setFrequency(val);
  return String(radio.getFrequency());
}

// Dieser Befehl schaltet den Mono-Modus des FM-Modul ein oder aus
// Parameter: keine
// Antwort: "off" wenn nun ausgeschalten,
//          "on" wenn nun eingeschalten
//          "noinit" wenn nicht initialisiert
String cmd_fm_set_mono(String args) {
  if (!is_fm_initialized) return "noinit";
  if (args.charAt(0) == '0') { // Wenn Argument dem Charakter '0' entspricht
    radio.setMono(false);
    return "off";
  } else {
    radio.setMono(true);
    return "on";
  }
}

// Dieser Befehl setzt die interne Lautsärke am FM-Modul
// Parameter: Zahl zwischen 0 und 15: Lautstärke
// Antwort: die neue Lautstärke (oder "noinit" wenn nicht initialisiert)
String cmd_fm_set_vol(String args) {
  if (!is_fm_initialized) return "noinit";
  uint16_t val = args.toInt();
  uint8_t vol = val;
  radio.setVolume(vol);
  return String(radio.getVolume());
}

// Befehlsfunktionen in Befehlsarray speichern, damit
// sie anhand ihrem enum Wert aufgerufen werden können
void setupCommandMap() {
  commandMap[c_notfound] = cmd_notfound;
  commandMap[c_change_att] = cmd_change_att;
  commandMap[c_set_amp_power] = cmd_set_amp_power;
  commandMap[c_set_lcd_pwm] = cmd_set_lcd_pwm;
  commandMap[c_power_off] = cmd_power_off;
  commandMap[c_fm_init] = cmd_fm_init;
  commandMap[c_fm_deinit] = cmd_fm_deinit;
  commandMap[c_fm_scan_up] = cmd_fm_scan_up;
  commandMap[c_fm_scan_down] = cmd_fm_scan_down;
  commandMap[c_fm_set_freq] = cmd_fm_set_freq;
  commandMap[c_fm_set_mono] = cmd_fm_set_mono;
  commandMap[c_fm_set_vol] = cmd_fm_set_vol;
}