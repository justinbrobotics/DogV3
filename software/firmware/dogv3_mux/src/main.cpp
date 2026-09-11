/*
 * DogV3 onboard ESP32 dual-UART transport.
 *
 * Current role: a transparent transport for D1 commissioning and D2 Home-PC
 * operation/tuning. The host runs the control loop and sends fully-formed
 * Feetech packets; this firmware routes each packet to the correct half-duplex
 * bus and wraps any servo reply back toward the host.
 *
 * Two host links, selected at build time:
 *   - USB-CDC (Serial)            : wired, for flashing + commissioning.
 *   - WiFi SoftAP + TCP (optional): wireless, for untethered driving.
 *     Build with -D DOGV3_WIFI (see platformio.ini env "esp32-wifi").
 *     The ESP32 hosts AP "DogV3" (192.168.4.1) and a raw TCP server on
 *     port 3333 that is byte-for-byte identical to the USB link. Whichever host
 *     (TCP client if connected, else USB) is active drives the buses.
 *
 * Bus wiring:
 *   Bus A (Serial1, RX=GPIO16, TX=GPIO17) @ 1 Mbaud = REAR  (BL, BR)
 *   Bus B (Serial2, RX=GPIO26, TX=GPIO25) @ 1 Mbaud = FRONT (FL, FR)
 *
 * Wire format (see dogv3/driver/mux.py â€” keep in lockstep):
 *   host -> ESP32:  0xFE  BUS  LEN  payload[LEN]  CKSUM
 *   ESP32 -> host:  0xFD  BUS  LEN  payload[LEN]  CKSUM
 *   CKSUM: (BUS + LEN + sum(payload)) & 0xFF
 * ASCII control: VERSION / PING->PONG / SCAN (newline-terminated).
 *
 * Boots disarmed: forwards nothing on its own; acts only on host frames.
 */
#include <Arduino.h>

#ifdef DOGV3_WIFI
#include <WiFi.h>
#ifndef DOGV3_AP_SSID
#define DOGV3_AP_SSID "DogV3"
#endif
#ifndef DOGV3_AP_PASS
#error "Define a private DOGV3_AP_PASS when explicitly enabling optional WiFi firmware"
#endif
#ifndef DOGV3_TCP_PORT
#define DOGV3_TCP_PORT 3333
#endif
static WiFiServer tcpServer(DOGV3_TCP_PORT);
static WiFiClient tcpClient;
#endif

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "DOGV3-MUX v1.0"
#endif

static const uint8_t START_HOST_TO_ESP = 0xFE;
static const uint8_t START_ESP_TO_HOST = 0xFD;

static const int BUS_A = 0;
static const int BUS_B = 1;

static const int A_RX = 16, A_TX = 17;
static const int B_RX = 26, B_TX = 25;
static const uint32_t SERVO_BAUD = 1000000;

static const uint32_t REPLY_INTERBYTE_US = 1500;
static const uint32_t REPLY_TIMEOUT_MS = 12;

static uint8_t payloadBuf[300];
static uint8_t replyBuf[300];

// Where replies/text go for the host link currently being serviced.
static Print *OUT = &Serial;

HardwareSerial &busSerial(int bus) { return bus == BUS_A ? Serial1 : Serial2; }

void sendMuxReply(int bus, const uint8_t *data, size_t len) {
  uint8_t cksum = (uint8_t)(bus + len);
  OUT->write(START_ESP_TO_HOST);
  OUT->write((uint8_t)bus);
  OUT->write((uint8_t)len);
  for (size_t i = 0; i < len; i++) {
    OUT->write(data[i]);
    cksum += data[i];
  }
  OUT->write(cksum);
}

size_t collectReply(int bus) {
  HardwareSerial &s = busSerial(bus);
  size_t n = 0;
  uint32_t startMs = millis();
  while (!s.available()) {
    if (millis() - startMs > REPLY_TIMEOUT_MS) return 0;
  }
  uint32_t lastByteUs = micros();
  while (n < sizeof(replyBuf)) {
    if (s.available()) {
      replyBuf[n++] = (uint8_t)s.read();
      lastByteUs = micros();
    } else if (micros() - lastByteUs > REPLY_INTERBYTE_US) {
      break;
    }
  }
  return n;
}

void forwardToBus(int bus, const uint8_t *payload, size_t len) {
  HardwareSerial &s = busSerial(bus);
  while (s.available()) s.read();
  s.write(payload, len);
  s.flush();
  bool broadcast = (len >= 3 && payload[2] == 0xFE);
  if (broadcast) return;
  size_t r = collectReply(bus);
  if (r > 0) sendMuxReply(bus, replyBuf, r);
}

bool pingId(int bus, uint8_t id) {
  HardwareSerial &s = busSerial(bus);
  while (s.available()) s.read();
  uint8_t cks = (uint8_t)(~(id + 0x02 + 0x01));
  uint8_t pkt[6] = {0xFF, 0xFF, id, 0x02, 0x01, cks};
  s.write(pkt, 6);
  s.flush();
  return collectReply(bus) >= 4;
}

void doScan() {
  for (int bus = BUS_A; bus <= BUS_B; bus++) {
    OUT->print("SCAN ");
    OUT->print(bus == BUS_A ? "A:" : "B:");
    for (uint8_t id = 1; id < 100; id++) {
      if (pingId(bus, id)) {
        OUT->print(' ');
        OUT->print(id);
      }
    }
    OUT->println();
  }
  OUT->println("SCAN DONE");
}

enum RxState { IDLE, MUX_BUS, MUX_LEN, MUX_PAYLOAD, MUX_CKSUM, ASCII_LINE };
static RxState rxState = IDLE;
static int muxBus = 0, muxLen = 0, muxIdx = 0;
static uint8_t muxCksum = 0;
static char asciiBuf[32];
static int asciiIdx = 0;

void handleAscii(const char *line) {
  if (strcasecmp(line, "VERSION") == 0) {
    OUT->println(FIRMWARE_VERSION);
  } else if (strcasecmp(line, "PING") == 0) {
    OUT->println("PONG");
  } else if (strcasecmp(line, "SCAN") == 0) {
    doScan();
  } else if (line[0] != '\0') {
    OUT->print("ERR unknown cmd: ");
    OUT->println(line);
  }
}

// Process all currently-available bytes from one host stream.
void pumpHost(Stream &in) {
  while (in.available()) {
    uint8_t b = (uint8_t)in.read();
    switch (rxState) {
      case IDLE:
        if (b == START_HOST_TO_ESP) {
          rxState = MUX_BUS;
        } else if (b == '\r' || b == '\n') {
        } else {
          asciiIdx = 0;
          asciiBuf[asciiIdx++] = (char)b;
          rxState = ASCII_LINE;
        }
        break;
      case ASCII_LINE:
        if (b == '\n' || b == '\r') {
          asciiBuf[asciiIdx] = '\0';
          handleAscii(asciiBuf);
          rxState = IDLE;
        } else if (asciiIdx < (int)sizeof(asciiBuf) - 1) {
          asciiBuf[asciiIdx++] = (char)b;
        } else {
          rxState = IDLE;
        }
        break;
      case MUX_BUS:
        muxBus = b;
        rxState = MUX_LEN;
        break;
      case MUX_LEN:
        muxLen = b;
        muxIdx = 0;
        muxCksum = (uint8_t)(muxBus + muxLen);
        rxState = (muxLen == 0) ? MUX_CKSUM : MUX_PAYLOAD;
        break;
      case MUX_PAYLOAD:
        payloadBuf[muxIdx++] = b;
        muxCksum += b;
        if (muxIdx >= muxLen) rxState = MUX_CKSUM;
        break;
      case MUX_CKSUM:
        if (b == muxCksum && (muxBus == BUS_A || muxBus == BUS_B)) {
          forwardToBus(muxBus, payloadBuf, muxLen);
        }
        rxState = IDLE;
        break;
    }
  }
}

void setup() {
  Serial.begin(SERVO_BAUD);
  Serial1.begin(SERVO_BAUD, SERIAL_8N1, A_RX, A_TX);
  Serial2.begin(SERVO_BAUD, SERIAL_8N1, B_RX, B_TX);
  delay(200);
  Serial.println(FIRMWARE_VERSION);
#ifdef DOGV3_WIFI
  WiFi.mode(WIFI_AP);
  WiFi.softAP(DOGV3_AP_SSID, DOGV3_AP_PASS);
  tcpServer.begin();
  tcpServer.setNoDelay(true);
  Serial.print("WIFI AP ");
  Serial.print(DOGV3_AP_SSID);
  Serial.print(" @ ");
  Serial.print(WiFi.softAPIP());
  Serial.print(":");
  Serial.println(DOGV3_TCP_PORT);
#endif
}

void loop() {
#ifdef DOGV3_WIFI
  if (!tcpClient || !tcpClient.connected()) {
    WiFiClient c = tcpServer.available();
    if (c) {
      tcpClient = c;
      tcpClient.setNoDelay(true);
      rxState = IDLE;  // fresh framing for the new client
    }
  }
  if (tcpClient && tcpClient.connected() && tcpClient.available()) {
    OUT = &tcpClient;
    pumpHost(tcpClient);
    return;  // service the wireless host this tick
  }
#endif
  if (Serial.available()) {
    OUT = &Serial;
    pumpHost(Serial);
  }
}
