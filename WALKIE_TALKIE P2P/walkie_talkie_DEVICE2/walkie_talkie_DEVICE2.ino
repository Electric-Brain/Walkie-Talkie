/**
 * ============================================================
 *  ESP32 WALKIE-TALKIE — DEVICE #2
 *  MY  MAC: 14:33:5C:03:DF:C8
 *  PEER MAC: A4:F0:0F:5C:08:98  (Device #1)
 *  Arduino Core v3.x / IDF v5.x
 * ============================================================
 *  KEY FIXES IN THIS VERSION:
 *  1. WiFi forced to channel 1 (both boards must use same channel)
 *  2. esp_now_add_peer error code printed for diagnosis
 *  3. esp_now_send error code printed — shows exactly why TX fails
 *  4. MAC address verified on boot — mismatch = no comms
 *  5. ADC offset correction (your mic reads low ~200-400, not 2048)
 * ============================================================
 */

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <esp_timer.h>
#include <math.h>

// ─────────────────────────────────────────────
//  !! IMPORTANT !!
//  On boot, Serial Monitor will print "My MAC: XX:XX:XX:XX:XX:XX"
//  Make sure that matches what is written below as MY expected MAC.
//  If it doesn't match, swap the two .ino files between boards.
// ─────────────────────────────────────────────
#define MY_EXPECTED_MAC   "14:33:5C:03:DF:C8"
static const uint8_t PEER_MAC[6] = {0xA4, 0xF0, 0x0F, 0x5C, 0x08, 0x98};

// ─────────────────────────────────────────────
//  AUDIO
// ─────────────────────────────────────────────
#define SAMPLE_RATE_HZ     8000
#define SAMPLE_INTERVAL_US (1000000 / SAMPLE_RATE_HZ)
#define SAMPLES_PER_PACKET 160
#define MAX_PAYLOAD_BYTES  SAMPLES_PER_PACKET

// ─────────────────────────────────────────────
//  RX RING BUFFER
// ─────────────────────────────────────────────
#define RX_RING_SIZE 4096
static uint8_t           gRxRing[RX_RING_SIZE];
static volatile uint32_t gRxWriteIdx = 0;
static volatile uint32_t gRxReadIdx  = 0;

// ─────────────────────────────────────────────
//  GPIO
// ─────────────────────────────────────────────
#define PTT_GPIO 14

// ─────────────────────────────────────────────
//  PACKET
// ─────────────────────────────────────────────
#pragma pack(push, 1)
struct AudioPacket {
  uint8_t  magic;
  uint16_t seq;
  uint8_t  len;
  uint8_t  payload[MAX_PAYLOAD_BYTES];
};
#pragma pack(pop)

// ─────────────────────────────────────────────
//  STATE
// ─────────────────────────────────────────────
typedef enum { MODE_RX, MODE_TX } DeviceMode;
volatile DeviceMode gMode = MODE_RX;

static uint8_t  gTxEncoded[MAX_PAYLOAD_BYTES];
static int      gTxSampleIdx  = 0;
static uint16_t gTxSeq        = 0;
static int16_t  gTxPrevSample = 2048;

static uint16_t          gRxExpectedSeq = 0;
static int16_t           gRxPrevDecoded = 2048;
static uint8_t           gLastDacVal    = 128;
static volatile uint32_t gPktsReceived  = 0;
static volatile uint32_t gPktsSent      = 0;
static volatile uint32_t gSendFails     = 0;
static volatile uint32_t gRxDropped     = 0;

static int16_t gMicBaseline = 2048;
static bool    gBaselineSet = false;

static float gHpPrev   = 0.0f;
static float gHpOutput = 0.0f;
static float gAgcGain  = 4.0f;
static float gAgcPeak  = 0.0f;

static esp_timer_handle_t gTxTimer  = nullptr;
static esp_timer_handle_t gDacTimer = nullptr;

// ─────────────────────────────────────────────
//  SIGNAL PROCESSING
// ─────────────────────────────────────────────
static inline float highPass(float s) {
  float hp  = s - gHpPrev + 0.995f * gHpOutput;
  gHpPrev   = s;
  gHpOutput = hp;
  return hp;
}

static inline float applyAGC(float s) {
  float a  = fabsf(s);
  gAgcPeak += (a > gAgcPeak) ? 0.02f * (a - gAgcPeak) : 0.0005f * (a - gAgcPeak);
  gAgcGain  = (gAgcPeak > 5.0f) ? (1800.0f / gAgcPeak) : 4.0f;
  s        *= gAgcGain;
  if (s >  2047.f) s =  2047.f;
  if (s < -2048.f) s = -2048.f;
  return s;
}

static inline uint8_t encodeDelta(int16_t sample, int16_t &prev) {
  int16_t d = sample - prev;
  if (d >  127) d =  127;
  if (d < -128) d = -128;
  prev += d;
  return (uint8_t)(int8_t)d;
}

static inline int16_t decodeDelta(uint8_t enc, int16_t &prev) {
  int16_t s = prev + (int8_t)enc;
  if (s > 4095) s = 4095;
  if (s <    0) s = 0;
  prev = s;
  return s;
}

// ─────────────────────────────────────────────
//  RING BUFFER
// ─────────────────────────────────────────────
static inline void rxPush(uint8_t b) {
  uint32_t next = (gRxWriteIdx + 1) % RX_RING_SIZE;
  if (next == gRxReadIdx) return;
  gRxRing[gRxWriteIdx] = b;
  gRxWriteIdx = next;
}
static inline bool rxPop(uint8_t &out) {
  if (gRxReadIdx == gRxWriteIdx) return false;
  out        = gRxRing[gRxReadIdx];
  gRxReadIdx = (gRxReadIdx + 1) % RX_RING_SIZE;
  return true;
}

// ─────────────────────────────────────────────
//  SPEAKER TEST
// ─────────────────────────────────────────────
static void playSpeakerTest() {
  Serial.println("[SPEAKER TEST] Playing 1kHz beep — you should hear a tone!");
  dac_output_enable(DAC_CHANNEL_1);
  uint32_t phase = 0;
  uint32_t start = millis();
  while (millis() - start < 2000) {
    float   angle = 2.0f * M_PI * (float)phase * 1000 / SAMPLE_RATE_HZ;
    uint8_t val   = (uint8_t)(128 + 110 * sinf(angle));
    dac_output_voltage(DAC_CHANNEL_1, val);
    phase++;
    delayMicroseconds(SAMPLE_INTERVAL_US);
  }
  dac_output_voltage(DAC_CHANNEL_1, 128);
  Serial.println("[SPEAKER TEST] Done.\n");
}

// ─────────────────────────────────────────────
//  AUTO MIC BASELINE CALIBRATION
// ─────────────────────────────────────────────
static void calibrateMicBaseline() {
  long sum = 0;
  for (int i = 0; i < 200; i++) {
    sum += adc1_get_raw(ADC1_CHANNEL_6);
    delayMicroseconds(500);
  }
  gMicBaseline = (int16_t)(sum / 200);
  gBaselineSet = true;
  Serial.printf("[CAL] Mic baseline = %d  (ideal=2048, yours=%d — offset applied)\n",
                gMicBaseline, gMicBaseline);
}

// ─────────────────────────────────────────────
//  ESP-NOW CALLBACKS
// ─────────────────────────────────────────────
static void onDataReceived(const esp_now_recv_info_t *info,
                           const uint8_t *data, int len) {
  gPktsReceived++;
  if (gMode != MODE_RX) return;
  if (len < (int)sizeof(AudioPacket)) return;
  const AudioPacket *pkt = (const AudioPacket *)data;
  if (pkt->magic != 0xAB) return;

  uint16_t gap = (uint16_t)(pkt->seq - gRxExpectedSeq);
  if (gap > 0 && gap < 100) {
    uint32_t fill = (uint32_t)gap * SAMPLES_PER_PACKET;
    if (fill > RX_RING_SIZE / 2) fill = RX_RING_SIZE / 2;
    for (uint32_t i = 0; i < fill; i++) rxPush(128);
    gRxDropped += gap;
  }
  gRxExpectedSeq = pkt->seq + 1;

  for (int i = 0; i < pkt->len; i++) {
    int16_t s = decodeDelta(pkt->payload[i], gRxPrevDecoded);
    rxPush((uint8_t)(s >> 4));
  }
}

static void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) gPktsSent++;
  else                                gSendFails++;
}

// ─────────────────────────────────────────────
//  TIMER CALLBACKS
// ─────────────────────────────────────────────
static void IRAM_ATTR txTimerISR(void *arg) {
  if (gMode != MODE_TX) return;

  int16_t raw     = (int16_t)adc1_get_raw(ADC1_CHANNEL_6);
  float   centred = (float)(raw - gMicBaseline);
  float   filt    = highPass(centred);
  float   gain    = applyAGC(filt);
  int16_t s       = (int16_t)(gain + 2048.0f);
  if (s < 0)    s = 0;
  if (s > 4095) s = 4095;

  gTxEncoded[gTxSampleIdx++] = encodeDelta(s, gTxPrevSample);

  if (gTxSampleIdx >= SAMPLES_PER_PACKET) {
    AudioPacket pkt;
    pkt.magic = 0xAB;
    pkt.seq   = gTxSeq++;
    pkt.len   = SAMPLES_PER_PACKET;
    memcpy(pkt.payload, gTxEncoded, SAMPLES_PER_PACKET);
    esp_err_t err = esp_now_send(PEER_MAC, (uint8_t *)&pkt, sizeof(pkt));
    if (err != ESP_OK) gSendFails++;
    gTxSampleIdx = 0;
  }
}

static void IRAM_ATTR dacTimerISR(void *arg) {
  if (gMode != MODE_RX) return;
  uint8_t s;
  if (rxPop(s)) {
    gLastDacVal = s;
  } else {
    if      (gLastDacVal > 128) gLastDacVal--;
    else if (gLastDacVal < 128) gLastDacVal++;
  }
  dac_output_voltage(DAC_CHANNEL_1, gLastDacVal);
}

// ─────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n========================================");
  Serial.println("  WALKIE-TALKIE DEVICE #2");
  Serial.println("========================================");

  pinMode(PTT_GPIO, INPUT_PULLUP);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

  dac_output_enable(DAC_CHANNEL_1);
  dac_output_voltage(DAC_CHANNEL_1, 128);

  playSpeakerTest();

  // ── WiFi: STA mode, FORCED to channel 1 ──
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  String myMac = WiFi.macAddress();
  Serial.println("----------------------------------------");
  Serial.print  ("[MAC] My  MAC  : ");
  Serial.println(myMac);
  Serial.print  ("[MAC] Expected : ");
  Serial.println(MY_EXPECTED_MAC);
  Serial.print  ("[MAC] Peer MAC : ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", PEER_MAC[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.println();

  if (myMac != MY_EXPECTED_MAC) {
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("[ERROR] MAC MISMATCH — you may have the");
    Serial.println("        wrong .ino file on this board!");
    Serial.println("        Swap DEVICE1 and DEVICE2 files.");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  } else {
    Serial.println("[MAC] MAC address verified OK");
  }
  Serial.println("----------------------------------------");

  // ── ESP-NOW ──
  esp_err_t initErr = esp_now_init();
  if (initErr != ESP_OK) {
    Serial.printf("[ERROR] esp_now_init failed: 0x%X — halting\n", initErr);
    while (true) delay(1000);
  }
  Serial.println("[ESP-NOW] Initialized OK");

  esp_now_register_recv_cb(onDataReceived);
  esp_now_register_send_cb(onDataSent);

  // ── Register Peer ──
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.channel = 1;
  peer.encrypt = false;
  peer.ifidx   = WIFI_IF_STA;

  esp_now_del_peer(PEER_MAC);  // clear stale registration

  esp_err_t peerErr = esp_now_add_peer(&peer);
  if (peerErr == ESP_OK) {
    Serial.println("[ESP-NOW] Peer registered OK ✓");
  } else {
    Serial.printf("[ERROR] esp_now_add_peer failed: 0x%X\n", peerErr);
    Serial.println("  Common causes:");
    Serial.println("  0x3066 = peer MAC invalid");
    Serial.println("  0x3069 = peer already exists (harmless)");
    Serial.println("  0x3065 = ESP-NOW not initialized");
  }

  // ── Timers ──
  esp_timer_create_args_t txArgs = {};
  txArgs.callback        = txTimerISR;
  txArgs.dispatch_method = ESP_TIMER_TASK;
  txArgs.name            = "tx_adc";
  esp_timer_create(&txArgs, &gTxTimer);

  esp_timer_create_args_t dacArgs = {};
  dacArgs.callback        = dacTimerISR;
  dacArgs.dispatch_method = ESP_TIMER_TASK;
  dacArgs.name            = "rx_dac";
  esp_timer_create(&dacArgs, &gDacTimer);
  esp_timer_start_periodic(gDacTimer, SAMPLE_INTERVAL_US);

  Serial.println("\n[INFO] Ready — hold PTT (GPIO14) to transmit");
  Serial.println("[INFO] Will auto-calibrate mic on first PTT press\n");
}

// ─────────────────────────────────────────────
//  MAIN LOOP
// ─────────────────────────────────────────────
static bool gPttWasPressed = false;

void loop() {
  bool pttPressed = (digitalRead(PTT_GPIO) == LOW);
  if (pttPressed != gPttWasPressed) {
    delay(8);
    pttPressed = (digitalRead(PTT_GPIO) == LOW);
  }

  if (pttPressed && !gPttWasPressed) {
    if (!gBaselineSet) calibrateMicBaseline();

    Serial.println("\n[PTT] >>>>>> TRANSMITTING >>>>>>");
    gMode         = MODE_TX;
    gTxSampleIdx  = 0;
    gTxPrevSample = 2048;
    gHpPrev = gHpOutput = 0.0f;
    gAgcGain = 4.0f; gAgcPeak = 0.0f;
    esp_timer_start_periodic(gTxTimer, SAMPLE_INTERVAL_US);
    gPttWasPressed = true;
  }

  if (!pttPressed && gPttWasPressed) {
    esp_timer_stop(gTxTimer);
    gMode          = MODE_RX;
    gRxWriteIdx    = 0;
    gRxReadIdx     = 0;
    gRxPrevDecoded = 2048;
    gRxExpectedSeq = gTxSeq;
    gPttWasPressed = false;
    Serial.println("[PTT] <<<<<< LISTENING <<<<<<\n");
  }

  // ── Diagnostics every 2 seconds ──
  static uint32_t lastDiag = 0;
  if (millis() - lastDiag >= 2000) {
    lastDiag = millis();
    int      adc     = adc1_get_raw(ADC1_CHANNEL_6);
    uint32_t bufUsed = (gRxWriteIdx + RX_RING_SIZE - gRxReadIdx) % RX_RING_SIZE;

    if (gMode == MODE_RX) {
      Serial.printf("[DIAG RX] ADC=%4d | buf=%4uB | pkts_rx=%u | dropped=%u\n",
                    adc, bufUsed, gPktsReceived, gRxDropped);
    } else {
      Serial.printf("[DIAG TX] ADC=%4d (baseline=%d) | sent=%u | fails=%u | AGC=%.1f\n",
                    adc, gMicBaseline, gPktsSent, gSendFails, gAgcGain);
      if (gSendFails > 0 && gPktsSent == 0)
        Serial.println("  [WARN] ALL sends failing — peer MAC wrong or not on same channel");
    }
  }
}
