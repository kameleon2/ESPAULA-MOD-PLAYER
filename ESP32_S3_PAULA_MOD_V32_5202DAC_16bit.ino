#include "Arduino.h"
#include "driver/i2s.h"
#include "SD.h"
#include "SPI.h"
#include <vector>

#define I2S_BCK       4
#define I2S_WS        5
#define I2S_DIN       6
#define SD_CS         10
#define BTN_NEXT      15
// A te pinjeid a LED-hez
#define LED_RED       41 
#define LED_GREEN     42 

const uint32_t AMIGA_CLOCK = 3546895;
const int SAMPLE_RATE = 44100;

// Arpeggio táblázat a Squawk mentéséhez
const uint16_t arp_periods[] = {
  1712,1616,1524,1440,1356,1280,1208,1140,1076,1016,960,906,
  856,808,762,720,678,640,604,570,538,508,480,453,
  428,404,381,360,339,320,302,285,269,254,240,226,
  214,202,190,180,170,160,151,143,135,127,120,113
};

struct Sample {
  int8_t* data = NULL;
  uint32_t length = 0, loopStart = 0, loopLength = 0;
  uint8_t volume = 64;
};

struct Channel {
  uint32_t pos_frac = 0, step_fixed = 0;
  uint32_t volume = 0, target_vol = 0; 
  uint16_t base_period = 0; // Megjegyezzük az alaphangot az Arpeggiohoz
  uint16_t effect_data = 0;  // Megjegyezzük az effektet a Tick-ekhez
  bool active = false;
  Sample* smp = NULL;
};

uint8_t* fullModBuffer = NULL; 
uint8_t* pMats = NULL;
Sample insts[32]; 
Channel chs[4];
uint32_t lut_step[1024];
std::vector<String> playlist;
int currentTrack = 0;

uint8_t sOrd[128], sLen = 0, cOrd = 0, cRow = 0, cTick = 0;
int bpm = 125, speed = 6; 
volatile bool isPlaying = false;
volatile uint32_t mixTimeUs = 0;

void generateLUT() {
  for (int p = 0; p < 1024; p++) {
    lut_step[p] = (p < 113) ? 0 : (uint32_t)(((double)AMIGA_CLOCK / (double)p / (double)SAMPLE_RATE) * 65536.0);
  }
}

// Segédfüggvény az Arpeggio hangmagasság kiszámításához
uint16_t getArp(uint16_t base, uint8_t add) {
  for(int i=0; i<48; i++) {
    if(arp_periods[i] <= base) {
      int idx = i - add;
      return arp_periods[idx < 0 ? 0 : (idx > 47 ? 47 : idx)];
    }
  }
  return base;
}

void freeModMemory() {
  isPlaying = false;
  vTaskDelay(20);
  if (pMats) { free(pMats); pMats = NULL; }
  // A mintákat mostantól nem külön malloc-oljuk, így nem fogy a RAM
  if (fullModBuffer) { free(fullModBuffer); fullModBuffer = NULL; }
}

bool loadMod(String fileName) {
  freeModMemory();
  digitalWrite(LED_RED, HIGH);

  File f = SD.open(fileName);
  if (!f) return false;
  
  uint32_t fSize = f.size();
  fullModBuffer = (uint8_t*)ps_malloc(fSize + 1024);
  if (!fullModBuffer) { f.close(); return false; }
  
  f.read(fullModBuffer, fSize);
  f.close();

  for (int i = 1; i <= 31; i++) {
    uint8_t* b = fullModBuffer + 20 + (i - 1) * 30;
    insts[i].length = ((b[22] << 8) | b[23]) * 2;
    insts[i].volume = (b[25] > 64) ? 64 : b[25];
    insts[i].loopStart = ((b[26] << 8) | b[27]) * 2;
    insts[i].loopLength = ((b[28] << 8) | b[29]) * 2;
  }
  
  sLen = fullModBuffer[950];
  memcpy(sOrd, fullModBuffer + 952, 128);
  
  uint8_t maxPat = 0;
  for (int i = 0; i < 128; i++) if (sOrd[i] > maxPat) maxPat = sOrd[i];
  
  uint32_t patSize = (maxPat + 1) * 1024;
  pMats = (uint8_t*)ps_malloc(patSize); 
  if (pMats) memcpy(pMats, fullModBuffer + 1084, patSize);

  // PSRAM FIX: A mintákat nem másoljuk sehova, közvetlenül a PSRAM-ban lévő bufferből olvassuk
  uint32_t currentOffset = 1084 + patSize;
  for (int i = 1; i <= 31; i++) {
    if (insts[i].length > 0) {
      insts[i].data = (int8_t*)(fullModBuffer + currentOffset);
      currentOffset += insts[i].length;
    }
  }

  bpm = 125; speed = 6; cOrd = 0; cRow = 0; cTick = 0;
  for(int i=0; i<4; i++) { chs[i].active = false; chs[i].volume = 0; }
  
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, HIGH); 
  isPlaying = true;
  return true;
}

void tickLogic() {
  if (!isPlaying || !pMats) return;
  
  if (cTick == 0) {
    uint8_t* row = pMats + (sOrd[cOrd] * 1024) + (cRow * 16);
    for (int i = 0; i < 4; i++) {
      uint8_t* cell = row + (i * 4);
      uint16_t per = ((uint16_t)(cell[0] & 0x0F) << 8) | cell[1];
      uint8_t inst = (cell[0] & 0xF0) | (cell[2] >> 4);
      
      if (inst > 0 && inst <= 31) { 
        chs[i].smp = &insts[inst]; 
        chs[i].target_vol = insts[inst].volume; 
      }
      if (per > 0) { 
        chs[i].base_period = per;
        chs[i].step_fixed = (per < 1024) ? lut_step[per] : 0; 
        chs[i].pos_frac = 0; 
        chs[i].active = true; 
      }
      
      uint8_t et = cell[2] & 0x0F, ed = cell[3];
      chs[i].effect_data = (et << 8) | ed; // Effekt mentése a Tick-ekhez

      if (et == 0xC) chs[i].target_vol = (ed > 64) ? 64 : ed;
      if (et == 0xF && ed > 0) { if (ed < 32) speed = ed; else bpm = ed; }
      if (et == 0xB) { cOrd = (ed < sLen) ? ed : 0; cRow = 63; } 
      if (et == 0xD) { cOrd++; cRow = (ed < 64) ? ed - 1 : -1; if(cOrd >= sLen) cOrd = 0; }
    }
  } else {
    // SQUAWK GYÓGYÍR: Arpeggio kezelés a 0. tick után is
    for (int i = 0; i < 4; i++) {
      uint8_t et = chs[i].effect_data >> 8;
      uint8_t ed = chs[i].effect_data & 0xFF;
      if (et == 0x0 && ed > 0 && chs[i].active) {
        uint8_t m = cTick % 3;
        uint16_t ap = chs[i].base_period;
        if (m == 1) ap = getArp(chs[i].base_period, ed >> 4);
        else if (m == 2) ap = getArp(chs[i].base_period, ed & 0x0F);
        if (ap > 0 && ap < 1024) chs[i].step_fixed = lut_step[ap];
      }
    }
  }

  if (++cTick >= speed) { 
    cTick = 0; 
    if (++cRow >= 64) { cRow = 0; if (++cOrd >= sLen) cOrd = 0; } 
  }
}

// Magas prioritású Audio Task (Core 1) - Maradt a te stabil verziód
void audioTask(void *pv) {
  const int numSamples = 128;
  int16_t outB[numSamples * 2];
  size_t bw;
  uint32_t samplesSinceLastTick = 0;

  while (true) {
    if (!isPlaying) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

    uint32_t startMix = micros();
    uint32_t samplesPerTick = (SAMPLE_RATE * 5) / (bpm * 2); 

    for (int s = 0; s < numSamples; s++) {
      if (samplesSinceLastTick >= samplesPerTick) {
        tickLogic();
        samplesSinceLastTick = 0;
        samplesPerTick = (SAMPLE_RATE * 5) / (bpm * 2);
      }
      samplesSinceLastTick++;

      int32_t mixL = 0;
      for (int i = 0; i < 4; i++) {
        if (chs[i].volume < (chs[i].target_vol << 8)) chs[i].volume += 128;
        else if (chs[i].volume > (chs[i].target_vol << 8)) chs[i].volume -= 128;
        
        if (chs[i].active && chs[i].smp && chs[i].smp->data) {
          uint32_t p_int = chs[i].pos_frac >> 16;
          mixL += ((int32_t)chs[i].smp->data[p_int] * (int32_t)chs[i].volume) >> 11;
          chs[i].pos_frac += chs[i].step_fixed;
          
          if (chs[i].smp->loopLength > 2) {
            uint32_t lEnd = chs[i].smp->loopStart + chs[i].smp->loopLength;
            if ((chs[i].pos_frac >> 16) >= lEnd) chs[i].pos_frac -= (chs[i].smp->loopLength << 16);
          } else if ((chs[i].pos_frac >> 16) >= chs[i].smp->length) chs[i].active = false;
        }
      }
      int16_t f = (int16_t)constrain(mixL, -28000, 28000);
      outB[s*2] = f; outB[s*2+1] = f;
    }
    mixTimeUs = micros() - startMix;
    i2s_write(I2S_NUM_0, outB, sizeof(outB), &bw, portMAX_DELAY);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  generateLUT();

  i2s_config_t ic = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8, .dma_buf_len = 512, .use_apll = false 
  };
  i2s_pin_config_t pc = { .bck_io_num = I2S_BCK, .ws_io_num = I2S_WS, .data_out_num = I2S_DIN, .data_in_num = -1 };
  i2s_driver_install(I2S_NUM_0, &ic, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pc);

  if (SD.begin(SD_CS)) {
    File root = SD.open("/");
    File file = root.openNextFile();
    while (file) {
      String n = file.name();
      if (n.endsWith(".mod") || n.endsWith(".MOD")) playlist.push_back("/" + n);
      file = root.openNextFile();
    }
  }

  if (!playlist.empty()) loadMod(playlist[currentTrack]);
  xTaskCreatePinnedToCore(audioTask, "AudioTask", 8192, NULL, 24, NULL, 1);
}

void loop() {
  if (digitalRead(BTN_NEXT) == LOW) {
    delay(200);
    currentTrack = (currentTrack + 1) % playlist.size();
    loadMod(playlist[currentTrack]);
  }
  vTaskDelay(10);
}