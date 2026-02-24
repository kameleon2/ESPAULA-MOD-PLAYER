/* * PROJECT ESPAULA - VERSION 34.8 (FULL RESTORATION)
 * * HARDWARE CONFIGURATION:
 * - I2S: BCK:4, WS:5, DIN:6
 * - SD CARD: CS:10
 * - UI: BTN_NEXT:15 (Input Pullup)
 * - DIAGNOSTICS: RGB LED:48 (WS2812)
 * * DIAGNOSTIC COLORS (Tick 0): 
 * - 4CH: Red | 8CH: Blue | 16CH: Yellow | 32CH: Green
 */

#include "Arduino.h"
#include "driver/i2s.h"
#include "SD.h"
#include "SPI.h"
#include <vector>

// --- Hardver Pin definíciók ---
#define I2S_BCK       4
#define I2S_WS        5
#define I2S_DIN       6
#define SD_CS         10
#define BTN_NEXT      15
#define RGB_LED_PIN   48 

// --- Audio paraméterek ---
const uint32_t AMIGA_CLOCK = 3546895;
const int SAMPLE_RATE = 44100;

// Amiga periódus táblázat az Arpeggio-hoz
const uint16_t arp_periods[] = {
  1712,1616,1524,1440,1356,1280,1208,1140,1076,1016,960,906,
  856,808,762,720,678,640,604,570,538,508,480,453,
  428,404,381,360,339,320,302,285,269,254,240,226,
  214,202,190,180,170,160,151,143,135,127,120,113
};

struct Sample {
  int8_t* data = NULL;
  uint32_t length = 0;
  uint32_t loopStart = 0;
  uint32_t loopLength = 0;
  uint8_t volume = 64;
};

struct Channel {
  uint32_t pos_frac = 0;
  uint32_t step_fixed = 0;
  uint32_t volume = 0;      // Belső hangerő (0-16384) a rampinghez
  uint32_t target_vol = 0;  // Cél hangerő (0-64)
  uint16_t base_period = 0;   
  uint16_t curr_period = 0;   
  uint16_t portamento_dest = 0; 
  uint8_t porta_speed = 0;
  uint16_t effect_data = 0; // (EffectType << 8) | EffectParameter
  bool active = false;
  Sample* smp = NULL;
};

// --- Globális változók ---
uint8_t* fullModBuffer = NULL; 
uint8_t* pMats = NULL;
Sample insts[32]; 
Channel chs[32]; // 32 csatornás támogatás
uint32_t lut_step[1024];
std::vector<String> playlist;
int currentTrack = 0;

uint8_t sOrd[128], sLen = 0, cOrd = 0, cRow = 0, cTick = 0;
int bpm = 125, speed = 6; 
uint8_t numChannels = 4;
int8_t mixShift = 1; 

volatile bool isPlaying = false;
volatile bool isChangingTrack = false; 

// --- Segédfüggvények ---

void generateLUT() {
  for (int p = 0; p < 1024; p++) {
    lut_step[p] = (p < 113) ? 0 : (uint32_t)(((double)AMIGA_CLOCK / (double)p / (double)SAMPLE_RATE) * 65536.0);
  }
}

uint16_t getArp(uint16_t base, uint8_t add) {
  for(int i=0; i<48; i++) {
    if(arp_periods[i] <= base) {
      int idx = i - add;
      if (idx < 0) idx = 0;
      if (idx > 47) idx = 47;
      return arp_periods[idx];
    }
  }
  return base;
}

void freeModMemory() {
  Serial.println("\n[SYSTEM] Fading out and clearing memory...");
  isChangingTrack = true;
  
  // Szoftveres halkítás a pattanás ellen
  for(int i=0; i<32; i++) chs[i].target_vol = 0;
  vTaskDelay(pdMS_TO_TICKS(150)); 

  isPlaying = false;
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_stop(I2S_NUM_0);
  
  if (pMats) { free(pMats); pMats = NULL; }
  if (fullModBuffer) { free(fullModBuffer); fullModBuffer = NULL; }
  for(int i=0; i<32; i++) { insts[i].data = NULL; insts[i].length = 0; }
  Serial.println("[SYSTEM] Memory clean.");
}

bool loadMod(String fileName) {
  freeModMemory();
  Serial.print("[LOADER] Opening: "); Serial.println(fileName);
  
  File f = SD.open(fileName);
  if (!f) { 
    Serial.println("[ERROR] File not found!");
    isChangingTrack = false; 
    return false; 
  }
  
  uint32_t fSize = f.size();
  fullModBuffer = (uint8_t*)ps_malloc(fSize + 4096);
  if (!fullModBuffer) { 
    Serial.println("[ERROR] PSRAM allocation failed!");
    f.close(); 
    isChangingTrack = false; 
    return false; 
  }
  
  f.read(fullModBuffer, fSize);
  f.close();

  // Magic Tag ellenőrzése
  char tag[5]; memcpy(tag, fullModBuffer + 1080, 4); tag[4] = 0;
  String tagS = String(tag);
  
  if (tagS == "8CHN" || tagS == "OCTA") numChannels = 8;
  else if (tagS == "16CH") numChannels = 16;
  else if (tagS == "32CH") numChannels = 32;
  else if (tagS.endsWith("CH")) numChannels = tagS.substring(0,2).toInt();
  else numChannels = 4;

  Serial.printf("[INFO] Channels: %d, Tag: %s\n", numChannels, tagS.c_str());

  // Dinamikus hangerő korrekció (mixShift)
  if (numChannels <= 4) mixShift = 1;      
  else if (numChannels <= 8) mixShift = 1; 
  else if (numChannels <= 16) mixShift = 0;
  else mixShift = -1;

  // Hangszer adatok beolvasása
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
  
  uint32_t patternDataSize = (maxPat + 1) * (numChannels * 256);
  pMats = (uint8_t*)ps_malloc(patternDataSize); 
  if (!pMats) { 
    free(fullModBuffer); 
    isChangingTrack = false; 
    return false; 
  }
  memcpy(pMats, fullModBuffer + 1084, patternDataSize);

  // Minták címeinek kiszámítása
  uint32_t currentOffset = 1084 + patternDataSize;
  for (int i = 1; i <= 31; i++) {
    if (insts[i].length > 0) {
      insts[i].data = (int8_t*)(fullModBuffer + currentOffset);
      currentOffset += insts[i].length;
    }
  }

  // Regiszterek alaphelyzetbe állítása
  bpm = 125; speed = 6; cOrd = 0; cRow = 0; cTick = 0;
  for(int i=0; i<32; i++) { 
    chs[i].active = false; chs[i].volume = 0; chs[i].target_vol = 0; 
    chs[i].pos_frac = 0; chs[i].step_fixed = 0; chs[i].effect_data = 0;
    chs[i].porta_speed = 0; chs[i].portamento_dest = 0;
  }
  
  i2s_start(I2S_NUM_0);
  isChangingTrack = false;
  isPlaying = true;
  return true;
}

void tickLogic() {
  if (!isPlaying || !pMats || isChangingTrack) return;

  if (cTick == 0) {
    // --- DIAGNOSZTIKAI SZÍNKÓDOK ---
    if (numChannels <= 4)      neopixelWrite(RGB_LED_PIN, 120, 0, 0);   // 4CH: PIROS
    else if (numChannels <= 8) neopixelWrite(RGB_LED_PIN, 0, 0, 120);   // 8CH: KÉK
    else if (numChannels <= 16)neopixelWrite(RGB_LED_PIN, 100, 100, 0); // 16CH: SÁRGA
    else                       neopixelWrite(RGB_LED_PIN, 0, 120, 0);   // 32CH: ZÖLD

    uint32_t patOffset = sOrd[cOrd] * (numChannels * 256);
    uint8_t* row = pMats + patOffset + (cRow * (numChannels * 4));

    for (int i = 0; i < numChannels; i++) {
      uint8_t* cell = row + (i * 4);
      uint16_t per = ((uint16_t)(cell[0] & 0x0F) << 8) | cell[1];
      uint8_t inst = (cell[0] & 0xF0) | (cell[2] >> 4);
      uint8_t et = cell[2] & 0x0F;
      uint8_t ed = cell[3];
      
      if (inst > 0 && inst <= 31) { 
        chs[i].smp = &insts[inst]; 
        chs[i].target_vol = insts[inst].volume; 
      }
      
      if (per > 0) { 
        if (et == 0x3) {
          chs[i].portamento_dest = per;
        } else {
          chs[i].base_period = per;
          chs[i].curr_period = per;
          chs[i].step_fixed = (per < 1024) ? lut_step[per] : 0;
          chs[i].pos_frac = 0; 
          chs[i].active = true;
        }
      }
      
      chs[i].effect_data = (et << 8) | ed;

      // Azonnali effektek (Tick 0)
      if (et == 0xC) chs[i].target_vol = (ed > 64) ? 64 : ed;
      if (et == 0xF) { if (ed > 0) { if (ed < 32) speed = ed; else bpm = ed; } }
      if (et == 0xB) { cOrd = (ed < sLen) ? ed : 0; cRow = 63; } 
      if (et == 0xD) { cOrd++; cRow = (ed < 64) ? ed - 1 : -1; if(cOrd >= sLen) cOrd = 0; }
      if (et == 0x3 && ed > 0) chs[i].porta_speed = ed;
      
      // E-szériás speciális effektek
      if (et == 0xE) {
        uint8_t ex = ed >> 4, ey = ed & 0x0F;
        if (ex == 0x1) { // Fine Slide Up
          chs[i].curr_period -= ey; 
          if(chs[i].curr_period < 113) chs[i].curr_period = 113;
          chs[i].step_fixed = lut_step[chs[i].curr_period];
        } 
        else if (ex == 0x2) { // Fine Slide Down
          chs[i].curr_period += ey; 
          if(chs[i].curr_period > 856) chs[i].curr_period = 856;
          chs[i].step_fixed = lut_step[chs[i].curr_period];
        }
        else if (ex == 0xA) { chs[i].target_vol = (chs[i].target_vol + ey > 64) ? 64 : chs[i].target_vol + ey; }
        else if (ex == 0xB) { chs[i].target_vol = (chs[i].target_vol > ey) ? chs[i].target_vol - ey : 0; }
      }
    }
  } else {
    // Tick 1-től kikapcsoljuk a LED-et, hogy villogjon
    if (cTick == 1) neopixelWrite(RGB_LED_PIN, 0, 0, 0); 

    // Folyamatos effektek (Tick 1-től)
    for (int i = 0; i < numChannels; i++) {
      if (!chs[i].active) continue;
      uint8_t et = chs[i].effect_data >> 8;
      uint8_t ed = chs[i].effect_data & 0xFF;
      
      if (et == 0x0 && ed > 0) { // Arpeggio
        uint8_t m = cTick % 3;
        uint16_t ap = chs[i].base_period;
        if (m == 1) ap = getArp(chs[i].base_period, ed >> 4); 
        else if (m == 2) ap = getArp(chs[i].base_period, ed & 0x0F);
        if (ap >= 113 && ap < 1024) chs[i].step_fixed = lut_step[ap];
      }
      else if (et == 0x1) { // Portamento Up
        chs[i].curr_period -= ed; 
        if(chs[i].curr_period < 113) chs[i].curr_period = 113;
        chs[i].step_fixed = lut_step[chs[i].curr_period];
      }
      else if (et == 0x2) { // Portamento Down
        chs[i].curr_period += ed; 
        if(chs[i].curr_period > 856) chs[i].curr_period = 856;
        chs[i].step_fixed = lut_step[chs[i].curr_period];
      }
      else if (et == 0x3 && chs[i].porta_speed > 0) { // Tone Portamento
        if (chs[i].curr_period < chs[i].portamento_dest) {
          chs[i].curr_period += chs[i].porta_speed;
          if (chs[i].curr_period > chs[i].portamento_dest) chs[i].curr_period = chs[i].portamento_dest;
        } else if (chs[i].curr_period > chs[i].portamento_dest) {
          chs[i].curr_period -= chs[i].porta_speed;
          if (chs[i].curr_period < chs[i].portamento_dest) chs[i].curr_period = chs[i].portamento_dest;
        }
        chs[i].step_fixed = lut_step[chs[i].curr_period];
      }
      else if (et == 0xA) { // Volume Slide
        uint8_t hi = ed >> 4, lo = ed & 0x0F;
        if (hi > 0) chs[i].target_vol = (chs[i].target_vol + hi > 64) ? 64 : chs[i].target_vol + hi;
        else if (lo > 0) chs[i].target_vol = (chs[i].target_vol > lo) ? chs[i].target_vol - lo : 0;
      }
      else if (et == 0xE && (ed >> 4) == 0x9) { // Retrig Note
        if (cTick % (ed & 0x0F) == 0) chs[i].pos_frac = 0;
      }
    }
  }

  // Időzítés léptetése
  if (++cTick >= speed) { 
    cTick = 0; 
    if (++cRow >= 64) { 
      cRow = 0; 
      if (++cOrd >= sLen) cOrd = 0; 
    } 
  }
}

void audioTask(void *pv) {
  const int numSamples = 128;
  int16_t outB[numSamples * 2];
  size_t bw;
  uint32_t samplesSinceLastTick = 0;

  while (true) {
    if (!isPlaying && !isChangingTrack) { 
      vTaskDelay(pdMS_TO_TICKS(10)); 
      continue; 
    }

    uint32_t samplesPerTick = (SAMPLE_RATE * 5) / (bpm * 2); 

    for (int s = 0; s < numSamples; s++) {
      if (samplesSinceLastTick >= samplesPerTick) { 
        tickLogic(); 
        samplesSinceLastTick = 0; 
        samplesPerTick = (SAMPLE_RATE * 5) / (bpm * 2); 
      }
      samplesSinceLastTick++;

      int64_t mixL = 0, mixR = 0;
      for (int i = 0; i < numChannels; i++) {
        // Hangerő rampa (V33.9 algoritmus)
        uint32_t tvf = chs[i].target_vol << 8;
        if (chs[i].volume < tvf) chs[i].volume += 16; 
        else if (chs[i].volume > tvf) chs[i].volume -= 16;
        
        if (chs[i].active && chs[i].smp && chs[i].smp->data) {
          int32_t s_val = (int32_t)chs[i].smp->data[chs[i].pos_frac >> 16] * (int32_t)(chs[i].volume >> 8); 
          
          // Amiga stílusú sztereó panoráma (0,3,4,7... bal | 1,2,5,6... jobb)
          if ((i % 4) == 0 || (i % 4) == 3) mixL += s_val; 
          else mixR += s_val;
          
          chs[i].pos_frac += chs[i].step_fixed;
          
          // Precíz Loop kezelés
          if (chs[i].smp->loopLength > 2) {
            uint32_t lEnd = chs[i].smp->loopStart + chs[i].smp->loopLength;
            if ((chs[i].pos_frac >> 16) >= lEnd) {
              chs[i].pos_frac -= (chs[i].smp->loopLength << 16);
            }
          } else if ((chs[i].pos_frac >> 16) >= chs[i].smp->length) {
            chs[i].active = false;
          }
        }
      }
      
      // Gain korrekció és 16-bit korlátozás
      int32_t finalL = (mixShift >= 0) ? (int32_t)(mixL << mixShift) : (int32_t)(mixL >> (-mixShift));
      int32_t finalR = (mixShift >= 0) ? (int32_t)(mixR << mixShift) : (int32_t)(mixR >> (-mixShift));

      outB[s*2]   = (int16_t)constrain(finalL, -31500, 31500); 
      outB[s*2+1] = (int16_t)constrain(finalR, -31500, 31500);
    }
    i2s_write(I2S_NUM_0, outB, sizeof(outB), &bw, portMAX_DELAY);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  delay(1000);
  Serial.println("\n--- ESPAULA V34.8 START ---");
  
  generateLUT();

  // I2S inicializálás (12 DMA puffer a stabilitásért)
  i2s_config_t ic = { 
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), 
    .sample_rate = SAMPLE_RATE, 
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, 
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, 
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, 
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, 
    .dma_buf_count = 12, 
    .dma_buf_len = 512, 
    .use_apll = false 
  };
  i2s_pin_config_t pc = { .bck_io_num = 4, .ws_io_num = 5, .data_out_num = 6, .data_in_num = -1 };
  i2s_driver_install(I2S_NUM_0, &ic, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pc);

  // SD kártya és Playlist
  if (SD.begin(SD_CS)) {
    File root = SD.open("/"); 
    File file = root.openNextFile();
    while (file) { 
      String n = file.name(); 
      if (n.endsWith(".mod") || n.endsWith(".mo")) playlist.push_back("/" + n); 
      file = root.openNextFile(); 
    }
    root.close();
    Serial.printf("[SD] Found %d tracks.\n", playlist.size());
  }

  if (!playlist.empty()) loadMod(playlist[currentTrack]);
  
  // Audio Task indítása a Core 1-en
  xTaskCreatePinnedToCore(audioTask, "AudioTask", 8192, NULL, 24, NULL, 1);
}

void loop() {
  // Következő szám gomb kezelése
  if (digitalRead(BTN_NEXT) == LOW) { 
    vTaskDelay(pdMS_TO_TICKS(50));
    if (digitalRead(BTN_NEXT) == LOW) {
      currentTrack = (currentTrack + 1) % playlist.size(); 
      loadMod(playlist[currentTrack]); 
      while(digitalRead(BTN_NEXT) == LOW) vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}