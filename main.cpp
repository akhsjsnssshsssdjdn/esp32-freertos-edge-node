// Smart Environmental & Predictive Maintenance Edge Node
// ESP32 (dual core) + FreeRTOS + Arduino framework
//
// Core 0: sensorTask        - 1 Hz deterministic acquisition (DHT22 + ADC vibration)
// Core 1: telemetryTask     - queue consumer, shared-state update, JSON logging
//         displayTask       - I2C OLED rendering
//         profilerTask      - stack high-water marks, execution timing, core load
// ISRs:   fault line / reset button - low-latency fault latching and alarm reset

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <Wire.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <math.h>
#include <stdarg.h>

#include "config.h"
#include "faults.h"
#include "moving_average.h"
#include "profiler.h"
#include "types.h"

// ------------------------------------------------------------------ Globals
static DHT dht(PIN_DHT, DHT22);
static Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static bool g_oledOk = false;

static QueueHandle_t     g_sampleQueue;  // sensorTask -> telemetryTask
static SemaphoreHandle_t g_stateMutex;   // protects g_latest
static SemaphoreHandle_t g_serialMutex;  // keeps Serial output lines intact

static SensorSample g_latest;
static bool g_latestValid = false;
static volatile uint32_t g_queueDrops = 0;

static TaskHandle_t g_tasks[TASK_COUNT];
static const char* const kTaskNames[TASK_COUNT] = { "sensor", "telemetry", "display", "profiler" };
static const uint32_t kStackSizes[TASK_COUNT] = { STACK_SENSOR, STACK_TELEMETRY, STACK_DISPLAY, STACK_PROFILER };
static const BaseType_t kTaskCores[TASK_COUNT] = { CORE_ACQUISITION, CORE_APPLICATION, CORE_APPLICATION, CORE_APPLICATION };

// ------------------------------------------------------------------ Helpers
static void logf(const char* fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (xSemaphoreTake(g_serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.println(buf);
    xSemaphoreGive(g_serialMutex);
  }
}

// JSON has no NaN, so invalid readings are emitted as null.
static void fmtFloat(char* dst, size_t n, float v, int decimals) {
  if (isnan(v)) snprintf(dst, n, "null");
  else snprintf(dst, n, "%.*f", decimals, v);
}

static inline uint32_t elapsedUs(int64_t start) {
  return static_cast<uint32_t>(esp_timer_get_time() - start);
}

// ------------------------------------------------------------------ Core 0: acquisition
static void sensorTask(void*) {
  MovingAverage<ADC_FILTER_WINDOW> filter;
  float lastTemp = NAN, lastHum = NAN;
  uint8_t dhtFailStreak = 0;
  uint32_t seq = 0;
  int64_t prevStartUs = 0;

  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    // Absolute-time delay: the period does not drift with work duration.
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

    const int64_t t0 = esp_timer_get_time();
    if (prevStartUs) {
      int64_t jitter = (t0 - prevStartUs) - static_cast<int64_t>(SAMPLE_PERIOD_MS) * 1000;
      if (jitter < 0) jitter = -jitter;
      profileRecordJitter(static_cast<uint32_t>(jitter));
    }
    prevStartUs = t0;

    SensorSample s = {};
    s.seq = ++seq;
    s.timestamp_ms = millis();

    // --- DHT22 (temperature / humidity)
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (isnan(t) || isnan(h)) {
      s.dht_ok = false;
      if (dhtFailStreak < 255) dhtFailStreak++;
      t = lastTemp;  // hold last good value
      h = lastHum;
    } else {
      s.dht_ok = true;
      dhtFailStreak = 0;
      lastTemp = t;
      lastHum = h;
    }
    s.temp_c = t;
    s.humidity_pct = h;

    // --- Simulated vibration sensor: ADC burst through a moving-average filter
    uint16_t raw = 0, filtered = 0;
    for (size_t i = 0; i < ADC_BURST_SAMPLES; i++) {
      raw = static_cast<uint16_t>(analogRead(PIN_VIB_ADC));
      filtered = filter.push(raw);
    }
    s.vib_raw = raw;
    s.vib_filtered = filtered;

    // --- Fault evaluation (latched until manual reset)
    uint8_t newFaults = 0;
    if (!isnan(t) && t >= TEMP_FAULT_C) newFaults |= FAULT_TEMP;
    if (!isnan(h) && h >= HUM_FAULT_PCT) newFaults |= FAULT_HUMIDITY;
    if (filtered >= VIB_FAULT_RAW) newFaults |= FAULT_VIBRATION;
    if (dhtFailStreak >= DHT_FAIL_LIMIT) newFaults |= FAULT_SENSOR;
    faultsLatch(newFaults);
    s.faults = faultsGet();

    // --- Hand off to Core 1. Never block the deterministic task: drop and count.
    if (xQueueSend(g_sampleQueue, &s, 0) != pdTRUE) g_queueDrops++;

    profileRecord(PROF_SENSOR, elapsedUs(t0));
  }
}

// ------------------------------------------------------------------ Core 1: telemetry
static void telemetryTask(void*) {
  SensorSample s;
  for (;;) {
    if (xQueueReceive(g_sampleQueue, &s, portMAX_DELAY) != pdTRUE) continue;
    const int64_t t0 = esp_timer_get_time();

    if (xSemaphoreTake(g_stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      g_latest = s;
      g_latestValid = true;
      xSemaphoreGive(g_stateMutex);
    }

    char temp[12], hum[12], names[32];
    fmtFloat(temp, sizeof(temp), s.temp_c, 1);
    fmtFloat(hum, sizeof(hum), s.humidity_pct, 1);
    faultsDescribe(s.faults, names, sizeof(names));

    logf("{\"seq\":%lu,\"t_ms\":%lu,\"temp_c\":%s,\"hum_pct\":%s,\"vib_raw\":%u,\"vib_filt\":%u,"
         "\"faults\":%u,\"fault_names\":\"%s\",\"dht_ok\":%s}",
         (unsigned long)s.seq, (unsigned long)s.timestamp_ms, temp, hum,
         s.vib_raw, s.vib_filtered, s.faults, names, s.dht_ok ? "true" : "false");

    profileRecord(PROF_TELEMETRY, elapsedUs(t0));
  }
}

// ------------------------------------------------------------------ Core 1: display
static void renderDisplay(const SensorSample& s, bool valid) {
  char line[28];
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  oled.setCursor(0, 0);
  snprintf(line, sizeof(line), "EDGE NODE   up %lus", (unsigned long)(millis() / 1000));
  oled.print(line);

  if (!valid) {
    oled.setCursor(0, 20);
    oled.print("waiting for data...");
  } else {
    oled.setCursor(0, 12);
    if (isnan(s.temp_c)) snprintf(line, sizeof(line), "Temp  --");
    else snprintf(line, sizeof(line), "Temp  %.1f C", s.temp_c);
    oled.print(line);

    oled.setCursor(0, 22);
    if (isnan(s.humidity_pct)) snprintf(line, sizeof(line), "Hum   --");
    else snprintf(line, sizeof(line), "Hum   %.1f %%", s.humidity_pct);
    oled.print(line);

    oled.setCursor(0, 32);
    snprintf(line, sizeof(line), "Vib   %u (raw %u)", s.vib_filtered, s.vib_raw);
    oled.print(line);

    if (s.faults) {
      char names[32];
      faultsDescribe(s.faults, names, sizeof(names));
      oled.setCursor(0, 43);
      oled.print(names);
      oled.fillRect(0, 54, OLED_WIDTH, 10, SSD1306_WHITE);
      oled.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      oled.setCursor(10, 55);
      oled.print("** FAULT LATCHED **");
    } else {
      oled.setCursor(0, 55);
      oled.print("STATUS: OK");
    }
  }
  oled.display();
}

static void displayTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(DISPLAY_PERIOD_MS));
    if (!g_oledOk) continue;

    SensorSample copy = {};
    bool valid = false;
    if (xSemaphoreTake(g_stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      copy = g_latest;
      valid = g_latestValid;
      xSemaphoreGive(g_stateMutex);
    }
    // Show the live latch state even between 1 Hz samples (ISR may have changed it).
    if (valid) copy.faults = faultsGet();

    const int64_t t0 = esp_timer_get_time();
    renderDisplay(copy, valid);
    profileRecord(PROF_DISPLAY, elapsedUs(t0));
  }
}

// ------------------------------------------------------------------ Core 1: diagnostics profiler
static void profilerTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  int64_t windowStart = esp_timer_get_time();

  for (;;) {
    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PROFILE_PERIOD_MS));

    const int64_t now = esp_timer_get_time();
    const float windowUs = static_cast<float>(now - windowStart);
    windowStart = now;

    ProfStats st[PROF_COUNT];
    for (int i = 0; i < PROF_COUNT; i++) profilerSnapshotAndReset(static_cast<ProfId>(i), &st[i]);

    logf("[PROF] ---- window %lu ms ----", (unsigned long)(windowUs / 1000.0f));
    for (int i = 0; i < PROF_COUNT; i++) {
      logf("[PROF] %-9s n=%lu exec_us min/avg/max = %lu/%lu/%lu",
           profilerName(static_cast<ProfId>(i)), (unsigned long)st[i].count,
           (unsigned long)st[i].min_us, (unsigned long)st[i].avg_us, (unsigned long)st[i].max_us);
    }
    logf("[PROF] sensor wake-up jitter max = %lu us", (unsigned long)profilerJitterMaxAndReset());

    // Approximate busy time: wall-clock time spent inside each task's work section.
    const float core0 = 100.0f * st[PROF_SENSOR].total_us / windowUs;
    const float core1 = 100.0f * (st[PROF_TELEMETRY].total_us + st[PROF_DISPLAY].total_us) / windowUs;
    logf("[PROF] busy approx: core0 = %.2f %%  core1 = %.2f %%", core0, core1);

    for (int i = 0; i < TASK_COUNT; i++) {
      if (!g_tasks[i]) continue;
      uint32_t freeMin = uxTaskGetStackHighWaterMark(g_tasks[i]);  // bytes on ESP32
      uint32_t used = kStackSizes[i] > freeMin ? kStackSizes[i] - freeMin : 0;
      logf("[PROF] stack %-9s core%d size=%lu used=%lu (%lu%%) min_free=%lu",
           kTaskNames[i], (int)kTaskCores[i], (unsigned long)kStackSizes[i], (unsigned long)used,
           (unsigned long)(100UL * used / kStackSizes[i]), (unsigned long)freeMin);
    }

    uint32_t faultIsr = 0, resetIsr = 0;
    faultsIsrCounts(&faultIsr, &resetIsr);
    logf("[PROF] heap free=%lu min_free=%lu | queue_drops=%lu | isr fault=%lu reset=%lu",
         (unsigned long)esp_get_free_heap_size(), (unsigned long)esp_get_minimum_free_heap_size(),
         (unsigned long)g_queueDrops, (unsigned long)faultIsr, (unsigned long)resetIsr);
  }
}

// ------------------------------------------------------------------ Setup
void setup() {
  Serial.begin(115200);
  delay(200);

  g_stateMutex = xSemaphoreCreateMutex();
  g_serialMutex = xSemaphoreCreateMutex();
  g_sampleQueue = xQueueCreate(SAMPLE_QUEUE_LEN, sizeof(SensorSample));
  if (!g_stateMutex || !g_serialMutex || !g_sampleQueue) {
    Serial.println("FATAL: RTOS object allocation failed");
    while (true) delay(1000);
  }

  profilerInit();
  faultsInit();

  dht.begin();
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VIB_ADC, ADC_11db);

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  g_oledOk = oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (!g_oledOk) Serial.println("WARN: OLED not found, display task will idle");

  // Consumers first, producer last, so no sample is queued before its reader exists.
  xTaskCreatePinnedToCore(telemetryTask, kTaskNames[TASK_TELEMETRY], STACK_TELEMETRY, nullptr,
                          PRIO_TELEMETRY, &g_tasks[TASK_TELEMETRY], CORE_APPLICATION);
  xTaskCreatePinnedToCore(displayTask, kTaskNames[TASK_DISPLAY], STACK_DISPLAY, nullptr,
                          PRIO_DISPLAY, &g_tasks[TASK_DISPLAY], CORE_APPLICATION);
  xTaskCreatePinnedToCore(profilerTask, kTaskNames[TASK_PROFILER], STACK_PROFILER, nullptr,
                          PRIO_PROFILER, &g_tasks[TASK_PROFILER], CORE_APPLICATION);
  xTaskCreatePinnedToCore(sensorTask, kTaskNames[TASK_SENSOR], STACK_SENSOR, nullptr,
                          PRIO_SENSOR, &g_tasks[TASK_SENSOR], CORE_ACQUISITION);

  logf("Edge node started: acquisition on core %d, application on core %d",
       (int)CORE_ACQUISITION, (int)CORE_APPLICATION);
}

// All work happens in FreeRTOS tasks; the Arduino loop task is not needed.
void loop() {
  vTaskDelete(nullptr);
}
