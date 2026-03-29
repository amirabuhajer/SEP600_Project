/*
 * ESP32 Sensor Dashboard + AI Prediction + Digital Twin
 * ======================================================
 * Receives LDR + LED data from FRDM-K64F via UART
 * Hosts a web dashboard with:
 *   - AI-based light prediction (linear regression)
 *   - Digital Twin LED control (bidirectional)
 *
 * DIGITAL TWIN:
 *   Dashboard toggle → ESP32 → UART → FRDM → physical LED
 *   Physical LDR/LED state → FRDM → UART → ESP32 → Dashboard
 *
 * ┌───────────────────────────────────────────────────┐
 * │  WIFI MODE: Uncomment ONE section below           │
 * │  Option 1: HOME_WIFI     (default, active now)    │
 * │  Option 2: EDUROAM       (for school)             │
 * └───────────────────────────────────────────────────┘
 */

#include <WebServer.h>
#include <WiFi.h>

// ╔══════════════════════════════════════════════════════╗
// ║  >>> OPTION 1: HOME WIFI (uncomment this block) <<< ║
// ╚══════════════════════════════════════════════════════╝
#define USE_HOME_WIFI
#define HOME_SSID "Abu Hajer"
#define HOME_PASSWORD "Moon2023"

// ╔══════════════════════════════════════════════════════╗
// ║  >>> OPTION 2: EDUROAM (uncomment this block)   <<< ║
// ╚══════════════════════════════════════════════════════╝
// #define USE_EDUROAM
// #include <HTTPClient.h>
// #include "esp_wifi.h"
// #include "esp_wpa2.h"
// #define EDUROAM_SSID     "eduroam"
// #define EAP_IDENTITY     ""
// #define EAP_USERNAME     "aabu-hajer@myseneca.ca"
// #define EAP_PASSWORD     "Ameer2003!"
// #define EAP_ANON_ID      ""

// ========= UART =========
#define RXD2 16
#define TXD2 17
#define UART_BAUD 9600

WebServer server(80);

// ========= SENSOR DATA =========
volatile uint32_t latestLDR = 0;
volatile float latestTemp = 0.0;
volatile bool ledOn = false;
String currentMode = "AUTO";
unsigned long lastUpdate = 0;

#define LOG_SIZE 50
struct LogEntry {
  uint32_t ldr;
  float temp;
  bool led;
  String mode;
  unsigned long timestamp;
};

LogEntry logBuffer[LOG_SIZE];
int logHead = 0;
int logCount = 0;

// ========= AI PREDICTION =========
#define LED_THRESHOLD 1000
#define TEMP_HIGH_THRESHOLD 27.0f
#define TEMP_LOW_THRESHOLD  23.0f
#define SAMPLE_INTERVAL 1.0f

struct PredictionResult {
  // Light prediction
  float slope;
  float predict5s;
  float predict10s;
  float predict30s;
  float confidence;
  String trend;
  String recommendation;
  bool ledWillChange;
  float timeToThreshold;

  // Temperature prediction
  float tempSlope;
  float tempPredict5s;
  float tempPredict10s;
  float tempPredict30s;
  float tempConfidence;
  String tempTrend;
  String tempRecommendation;
  bool tempWillExceed;
  float timeToTempThreshold;
};

PredictionResult prediction;

void runPrediction() {
  if (logCount < 5) {
    prediction.trend = "COLLECTING";
    prediction.recommendation = "Gathering data... need at least 5 readings";
    prediction.confidence = 0;
    prediction.slope = 0;
    prediction.predict5s = latestLDR;
    prediction.predict10s = latestLDR;
    prediction.predict30s = latestLDR;
    prediction.ledWillChange = false;
    prediction.timeToThreshold = -1;

    prediction.tempTrend = "COLLECTING";
    prediction.tempRecommendation = "Gathering temperature data...";
    prediction.tempConfidence = 0;
    prediction.tempSlope = 0;
    prediction.tempPredict5s = latestTemp;
    prediction.tempPredict10s = latestTemp;
    prediction.tempPredict30s = latestTemp;
    prediction.tempWillExceed = false;
    prediction.timeToTempThreshold = -1;
    return;
  }

  int n = min(logCount, 20);
  float nf = (float)n;

  // ── Light Linear Regression ──
  float sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
  for (int i = 0; i < n; i++) {
    int idx = (logHead - 1 - i + LOG_SIZE) % LOG_SIZE;
    float x = (float)(n - 1 - i);
    float y = (float)logBuffer[idx].ldr;
    sumX += x; sumY += y; sumXY += x * y; sumX2 += x * x;
  }

  float denom = (nf * sumX2 - sumX * sumX);
  float m = 0, b = 0;
  if (denom != 0) {
    m = (nf * sumXY - sumX * sumY) / denom;
    b = (sumY - m * sumX) / nf;
  }

  prediction.slope = m;
  prediction.predict5s  = constrain(m * ((nf - 1) + 5)  + b, 0, 4095);
  prediction.predict10s = constrain(m * ((nf - 1) + 10) + b, 0, 4095);
  prediction.predict30s = constrain(m * ((nf - 1) + 30) + b, 0, 4095);

  float meanY = sumY / nf;
  float ssTot = 0, ssRes = 0;
  for (int i = 0; i < n; i++) {
    int idx = (logHead - 1 - i + LOG_SIZE) % LOG_SIZE;
    float x = (float)(n - 1 - i);
    float y = (float)logBuffer[idx].ldr;
    float yFit = m * x + b;
    ssTot += (y - meanY) * (y - meanY);
    ssRes += (y - yFit)  * (y - yFit);
  }
  prediction.confidence = (ssTot > 0) ? constrain(1.0f - ssRes / ssTot, 0, 1) : 0;

  float absSlope = abs(m);
  if (absSlope < 5)      prediction.trend = "STABLE";
  else if (m > 0)        prediction.trend = "RISING";
  else                   prediction.trend = "FALLING";

  prediction.ledWillChange = false;
  prediction.timeToThreshold = -1;
  float currentVal = (float)latestLDR;
  if (abs(m) > 0.5) {
    float timeToThresh = ((float)LED_THRESHOLD - currentVal) / m;
    if (timeToThresh > 0 && timeToThresh < 120) {
      prediction.timeToThreshold = timeToThresh;
      prediction.ledWillChange = true;
    }
  }

  if (prediction.trend == "STABLE") {
    prediction.recommendation = (latestLDR < LED_THRESHOLD) ?
      "Light levels stable and LOW. LED will remain ON." :
      "Light levels stable and NORMAL. No action needed.";
  } else if (prediction.trend == "FALLING") {
    if (prediction.ledWillChange && latestLDR >= LED_THRESHOLD) {
      prediction.recommendation = "WARNING: Light DECREASING. LED will turn ON in ~" + String((int)prediction.timeToThreshold) + "s.";
    } else if (latestLDR < LED_THRESHOLD) {
      prediction.recommendation = "Light continues to DECREASE. LED is ON.";
    } else {
      prediction.recommendation = "Light is DECREASING but still above threshold.";
    }
  } else {
    if (prediction.ledWillChange && latestLDR < LED_THRESHOLD) {
      prediction.recommendation = "Light INCREASING. LED will turn OFF in ~" + String((int)prediction.timeToThreshold) + "s.";
    } else if (latestLDR >= LED_THRESHOLD) {
      prediction.recommendation = "Light levels RISING. Conditions are good.";
    } else {
      prediction.recommendation = "Light is RISING but still below threshold. LED remains ON.";
    }
  }

  // ── Temperature Linear Regression ──
  float tsX = 0, tsY = 0, tsXY = 0, tsX2 = 0;
  for (int i = 0; i < n; i++) {
    int idx = (logHead - 1 - i + LOG_SIZE) % LOG_SIZE;
    float x = (float)(n - 1 - i);
    float y = logBuffer[idx].temp;
    tsX += x; tsY += y; tsXY += x * y; tsX2 += x * x;
  }

  float tDenom = (nf * tsX2 - tsX * tsX);
  float tm = 0, tb = 0;
  if (tDenom != 0) {
    tm = (nf * tsXY - tsX * tsY) / tDenom;
    tb = (tsY - tm * tsX) / nf;
  }

  prediction.tempSlope = tm;
  prediction.tempPredict5s  = constrain(tm * ((nf - 1) + 5)  + tb, -40, 125);
  prediction.tempPredict10s = constrain(tm * ((nf - 1) + 10) + tb, -40, 125);
  prediction.tempPredict30s = constrain(tm * ((nf - 1) + 30) + tb, -40, 125);

  float tMeanY = tsY / nf;
  float tSsTot = 0, tSsRes = 0;
  for (int i = 0; i < n; i++) {
    int idx = (logHead - 1 - i + LOG_SIZE) % LOG_SIZE;
    float x = (float)(n - 1 - i);
    float y = logBuffer[idx].temp;
    float yFit = tm * x + tb;
    tSsTot += (y - tMeanY) * (y - tMeanY);
    tSsRes += (y - yFit)  * (y - yFit);
  }
  prediction.tempConfidence = (tSsTot > 0) ? constrain(1.0f - tSsRes / tSsTot, 0, 1) : 0;

  float tAbsSlope = abs(tm);
  if (tAbsSlope < 0.05)   prediction.tempTrend = "STABLE";
  else if (tm > 0)        prediction.tempTrend = "HEATING";
  else                    prediction.tempTrend = "COOLING";

  prediction.tempWillExceed = false;
  prediction.timeToTempThreshold = -1;
  if (abs(tm) > 0.01) {
    if (latestTemp < TEMP_HIGH_THRESHOLD && tm > 0) {
      float timeToHigh = (TEMP_HIGH_THRESHOLD - latestTemp) / tm;
      if (timeToHigh > 0 && timeToHigh < 300) {
        prediction.timeToTempThreshold = timeToHigh;
        prediction.tempWillExceed = true;
      }
    } else if (latestTemp > TEMP_LOW_THRESHOLD && tm < 0) {
      float timeToLow = (TEMP_LOW_THRESHOLD - latestTemp) / tm;
      if (timeToLow > 0 && timeToLow < 300) {
        prediction.timeToTempThreshold = timeToLow;
        prediction.tempWillExceed = true;
      }
    }
  }

  if (prediction.tempTrend == "STABLE") {
    if (latestTemp >= TEMP_HIGH_THRESHOLD) {
      prediction.tempRecommendation = "Temperature stable at " + String(latestTemp, 1) + "C. OPEN ventilation to cool down.";
    } else if (latestTemp <= TEMP_LOW_THRESHOLD) {
      prediction.tempRecommendation = "Temperature stable at " + String(latestTemp, 1) + "C. CLOSE ventilation to retain heat.";
    } else {
      prediction.tempRecommendation = "Temperature stable at " + String(latestTemp, 1) + "C. Conditions are comfortable.";
    }
  } else if (prediction.tempTrend == "HEATING") {
    if (prediction.tempWillExceed && latestTemp < TEMP_HIGH_THRESHOLD) {
      prediction.tempRecommendation = "WARNING: Temperature RISING. Will reach 27C in ~" + String((int)prediction.timeToTempThreshold) + "s. Prepare to OPEN ventilation!";
    } else if (latestTemp >= TEMP_HIGH_THRESHOLD) {
      prediction.tempRecommendation = "Temperature HIGH (" + String(latestTemp, 1) + "C) and RISING! OPEN ventilation now!";
    } else {
      prediction.tempRecommendation = "Temperature rising gradually. Currently " + String(latestTemp, 1) + "C.";
    }
  } else {
    if (prediction.tempWillExceed && latestTemp > TEMP_LOW_THRESHOLD) {
      prediction.tempRecommendation = "Temperature COOLING. Will reach 23C in ~" + String((int)prediction.timeToTempThreshold) + "s. Prepare to CLOSE ventilation.";
    } else if (latestTemp <= TEMP_LOW_THRESHOLD) {
      prediction.tempRecommendation = "Temperature LOW (" + String(latestTemp, 1) + "C). CLOSE ventilation to retain heat.";
    } else {
      prediction.tempRecommendation = "Temperature cooling. Currently " + String(latestTemp, 1) + "C. Comfortable range.";
    }
  }
}

// ================= UART PARSER =================
// Expects: LDR:1234,TEMP:25.50,LED:ON,MODE:AUTO\r\n
void parseUARTData() {
  static String lineBuffer = "";

  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '\n' || c == '\r') {
      if (lineBuffer.length() > 0) {

        int ldrIdx = lineBuffer.indexOf("LDR:");
        int tempIdx = lineBuffer.indexOf("TEMP:");
        int ledIdx = lineBuffer.indexOf("LED:");
        int modeIdx = lineBuffer.indexOf("MODE:");

        if (ldrIdx >= 0 && ledIdx >= 0) {
          // LDR
          String ldrStr =
              lineBuffer.substring(ldrIdx + 4, lineBuffer.indexOf(",", ldrIdx));
          ldrStr.trim();
          latestLDR = ldrStr.toInt();

          // Temperature
          if (tempIdx >= 0) {
            String tempStr = lineBuffer.substring(
                tempIdx + 5, lineBuffer.indexOf(",", tempIdx));
            tempStr.trim();
            latestTemp = tempStr.toFloat();
          }

          // LED
          int ledEnd = (modeIdx >= 0) ? modeIdx - 1 : lineBuffer.length();
          String ledStr = lineBuffer.substring(ledIdx + 4, ledEnd);
          ledStr.trim();
          if (ledStr.endsWith(","))
            ledStr = ledStr.substring(0, ledStr.length() - 1);
          ledOn = ledStr.equalsIgnoreCase("ON");

          // Mode
          if (modeIdx >= 0) {
            String modeStr = lineBuffer.substring(modeIdx + 5);
            modeStr.trim();
            currentMode = modeStr;
          }

          lastUpdate = millis();

          logBuffer[logHead] = {latestLDR, latestTemp, ledOn, currentMode,
                                lastUpdate};
          logHead = (logHead + 1) % LOG_SIZE;
          if (logCount < LOG_SIZE)
            logCount++;

          runPrediction();

          Serial.printf("[UART] LDR=%d TEMP=%.2f LED=%s MODE=%s\n", latestLDR,
                        latestTemp, ledOn ? "ON" : "OFF", currentMode.c_str());
        }

        lineBuffer = "";
      }
    } else {
      lineBuffer += c;
    }
  }
}

// ================= DIGITAL TWIN ENDPOINTS =================

// POST /toggle?state=on  or  /toggle?state=off
void handleToggle() {
  String state = server.arg("state");

  if (state == "on") {
    Serial2.flush();                  // wait for pending TX to finish
    Serial2.print("CMD:LED_ON\n");    // send command
    delay(50);                        // small gap so FRDM can drain its FIFO
    Serial2.print("CMD:LED_ON\n");    // send again for reliability
    ledOn = true;
    currentMode = "MANUAL";
    Serial.println("[TWIN] Sent CMD:LED_ON to FRDM");
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"LED_ON\"}");
  } else if (state == "off") {
    Serial2.flush();
    Serial2.print("CMD:LED_OFF\n");
    delay(50);
    Serial2.print("CMD:LED_OFF\n");
    ledOn = false;
    currentMode = "MANUAL";
    Serial.println("[TWIN] Sent CMD:LED_OFF to FRDM");
    server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"LED_OFF\"}");
  } else {
    server.send(400, "application/json",
                "{\"ok\":false,\"error\":\"invalid state\"}");
  }
}

// POST /auto  -> switch back to auto mode
void handleAuto() {
  Serial2.flush();
  Serial2.print("CMD:AUTO\n");
  delay(50);
  Serial2.print("CMD:AUTO\n");
  currentMode = "AUTO";
  Serial.println("[TWIN] Sent CMD:AUTO to FRDM");
  server.send(200, "application/json", "{\"ok\":true,\"cmd\":\"AUTO\"}");
}

// ================= JSON API =================
void handleData() {
  String json = "{";
  json += "\"ldr\":" + String(latestLDR) + ",";
  json += "\"temp\":" + String(latestTemp, 2) + ",";
  json += "\"led\":" + String(ledOn ? "true" : "false") + ",";
  json += "\"mode\":\"" + currentMode + "\",";
  json += "\"uptime\":" + String(millis() / 1000) + ",";
  json += "\"lastUpdate\":" + String(lastUpdate / 1000) + ",";

  // AI prediction - Light
  json += "\"prediction\":{";
  json += "\"slope\":" + String(prediction.slope, 2) + ",";
  json += "\"trend\":\"" + prediction.trend + "\",";
  json += "\"confidence\":" + String(prediction.confidence, 3) + ",";
  json += "\"predict5s\":" + String((int)prediction.predict5s) + ",";
  json += "\"predict10s\":" + String((int)prediction.predict10s) + ",";
  json += "\"predict30s\":" + String((int)prediction.predict30s) + ",";
  json += "\"ledWillChange\":" +
          String(prediction.ledWillChange ? "true" : "false") + ",";
  json += "\"timeToThreshold\":" + String(prediction.timeToThreshold, 1) + ",";
  json += "\"recommendation\":\"" + prediction.recommendation + "\"";
  json += "},";

  // AI prediction - Temperature
  json += "\"tempPrediction\":{";
  json += "\"slope\":" + String(prediction.tempSlope, 4) + ",";
  json += "\"trend\":\"" + prediction.tempTrend + "\",";
  json += "\"confidence\":" + String(prediction.tempConfidence, 3) + ",";
  json += "\"predict5s\":" + String(prediction.tempPredict5s, 2) + ",";
  json += "\"predict10s\":" + String(prediction.tempPredict10s, 2) + ",";
  json += "\"predict30s\":" + String(prediction.tempPredict30s, 2) + ",";
  json += "\"willExceed\":" + String(prediction.tempWillExceed ? "true" : "false") + ",";
  json += "\"timeToThreshold\":" + String(prediction.timeToTempThreshold, 1) + ",";
  json += "\"recommendation\":\"" + prediction.tempRecommendation + "\"";
  json += "},";

  // Log
  json += "\"log\":[";
  for (int i = 0; i < logCount; i++) {
    int idx = (logHead - 1 - i + LOG_SIZE) % LOG_SIZE;
    if (i > 0)
      json += ",";
    json += "{\"ldr\":" + String(logBuffer[idx].ldr);
    json += ",\"temp\":" + String(logBuffer[idx].temp, 2);
    json += ",\"led\":" + String(logBuffer[idx].led ? "true" : "false");
    json += ",\"mode\":\"" + logBuffer[idx].mode + "\"";
    json += ",\"ts\":" + String(logBuffer[idx].timestamp / 1000) + "}";
  }

  json += "]}";
  server.send(200, "application/json", json);
}

// ================= DASHBOARD HTML =================
const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 AI + Digital Twin Dashboard</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap');
  * { margin:0; padding:0; box-sizing:border-box; }
  body {
    font-family: 'Inter', sans-serif;
    background: #0a0e1a;
    color: #e0e6f0;
    min-height: 100vh;
    overflow-x: hidden;
  }
  body::before {
    content: '';
    position: fixed;
    top: -50%; left: -50%;
    width: 200%; height: 200%;
    background: radial-gradient(circle at 20% 50%, rgba(29,78,216,0.08) 0%, transparent 50%),
                radial-gradient(circle at 80% 20%, rgba(139,92,246,0.06) 0%, transparent 50%),
                radial-gradient(circle at 50% 80%, rgba(6,182,212,0.05) 0%, transparent 50%);
    animation: bgShift 20s ease-in-out infinite alternate;
    z-index: 0;
  }
  @keyframes bgShift {
    0%   { transform: translate(0,0) rotate(0deg); }
    100% { transform: translate(-5%,3%) rotate(3deg); }
  }

  .container {
    position: relative; z-index: 1;
    max-width: 1000px;
    margin: 0 auto;
    padding: 24px 20px 40px;
  }

  .header { text-align: center; margin-bottom: 28px; }
  .header h1 {
    font-size: 26px;
    font-weight: 700;
    background: linear-gradient(135deg, #60a5fa, #a78bfa, #34d399);
    -webkit-background-clip: text;
    -webkit-text-fill-color: transparent;
  }
  .header .subtitle { font-size: 13px; color: #64748b; margin-top: 4px; }
  .status-badge { margin-top: 8px; font-size: 12px; color: #34d399; }

  /* Cards */
  .cards {
    display: grid;
    grid-template-columns: 1fr 1fr 1fr;
    gap: 16px;
    margin-bottom: 20px;
  }
  .card {
    background: rgba(15,23,42,0.7);
    backdrop-filter: blur(16px);
    border: 1px solid rgba(255,255,255,0.06);
    border-radius: 16px;
    padding: 24px;
    text-align: center;
    transition: transform 0.2s;
  }
  .card:hover { transform: translateY(-3px); }
  .card-label { font-size: 11px; text-transform: uppercase; letter-spacing: 1px; color: #94a3b8; margin-bottom: 10px; }
  .card-value { font-size: 44px; font-weight: 700; background: linear-gradient(135deg, #60a5fa, #34d399); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
  .card-unit { font-size: 12px; color: #64748b; margin-top: 6px; }

  /* LED indicator */
  .led-indicator { width: 60px; height: 60px; border-radius: 50%; margin: 8px auto; transition: all 0.4s ease; }
  .led-on { background: radial-gradient(circle, #4ade80, #22c55e); box-shadow: 0 0 25px rgba(74,222,128,0.6); animation: glow 1.5s ease-in-out infinite alternate; }
  .led-off { background: radial-gradient(circle, #374151, #1f2937); }
  @keyframes glow { from { box-shadow: 0 0 20px rgba(74,222,128,0.5); } to { box-shadow: 0 0 35px rgba(74,222,128,0.8); } }
  .led-text { font-size: 16px; font-weight: 600; margin-top: 8px; }

  /* ── DIGITAL TWIN SECTION ── */
  .twin-section {
    background: rgba(15,23,42,0.7);
    backdrop-filter: blur(16px);
    border: 1px solid rgba(6,182,212,0.25);
    border-radius: 16px;
    padding: 24px;
    margin-bottom: 20px;
  }
  .twin-section h3 {
    font-size: 15px; font-weight: 600; margin-bottom: 16px;
    display: flex; align-items: center; gap: 8px;
  }
  .twin-badge {
    background: linear-gradient(135deg, #06b6d4, #0891b2);
    color: white; font-size: 10px; padding: 2px 8px; border-radius: 8px; font-weight: 700;
  }

  .twin-controls {
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 24px;
    flex-wrap: wrap;
  }

  .twin-led-display {
    text-align: center;
  }
  .twin-led-circle {
    width: 100px; height: 100px; border-radius: 50%;
    margin: 0 auto 10px;
    transition: all 0.4s ease;
    border: 3px solid rgba(255,255,255,0.1);
  }
  .twin-led-on {
    background: radial-gradient(circle, #4ade80, #16a34a);
    box-shadow: 0 0 40px rgba(74,222,128,0.5), 0 0 80px rgba(74,222,128,0.2);
    border-color: rgba(74,222,128,0.3);
  }
  .twin-led-off {
    background: radial-gradient(circle, #374151, #111827);
    box-shadow: inset 0 2px 10px rgba(0,0,0,0.5);
  }
  .twin-led-label { font-size: 14px; font-weight: 600; margin-top: 8px; }

  .twin-panel { text-align: center; }

  /* Mode badge */
  .mode-badge {
    display: inline-block; padding: 4px 14px; border-radius: 20px;
    font-size: 12px; font-weight: 600; margin-bottom: 16px;
  }
  .mode-auto { background: rgba(52,211,153,0.12); color: #34d399; border: 1px solid rgba(52,211,153,0.25); }
  .mode-manual { background: rgba(6,182,212,0.12); color: #22d3ee; border: 1px solid rgba(6,182,212,0.25); }

  /* Toggle switch */
  .toggle-container { margin: 16px 0; }
  .toggle-label { font-size: 11px; color: #94a3b8; margin-bottom: 8px; text-transform: uppercase; letter-spacing: 1px; }
  .toggle-switch {
    position: relative; width: 72px; height: 36px;
    background: #1e293b; border-radius: 18px;
    cursor: pointer; transition: background 0.3s;
    border: 2px solid rgba(255,255,255,0.08);
    display: inline-block;
  }
  .toggle-switch.active { background: #065f46; border-color: rgba(52,211,153,0.3); }
  .toggle-knob {
    position: absolute; top: 3px; left: 3px;
    width: 26px; height: 26px; border-radius: 50%;
    background: #94a3b8; transition: all 0.3s;
  }
  .toggle-switch.active .toggle-knob { left: 39px; background: #34d399; box-shadow: 0 0 10px rgba(52,211,153,0.5); }

  /* Auto button */
  .auto-btn {
    margin-top: 12px; padding: 8px 20px; border-radius: 10px;
    background: rgba(52,211,153,0.1); border: 1px solid rgba(52,211,153,0.25);
    color: #34d399; font-size: 12px; font-weight: 600;
    cursor: pointer; transition: all 0.2s;
  }
  .auto-btn:hover { background: rgba(52,211,153,0.2); }
  .auto-btn.active-mode { background: rgba(52,211,153,0.2); box-shadow: 0 0 12px rgba(52,211,153,0.2); }

  .arrow-icon { font-size: 28px; color: #22d3ee; animation: pulse 2s ease-in-out infinite; }
  @keyframes pulse { 0%,100% { opacity:0.4; } 50% { opacity:1; } }

  /* AI Section */
  .ai-section {
    background: rgba(15,23,42,0.7);
    backdrop-filter: blur(16px);
    border: 1px solid rgba(139,92,246,0.2);
    border-radius: 16px;
    padding: 24px;
    margin-bottom: 20px;
  }
  .ai-section h3 { font-size: 15px; font-weight: 600; margin-bottom: 16px; display: flex; align-items: center; gap: 8px; }
  .ai-badge { background: linear-gradient(135deg, #a78bfa, #818cf8); color: white; font-size: 10px; padding: 2px 8px; border-radius: 8px; font-weight: 700; }
  .ai-grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 12px; margin-bottom: 16px; }
  .ai-card { background: rgba(139,92,246,0.06); border: 1px solid rgba(139,92,246,0.12); border-radius: 12px; padding: 14px; text-align: center; }
  .ai-card .label { font-size: 10px; text-transform: uppercase; letter-spacing: 1px; color: #a78bfa; margin-bottom: 6px; }
  .ai-card .value { font-size: 26px; font-weight: 700; color: #c4b5fd; }
  .ai-card .sub { font-size: 11px; color: #64748b; margin-top: 4px; }

  .trend-row { display: flex; gap: 10px; margin-bottom: 14px; flex-wrap: wrap; }
  .trend-chip { display: inline-flex; align-items: center; gap: 5px; padding: 5px 12px; border-radius: 20px; font-size: 12px; font-weight: 600; }
  .trend-rising  { background: rgba(251,191,36,0.12); color: #fbbf24; border: 1px solid rgba(251,191,36,0.2); }
  .trend-falling { background: rgba(59,130,246,0.12); color: #60a5fa; border: 1px solid rgba(59,130,246,0.2); }
  .trend-stable  { background: rgba(52,211,153,0.12); color: #34d399; border: 1px solid rgba(52,211,153,0.2); }
  .trend-collecting { background: rgba(148,163,184,0.12); color: #94a3b8; border: 1px solid rgba(148,163,184,0.2); }

  .confidence-bar { height: 5px; background: rgba(255,255,255,0.06); border-radius: 3px; overflow: hidden; margin-bottom: 14px; }
  .confidence-fill { height: 100%; background: linear-gradient(90deg, #a78bfa, #818cf8); border-radius: 3px; transition: width 0.5s ease; }

  .recommendation-box {
    background: rgba(139,92,246,0.08); border-left: 3px solid #a78bfa;
    padding: 12px 14px; border-radius: 0 10px 10px 0; font-size: 13px; line-height: 1.5; color: #cbd5e1;
  }
  .recommendation-box .rec-label { font-size: 10px; text-transform: uppercase; letter-spacing: 1px; color: #a78bfa; margin-bottom: 5px; font-weight: 600; }
  .warning-box { border-left-color: #f59e0b; }
  .warning-box .rec-label { color: #f59e0b; }

  /* Log table */
  .log-section {
    background: rgba(15,23,42,0.7);
    backdrop-filter: blur(16px);
    border: 1px solid rgba(255,255,255,0.06);
    border-radius: 16px;
    padding: 24px;
  }
  .log-section h3 { font-size: 15px; font-weight: 600; margin-bottom: 16px; color: #94a3b8; }
  table { width: 100%; border-collapse: collapse; }
  th { padding: 8px 12px; text-align: left; font-size: 10px; text-transform: uppercase; letter-spacing: 1px; color: #64748b; border-bottom: 1px solid rgba(255,255,255,0.06); }
  td { padding: 8px 12px; font-size: 13px; color: #cbd5e1; }
  tr:nth-child(even) { background: rgba(255,255,255,0.02); }
  .led-badge { display: inline-block; padding: 2px 10px; border-radius: 12px; font-size: 11px; font-weight: 600; }
  .badge-on { background: rgba(74,222,128,0.15); color: #4ade80; }
  .badge-off { background: rgba(100,116,139,0.15); color: #94a3b8; }
  .badge-auto { background: rgba(52,211,153,0.1); color: #34d399; }
  .badge-manual { background: rgba(6,182,212,0.1); color: #22d3ee; }

  @media (max-width: 600px) {
    .cards { grid-template-columns: 1fr; }
    .ai-grid { grid-template-columns: 1fr; }
    .twin-controls { flex-direction: column; }
  }
</style>
</head>
<body>

<div class="container">

  <div class="header">
    <h1>FRDM-K64F Sensor Dashboard</h1>
    <div class="subtitle">AI Light Prediction &bull; Digital Twin LED Control</div>
    <div class="status-badge" id="status">Connecting...</div>
  </div>

  <!-- SENSOR CARDS -->
  <div class="cards">
    <div class="card">
      <div class="card-label">LDR Light Level</div>
      <div class="card-value" id="ldrVal">--</div>
      <div class="card-unit">ADC (0 – 4095) &bull; Threshold: 1000</div>
    </div>
    <div class="card">
      <div class="card-label">Temperature</div>
      <div class="card-value" id="tempVal" style="background:linear-gradient(135deg,#f97316,#ef4444);-webkit-background-clip:text;-webkit-text-fill-color:transparent;">--</div>
      <div class="card-unit">&deg;C &bull; LM35DZ Sensor</div>
    </div>
    <div class="card">
      <div class="card-label">LED Status</div>
      <div class="led-indicator led-off" id="ledDot"></div>
      <div class="led-text" id="ledText">OFF</div>
    </div>
  </div>

  <!-- DIGITAL TWIN -->
  <div class="twin-section">
    <h3>
      LED Digital Twin
      <span class="twin-badge">BIDIRECTIONAL</span>
      <span class="mode-badge mode-auto" id="modeBadge">AUTO</span>
    </h3>

    <div class="twin-controls">

      <div class="twin-panel">
        <div class="toggle-label">Dashboard Control</div>
        <div class="toggle-container">
          <div class="toggle-switch" id="twinToggle" onclick="toggleLED()">
            <div class="toggle-knob"></div>
          </div>
        </div>
        <div style="font-size:11px;color:#64748b;margin-top:4px;" id="toggleHint">Click to take manual control</div>
        <button class="auto-btn" id="autoBtn" onclick="switchAuto()">Switch to AUTO Mode</button>
      </div>

      <div class="arrow-icon">⇄</div>

      <div class="twin-led-display">
        <div class="toggle-label">Physical LED Mirror</div>
        <div class="twin-led-circle twin-led-off" id="twinLed"></div>
        <div class="twin-led-label" id="twinLedLabel">OFF</div>
      </div>

    </div>
  </div>

  <!-- AI PREDICTION - LIGHT -->
  <div class="ai-section">
    <h3>
      Light Prediction
      <span class="ai-badge">AI MODULE</span>
    </h3>

    <div class="trend-row">
      <div class="trend-chip trend-stable" id="trendChip">
        <span id="trendIcon">&bull;</span> <span id="trendText">STABLE</span>
      </div>
      <div class="trend-chip" id="changeChip" style="display:none">
        &#9889; <span id="changeText"></span>
      </div>
    </div>

    <div style="font-size:11px; color:#64748b; margin-bottom:5px;">
      Model confidence: <span id="confVal">0%</span>
    </div>
    <div class="confidence-bar">
      <div class="confidence-fill" id="confBar" style="width:0%"></div>
    </div>

    <div class="ai-grid">
      <div class="ai-card"><div class="label">In 5 seconds</div><div class="value" id="pred5">--</div><div class="sub">predicted LDR</div></div>
      <div class="ai-card"><div class="label">In 10 seconds</div><div class="value" id="pred10">--</div><div class="sub">predicted LDR</div></div>
      <div class="ai-card"><div class="label">In 30 seconds</div><div class="value" id="pred30">--</div><div class="sub">predicted LDR</div></div>
    </div>

    <div class="recommendation-box" id="recBox">
      <div class="rec-label">AI Recommendation</div>
      <div id="recText">Gathering data...</div>
    </div>
  </div>

  <!-- AI PREDICTION - TEMPERATURE -->
  <div class="ai-section" style="border-color:rgba(249,115,22,0.25);">
    <h3>
      Temperature Prediction
      <span class="ai-badge" style="background:linear-gradient(135deg,#f97316,#ef4444);">AI MODULE</span>
    </h3>

    <div class="trend-row">
      <div class="trend-chip trend-stable" id="tempTrendChip">
        <span id="tempTrendIcon">&bull;</span> <span id="tempTrendText">STABLE</span>
      </div>
      <div class="trend-chip" id="tempChangeChip" style="display:none">
        &#9889; <span id="tempChangeText"></span>
      </div>
    </div>

    <div style="font-size:11px; color:#64748b; margin-bottom:5px;">
      Model confidence: <span id="tempConfVal">0%</span>
    </div>
    <div class="confidence-bar">
      <div class="confidence-fill" id="tempConfBar" style="width:0%;background:linear-gradient(90deg,#f97316,#ef4444);"></div>
    </div>

    <div class="ai-grid">
      <div class="ai-card" style="border-color:rgba(249,115,22,0.15);background:rgba(249,115,22,0.04);">
        <div class="label" style="color:#f97316;">In 5 seconds</div>
        <div class="value" id="tempPred5" style="color:#fb923c;">--</div>
        <div class="sub">&deg;C predicted</div>
      </div>
      <div class="ai-card" style="border-color:rgba(249,115,22,0.15);background:rgba(249,115,22,0.04);">
        <div class="label" style="color:#f97316;">In 10 seconds</div>
        <div class="value" id="tempPred10" style="color:#fb923c;">--</div>
        <div class="sub">&deg;C predicted</div>
      </div>
      <div class="ai-card" style="border-color:rgba(249,115,22,0.15);background:rgba(249,115,22,0.04);">
        <div class="label" style="color:#f97316;">In 30 seconds</div>
        <div class="value" id="tempPred30" style="color:#fb923c;">--</div>
        <div class="sub">&deg;C predicted</div>
      </div>
    </div>

    <div class="recommendation-box" id="tempRecBox" style="border-left-color:#f97316;background:rgba(249,115,22,0.06);">
      <div class="rec-label" style="color:#f97316;">Ventilation Recommendation</div>
      <div id="tempRecText">Gathering temperature data...</div>
    </div>
  </div>

  <!-- LOG -->
  <div class="log-section">
    <h3>Sensor Log</h3>
    <table>
      <thead><tr><th>#</th><th>LDR</th><th>Temp</th><th>LED</th><th>Mode</th><th>Time(s)</th></tr></thead>
      <tbody id="logBody"></tbody>
    </table>
  </div>

</div>

<script>
let currentLedState = false;
let ignoreNextFetch = false;
let ignoreTimer = null;

function toggleLED() {
  const newState = !currentLedState;
  fetch('/toggle?state=' + (newState ? 'on' : 'off'), { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.ok) {
        currentLedState = newState;
        updateToggleUI(newState);
        updateTwinLed(newState);
        // Prevent fetchData from overwriting for 3 seconds
        ignoreNextFetch = true;
        clearTimeout(ignoreTimer);
        ignoreTimer = setTimeout(() => { ignoreNextFetch = false; }, 3000);
      }
    });
}

function switchAuto() {
  fetch('/auto', { method: 'POST' })
    .then(r => r.json())
    .then(d => {
      if (d.ok) {
        document.getElementById('toggleHint').textContent = 'Switched to AUTO mode';
        document.getElementById('modeBadge').className = 'mode-badge mode-auto';
        document.getElementById('modeBadge').textContent = 'AUTO';
        // Allow fetch to update after brief delay
        ignoreNextFetch = true;
        clearTimeout(ignoreTimer);
        ignoreTimer = setTimeout(() => { ignoreNextFetch = false; }, 3000);
      }
    });
}

function updateToggleUI(on) {
  const tog = document.getElementById('twinToggle');
  if (on) { tog.classList.add('active'); } else { tog.classList.remove('active'); }
}

function updateTwinLed(on) {
  const twinLed = document.getElementById('twinLed');
  const twinLabel = document.getElementById('twinLedLabel');
  if (on) { twinLed.className = 'twin-led-circle twin-led-on'; twinLabel.textContent = 'ON'; twinLabel.style.color = '#4ade80'; }
  else    { twinLed.className = 'twin-led-circle twin-led-off'; twinLabel.textContent = 'OFF'; twinLabel.style.color = '#94a3b8'; }
  document.getElementById('modeBadge').className = 'mode-badge mode-manual';
  document.getElementById('modeBadge').textContent = 'MANUAL';
  document.getElementById('toggleHint').textContent = 'You are controlling the LED';
}

function fetchData() {
  fetch('/data')
    .then(r => r.json())
    .then(d => {

      document.getElementById('ldrVal').textContent = d.ldr;
      document.getElementById('tempVal').textContent = d.temp.toFixed(2);

      // LED card
      const led = document.getElementById('ledDot');
      const ledTxt = document.getElementById('ledText');
      if (d.led) { led.className = 'led-indicator led-on'; ledTxt.textContent = 'ON'; ledTxt.style.color = '#4ade80'; }
      else       { led.className = 'led-indicator led-off'; ledTxt.textContent = 'OFF'; ledTxt.style.color = '#94a3b8'; }

      // Status
      document.getElementById('status').textContent = 'Online \u2022 Uptime: ' + d.uptime + 's \u2022 Last: ' + d.lastUpdate + 's';
      document.getElementById('status').style.color = '#34d399';

      // Digital Twin (skip if user just toggled)
      if (!ignoreNextFetch) {
        currentLedState = d.led;
        updateToggleUI(d.led);
      }

      if (!ignoreNextFetch) {
        const twinLed = document.getElementById('twinLed');
        const twinLabel = document.getElementById('twinLedLabel');
        if (d.led) { twinLed.className = 'twin-led-circle twin-led-on'; twinLabel.textContent = 'ON'; twinLabel.style.color = '#4ade80'; }
        else       { twinLed.className = 'twin-led-circle twin-led-off'; twinLabel.textContent = 'OFF'; twinLabel.style.color = '#94a3b8'; }
      }

      // Mode badge
      if (!ignoreNextFetch) {
        const modeBadge = document.getElementById('modeBadge');
        const autoBtn = document.getElementById('autoBtn');
        const hint = document.getElementById('toggleHint');
        if (d.mode === 'AUTO') {
          modeBadge.className = 'mode-badge mode-auto'; modeBadge.textContent = 'AUTO';
          autoBtn.classList.add('active-mode');
          hint.textContent = 'Click toggle to take manual control';
        } else {
          modeBadge.className = 'mode-badge mode-manual'; modeBadge.textContent = 'MANUAL';
          autoBtn.classList.remove('active-mode');
          hint.textContent = 'You are controlling the LED';
        }
      }

      // AI
      if (d.prediction) {
        const p = d.prediction;
        document.getElementById('pred5').textContent = p.predict5s;
        document.getElementById('pred10').textContent = p.predict10s;
        document.getElementById('pred30').textContent = p.predict30s;
        const confPct = (p.confidence * 100).toFixed(1);
        document.getElementById('confVal').textContent = confPct + '%';
        document.getElementById('confBar').style.width = confPct + '%';

        const tc = document.getElementById('trendChip');
        const tt = document.getElementById('trendText');
        const ti = document.getElementById('trendIcon');
        tt.textContent = p.trend;
        if (p.trend === 'RISING')       { tc.className = 'trend-chip trend-rising';     ti.textContent = '\u2191'; }
        else if (p.trend === 'FALLING') { tc.className = 'trend-chip trend-falling';    ti.textContent = '\u2193'; }
        else if (p.trend === 'STABLE')  { tc.className = 'trend-chip trend-stable';     ti.textContent = '\u25CF'; }
        else                            { tc.className = 'trend-chip trend-collecting'; ti.textContent = '\u25CC'; }

        const cc = document.getElementById('changeChip');
        const ct = document.getElementById('changeText');
        if (p.ledWillChange && p.timeToThreshold > 0) {
          cc.style.display = 'inline-flex'; cc.style.background = 'rgba(248,113,113,0.12)'; cc.style.color = '#f87171'; cc.style.border = '1px solid rgba(248,113,113,0.2)';
          ct.textContent = 'LED change in ~' + Math.round(p.timeToThreshold) + 's';
        } else { cc.style.display = 'none'; }

        document.getElementById('recText').textContent = p.recommendation;
        document.getElementById('recBox').className = p.recommendation.includes('WARNING') ? 'recommendation-box warning-box' : 'recommendation-box';
      }

      // AI - Temperature
      if (d.tempPrediction) {
        const tp = d.tempPrediction;
        document.getElementById('tempPred5').textContent = parseFloat(tp.predict5s).toFixed(1);
        document.getElementById('tempPred10').textContent = parseFloat(tp.predict10s).toFixed(1);
        document.getElementById('tempPred30').textContent = parseFloat(tp.predict30s).toFixed(1);
        const tConfPct = (tp.confidence * 100).toFixed(1);
        document.getElementById('tempConfVal').textContent = tConfPct + '%';
        document.getElementById('tempConfBar').style.width = tConfPct + '%';

        const ttc = document.getElementById('tempTrendChip');
        const ttt = document.getElementById('tempTrendText');
        const tti = document.getElementById('tempTrendIcon');
        ttt.textContent = tp.trend;
        if (tp.trend === 'HEATING')       { ttc.className = 'trend-chip trend-rising';     tti.textContent = '\u2191'; }
        else if (tp.trend === 'COOLING')  { ttc.className = 'trend-chip trend-falling';    tti.textContent = '\u2193'; }
        else if (tp.trend === 'STABLE')   { ttc.className = 'trend-chip trend-stable';     tti.textContent = '\u25CF'; }
        else                              { ttc.className = 'trend-chip trend-collecting'; tti.textContent = '\u25CC'; }

        const tcc = document.getElementById('tempChangeChip');
        const tct = document.getElementById('tempChangeText');
        if (tp.willExceed && tp.timeToThreshold > 0) {
          tcc.style.display = 'inline-flex'; tcc.style.background = 'rgba(248,113,113,0.12)'; tcc.style.color = '#f87171'; tcc.style.border = '1px solid rgba(248,113,113,0.2)';
          tct.textContent = 'High temp in ~' + Math.round(tp.timeToThreshold) + 's';
        } else { tcc.style.display = 'none'; }

        document.getElementById('tempRecText').textContent = tp.recommendation;
        document.getElementById('tempRecBox').className = tp.recommendation.includes('WARNING') ? 'recommendation-box warning-box' : 'recommendation-box';
        document.getElementById('tempRecBox').style.borderLeftColor = tp.recommendation.includes('WARNING') ? '#f59e0b' : '#f97316';
        document.getElementById('tempRecBox').style.background = tp.recommendation.includes('WARNING') ? 'rgba(245,158,11,0.06)' : 'rgba(249,115,22,0.06)';
      }

      // Log
      const tbody = document.getElementById('logBody');
      tbody.innerHTML = '';
      d.log.forEach((e, i) => {
        const tr = document.createElement('tr');
        const lbc = e.led ? 'badge-on' : 'badge-off';
        const lbt = e.led ? 'ON' : 'OFF';
        const mbc = e.mode === 'AUTO' ? 'badge-auto' : 'badge-manual';
        tr.innerHTML =
          '<td>'+(i+1)+'</td>' +
          '<td>'+e.ldr+'</td>' +
          '<td>'+e.temp.toFixed(2)+'</td>' +
          '<td><span class="led-badge '+lbc+'">'+lbt+'</span></td>' +
          '<td><span class="led-badge '+mbc+'">'+e.mode+'</span></td>' +
          '<td>'+e.ts+'</td>';
        tbody.appendChild(tr);
      });

    })
    .catch(() => {
      document.getElementById('status').textContent = 'Disconnected';
      document.getElementById('status').style.color = '#f87171';
    });
}

setInterval(fetchData, 2000);
fetchData();
</script>

</body>
</html>
)rawliteral";

// ================= ROOT =================
void handleRoot() { server.send(200, "text/html", DASHBOARD_HTML); }

// ================= WIFI =================
void connectWiFi() {
#ifdef USE_HOME_WIFI
  Serial.println("[WiFi] Connecting to home WiFi...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.begin(HOME_SSID, HOME_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
#endif

#ifdef USE_EDUROAM
  Serial.println("[WiFi] Connecting to eduroam...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_STA);
  esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_ANON_ID,
                                     strlen(EAP_ANON_ID));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_USERNAME,
                                     strlen(EAP_USERNAME));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD,
                                     strlen(EAP_PASSWORD));
  esp_wifi_sta_wpa2_ent_enable();
  WiFi.begin(EDUROAM_SSID);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
#endif

  Serial.println("\n[WiFi] Connected!");
  Serial.print("[WiFi] Dashboard IP: ");
  Serial.println(WiFi.localIP());
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Serial2.begin(UART_BAUD, SERIAL_8N1, RXD2, TXD2);

  connectWiFi();

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/toggle", HTTP_POST, handleToggle);
  server.on("/auto", HTTP_POST, handleAuto);
  server.begin();

  Serial.println("[Server] AI + Digital Twin Dashboard running!");
}

// ================= LOOP =================
void loop() {
  server.handleClient();
  parseUARTData();
}