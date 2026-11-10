#include <WiFi.h>
#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ESP_I2S.h>
#include <esp_camera.h>
#include "mbedtls/base64.h"

#include "camera.h"
#include "wav_header.h"

// =================================================
// HARDWARE CONFIGURATION: DFRobot DFR1154 (AI Cam V1.1)
// =================================================
#define BUTTON_PIN 0
#define LED_PIN 3

#define SAMPLE_RATE 16000
#define CLOCK_PIN GPIO_NUM_13  // PDM Mic Clock (Shifted off camera data bus)
#define DATA_PIN  GPIO_NUM_21  // PDM Mic Data

#define MAX_RECORDING_SECONDS 4
#define PCM_WAV_HEADER_SIZE 44
#define MAX_AUDIO_BYTES (SAMPLE_RATE * 2 * MAX_RECORDING_SECONDS)
const char* ssid = "*******";
const char* password = "******";
const char* apiKey = "********";

const char* geminiURL = "https://generativelanguage.googleapis.com/v1beta/models/gemini-3.8-flash:generateContent";

// =================================================
// GLOBAL VARIABLES & MEMORY BUFFERS
// =================================================
I2SClass I2S;
bool isRecording = false;
bool bufferFull = false;

uint8_t* wavBuffer = nullptr;
size_t wavBufferSize = 0;

void audioInit();
void startRecording();
void stopRecording();
void sendToGemini();
char* psramBase64Encode(const uint8_t* input, size_t inputLen, size_t* outputLen);

// =================================================
// SETUP
// =================================================
void setup() {
    Serial.begin(115200);

    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000)) {
        delay(10);
    }

    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println("\n[NEXUS] Initializing DFRobot ESP32-S3...");

    if (!psramFound()) {
        Serial.println("[ERROR] PSRAM not detected! Verify OPI PSRAM settings in Arduino IDE.");
        while (true) delay(1000);
    }
    Serial.printf("[SYSTEM] Free PSRAM: %d KB\n", ESP.getFreePsram() / 1024);

    // 1. Initialize Camera FIRST so GPIO 41/42 allocate cleanly
    if (!initCamera()) {
        Serial.println("[ERROR] Camera initialization failed!");
        while (true) delay(1000);
    }

    // 2. Initialize Microphone SECOND
    audioInit();

    // Allocate continuous audio buffer in PSRAM
    wavBuffer = (uint8_t*)ps_malloc(PCM_WAV_HEADER_SIZE + MAX_AUDIO_BYTES);
    if (!wavBuffer) {
        Serial.println("[ERROR] Failed to allocate PSRAM audio buffer!");
        while (true) delay(1000);
    }

    WiFi.begin(ssid, password);
    WiFi.setSleep(false);

    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("\n[NET] WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    digitalWrite(LED_PIN, HIGH);
    Serial.println("\n--- Project Nexus AI Assistant Ready ---");
    Serial.println("Press and hold BOOT button to talk.");
}

// =================================================
// MAIN LOOP
// =================================================
void loop() {
    if (digitalRead(BUTTON_PIN) == LOW) {
        if (!isRecording) {
            startRecording();
        }
    } else {
        if (isRecording) {
            stopRecording();
        }
    }

    if (isRecording && !bufferFull) {
        size_t available = I2S.available();
        if (available > 0) {
            size_t remaining = MAX_AUDIO_BYTES - (wavBufferSize - PCM_WAV_HEADER_SIZE);
            size_t toRead = min(available, remaining);

            if (toRead == 0) {
                bufferFull = true;
                Serial.println("[AUDIO] Buffer full reached.");
            } else {
                size_t bytesRead = I2S.readBytes((char*)(wavBuffer + wavBufferSize), toRead);
                wavBufferSize += bytesRead;

                if ((wavBufferSize - PCM_WAV_HEADER_SIZE) >= MAX_AUDIO_BYTES) {
                    bufferFull = true;
                    Serial.println("[AUDIO] Reached max recording limit.");
                }
            }
        }
    }
    delay(10);
}

// =================================================
// AUDIO INITIALIZATION
// =================================================
void audioInit() {
    I2S.setPinsPdmRx(CLOCK_PIN, DATA_PIN);
    if (!I2S.begin(I2S_MODE_PDM_RX, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO)) {
        Serial.println("[ERROR] PDM Microphone initialization failed!");
        while (true) delay(1000);
    }
    Serial.println("[AUDIO] Microphone initialized.");
}

// =================================================
// START RECORDING
// =================================================
void startRecording() {
    wavBufferSize = PCM_WAV_HEADER_SIZE;
    bufferFull = false;

    createWavHeader(wavBuffer, 0, SAMPLE_RATE);

    isRecording = true;
    digitalWrite(LED_PIN, LOW);
    Serial.println("\n[AUDIO] Recording started...");
}

// =================================================
// STOP RECORDING
// =================================================
void stopRecording() {
    if (!isRecording) return;
    isRecording = false;
    digitalWrite(LED_PIN, HIGH);

    Serial.printf("[AUDIO] Recording finished. Size: %u bytes\n", (unsigned int)wavBufferSize);

    if (wavBufferSize <= PCM_WAV_HEADER_SIZE + 800) {
        Serial.println("[AUDIO] Audio trigger too short. Discarding.");
        wavBufferSize = 0;
        return;
    }

    int dataSize = wavBufferSize - PCM_WAV_HEADER_SIZE;
    createWavHeader(wavBuffer, dataSize, SAMPLE_RATE);

    sendToGemini();
    wavBufferSize = 0;
}

// =================================================
// PSRAM BASE64 ENCODER
// =================================================
char* psramBase64Encode(const uint8_t* input, size_t inputLen, size_t* outputLen) {
    size_t encodedLen = 0;
    mbedtls_base64_encode(NULL, 0, &encodedLen, input, inputLen);

    char* outBuf = (char*)ps_malloc(encodedLen + 1);
    if (!outBuf) return NULL;

    mbedtls_base64_encode((unsigned char*)outBuf, encodedLen + 1, outputLen, input, inputLen);
    outBuf[*outputLen] = '\0';
    return outBuf;
}

// =================================================
// GEMINI API COMMUNICATION
// =================================================

void sendToGemini() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[NET] WiFi connection lost.");
        return;
    }

    Serial.println("[GEMINI] Starting text-only test...");

    JsonDocument doc;

    JsonArray parts = doc["contents"][0]["parts"].to<JsonArray>();

    JsonObject textPart = parts.add<JsonObject>();
    textPart["text"] = "Hello Gemini! Reply with: Nexus is online.";

    doc["generationConfig"]["maxOutputTokens"] = 100;

    String payload;
    serializeJson(doc, payload);

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient https;

    String requestURL = String(geminiURL) + "?key=" + apiKey;

    if (https.begin(client, requestURL)) {
        https.addHeader("Content-Type", "application/json");
        https.setTimeout(30000);

        Serial.println("[HTTP] Sending text-only request...");

        int httpCode = https.POST(payload);
        String response = https.getString();

        Serial.printf("[HTTP] Code: %d\n", httpCode);
        Serial.println("[GEMINI RESPONSE]");
        Serial.println(response);

        if (httpCode == HTTP_CODE_OK) {
            JsonDocument resDoc;

            DeserializationError err =
                deserializeJson(resDoc, response);

            if (!err) {
                const char* reply =
                    resDoc["candidates"][0]["content"]["parts"][0]["text"];

                if (reply) {
                    Serial.println("[GEMINI] Response:");
                    Serial.println(reply);
                }
            }
        }

        https.end();
    } else {
        Serial.println("[ERROR] HTTPS connection failed.");
    }

    Serial.println("[SYSTEM] Text-only test finished!");
}
