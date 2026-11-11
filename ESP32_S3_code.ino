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
// CORRECTED HARDWARE CONFIGURATION: DFRobot DFR1154
// =================================================
#define BUTTON_PIN 0
#define LED_PIN    3

#define SAMPLE_RATE 16000
// DFR1154 Onboard MSM261S4030H0 PDM Microphone Pins
#define CLOCK_PIN   GPIO_NUM_42  // PDM CLK (Mic Clock)
#define DATA_PIN    GPIO_NUM_41  // PDM DAT (Mic Data)

#define MAX_RECORDING_SECONDS 4
#define PCM_WAV_HEADER_SIZE 44
#define MAX_AUDIO_BYTES (SAMPLE_RATE * 2 * MAX_RECORDING_SECONDS)
const char* ssid = "*******";
const char* password = "******";
const char* apiKey = "********";

const char* geminiURL = "https://generativelanguage.googleapis.com/v1beta/models/gemini-3.8-flash:generateContent";


// =================================================
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
    // Read BOOT button state (Active LOW)
    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(20); // Basic debounce delay
        if (digitalRead(BUTTON_PIN) == LOW && !isRecording) {
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
                size_t bytesRead = I2S.readBytes(
                    (char*)(wavBuffer + wavBufferSize),
                    toRead
                );

                // Boost/align MEMS PDM mic gain
                int16_t* readSamples = (int16_t*)(wavBuffer + wavBufferSize);
                size_t sampleCount = bytesRead / 2;
                for (size_t i = 0; i < sampleCount; i++) {
                    readSamples[i] = readSamples[i] << 3;
                }

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

void startRecording() {
    // Safely drain leftover audio without getting stuck in a loop
    size_t flushBytes = I2S.available();
    if (flushBytes > 0) {
        size_t bytesToFlush = min(flushBytes, (size_t)1024);
        uint8_t dummyBuffer[128];
        while (bytesToFlush > 0) {
            size_t chunk = min(bytesToFlush, sizeof(dummyBuffer));
            I2S.readBytes((char*)dummyBuffer, chunk);
            bytesToFlush -= chunk;
        }
    }

    // Reset recording buffer state
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

    if (wavBuffer == nullptr || wavBufferSize <= PCM_WAV_HEADER_SIZE) {
        Serial.println("[AUDIO] No valid recording available.");
        return;
    }

    Serial.println("[GEMINI] Preparing audio...");

    // Camera capture test
    Serial.println("[CAMERA] Trying to capture image...");

    camera_fb_t* fb = esp_camera_fb_get();

    if (fb != NULL) {
        Serial.printf("[CAMERA] SUCCESS! Image size: %u bytes\n", fb->len);
        esp_camera_fb_return(fb);
    } else {
        Serial.println("[CAMERA] FAILED! No image captured.");
    }

    // Convert WAV audio to Base64
    size_t base64Length = 0;

    char* audioBase64 = psramBase64Encode(
        wavBuffer,
        wavBufferSize,
        &base64Length
    );

    if (audioBase64 == NULL || base64Length == 0) {
        Serial.println("[ERROR] Audio Base64 encoding failed.");
        return;
    }

    Serial.printf("[AUDIO] Base64 size: %u bytes\n",
                  (unsigned int)base64Length);
    Serial.printf("[AUDIO] First Base64 characters: %.20s\n",
                  audioBase64);

    // Build Gemini request
    JsonDocument doc;

    JsonArray parts = doc["contents"][0]["parts"].to<JsonArray>();

    JsonObject textPart = parts.add<JsonObject>();

    textPart["text"] =
        "Listen carefully to the audio. First, transcribe exactly "
        "what the speaker says. Do not guess if speech is unclear. "
        "Then answer the speaker's question or respond naturally. "
        "Use this format:\n"
        "TRANSCRIPT: [the exact words heard]\n"
        "ANSWER: [your response]";

    JsonObject audioPart = parts.add<JsonObject>();
    JsonObject inlineData = audioPart["inlineData"].to<JsonObject>();

    inlineData["mimeType"] = "audio/wav";
    inlineData["data"] = audioBase64;

    doc["generationConfig"]["maxOutputTokens"] = 300;

   
    // Build JSON manually to avoid ArduinoJson memory limitations
    String payload;

    if (!payload.reserve(base64Length + 600)) {
        Serial.println("[ERROR] Could not allocate JSON payload memory.");
        free(audioBase64);
        return;
    }

    payload =
        "{\"contents\":[{\"parts\":["
        "{\"text\":\"Listen carefully to the audio. First, transcribe exactly "
        "what the speaker says. Do not guess if speech is unclear. "
        "Then answer the speaker's question or respond naturally. "
        "Use this format:\\n"
        "TRANSCRIPT: [the exact words heard]\\n"
        "ANSWER: [your response]\"},"
        "{\"inlineData\":{\"mimeType\":\"audio/wav\",\"data\":\"";

    // Insert the complete Base64 audio
    payload += audioBase64;

    payload +=
        "\"}}]}],"
        "\"generationConfig\":{\"maxOutputTokens\":300}}";

    Serial.printf("[DEBUG] JSON payload size: %u bytes\n",
                  (unsigned int)payload.length());

    if (payload.indexOf("\"data\":null") >= 0 ||
        payload.length() < base64Length) {
        Serial.println("[ERROR] Audio data was not inserted correctly!");
        free(audioBase64);
        return;
    }

    Serial.println("[DEBUG] Audio data successfully inserted into JSON!");

    // Base64 is no longer needed
    free(audioBase64);


    Serial.println("[GEMINI] Sending recorded audio...");

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient https;

    String requestURL = String(geminiURL) + "?key=" + apiKey;

    if (https.begin(client, requestURL)) {

        https.addHeader("Content-Type", "application/json");
        https.setTimeout(60000);

        int httpCode = https.POST(payload);

        Serial.printf("[HTTP] Code: %d\n", httpCode);

        String response = https.getString();

        if (httpCode == HTTP_CODE_OK) {

            JsonDocument responseDoc;

            DeserializationError error =
                deserializeJson(responseDoc, response);

            if (!error) {

                const char* geminiText =
                    responseDoc["candidates"][0]["content"]["parts"][0]["text"];

                if (geminiText != NULL) {
                    Serial.println("\n========== NEXUS AI ==========");
                    Serial.println(geminiText);
                    Serial.println("==============================");
                } else {
                    Serial.println("[ERROR] No text found in Gemini response.");
                    Serial.println(response);
                }

            } else {
                Serial.println("[ERROR] Response JSON parsing failed.");
                Serial.println(response);
            }

        } else {
            Serial.println("[GEMINI ERROR RESPONSE]");
            Serial.println(response);
        }

        https.end();

    } else {
        Serial.println("[ERROR] HTTPS connection failed.");
    }

    Serial.println("[SYSTEM] Audio test finished!");
}
