#ifndef WAV_HEADER_H
#define WAV_HEADER_H

#include <Arduino.h>

// Generates a standard 44-byte WAV header for 16kHz, 16-bit Mono PCM audio
inline void createWavHeader(byte* header, int waveDataSize, int sampleRate = 16000) {
  int totalDataLen = waveDataSize + 36;
  int byteRate = sampleRate * 1 * 2; // SampleRate * Channels * (BitsPerSample/8)

  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  header[4] = (byte)(totalDataLen & 0xff);
  header[5] = (byte)((totalDataLen >> 8) & 0xff);
  header[6] = (byte)((totalDataLen >> 16) & 0xff);
  header[7] = (byte)((totalDataLen >> 24) & 0xff);
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  header[16] = 16;  // 16 for PCM
  header[17] = 0;
  header[18] = 0;
  header[19] = 0;
  header[20] = 1;   // PCM Format
  header[21] = 0;
  header[22] = 1;   // Mono (1 channel)
  header[23] = 0;
  header[24] = (byte)(sampleRate & 0xff);
  header[25] = (byte)((sampleRate >> 8) & 0xff);
  header[26] = (byte)((sampleRate >> 16) & 0xff);
  header[27] = (byte)((sampleRate >> 24) & 0xff);
  header[28] = (byte)(byteRate & 0xff);
  header[29] = (byte)((byteRate >> 8) & 0xff);
  header[30] = (byte)((byteRate >> 16) & 0xff);
  header[31] = (byte)((byteRate >> 24) & 0xff);
  header[32] = 2;   // Block align (Channels * BitsPerSample / 8)
  header[33] = 0;
  header[34] = 16;  // Bits per sample
  header[35] = 0;
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  header[40] = (byte)(waveDataSize & 0xff);
  header[41] = (byte)((waveDataSize >> 8) & 0xff);
  header[42] = (byte)((waveDataSize >> 16) & 0xff);
  header[43] = (byte)((waveDataSize >> 24) & 0xff);
}

#endif // WAV_HEADER_H