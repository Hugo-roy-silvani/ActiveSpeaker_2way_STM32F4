/*
 * DSP.h
 *
 *  Created on: 5 déc. 2025
 *      Author: hugo
 */

#ifndef DSP_H
#define DSP_H

#include <stdint.h>

#define AUDIO_FRAMES_PER_HALF   256
#define DSP_MAX_EQ_PER_CHANNEL  8   // par ex : jusqu’à 8 biquads d’EQ par voie
#define DSP_MAX_HPF_STAGES  3
#define DSP_MAX_DELAY_SAMPLES   1024   // par ex : ~21 ms à 48 kHz

typedef enum {
    DSP_CH_LOW = 0,   // voie grave
    DSP_CH_HIGH = 1,  // voie aigu
    DSP_CH_COUNT
} DspChannelId;

// Config d’EQ pour une voie (liste de biquads supplémentaires)
typedef struct {
    uint32_t numEq;                            // nombre de biquads utilisés
    float    eqCoeffs[DSP_MAX_EQ_PER_CHANNEL][5]; // coefficients CMSIS {b0,b1,b2,a1,a2}
} DspEqConfig;

typedef struct {
    uint8_t enabled;      // 0 = off, 1 = on
    float   threshold_dB; // seuil en dBFS (ex: -1.0f, -3.0f, -6.0f)
    float   ratio;        // ex: 10.0f, 20.0f, ou très grand pour brickwall
    float   attack_ms;    // temps d'attaque
    float   release_ms;   // temps de release
    float   makeupGain_dB;// souvent 0 pour un limiteur de protection
} DspLimiterConfig;

typedef struct {
    uint8_t enabled;   // 0 = off, 1 = on
    uint8_t stages;    // nb de biquads en cascade (1..DSP_MAX_HPF_STAGES)
    float   freq;      // fréquence de coupure (Hz)
    float   Q;         // Q du filtre (ex: 0.707 pour Butterworth)
} DspHpfConfig;

// Config complète DSP
typedef struct {
    float sampleRate;      // Hz: ex. 48000
    float crossoverFreq;   // Hz: ex. 2000

    float inputTrim_dB; // Trim global en entrée

    // paramètres par voie
    float gain[DSP_CH_COUNT];          // gain linéaire par voie (LOW/HIGH)
    int   delaySamples[DSP_CH_COUNT];  // délai en samples par voie (pour plus tard)

    uint8_t mute[DSP_CH_COUNT];        // 0 = normal, 1 = mute
    uint8_t invert[DSP_CH_COUNT];      // 0 = normal, 1 = inversion de polarité

    DspHpfConfig      hpf;

    DspEqConfig eq[DSP_CH_COUNT];

    DspLimiterConfig limiter[DSP_CH_COUNT];
} DspConfig;

// API
void DSP_Init(const DspConfig *cfg);
void DSP_ProcessHalfBuffer(int16_t *buf, uint32_t frames);

#endif // DSP_H

