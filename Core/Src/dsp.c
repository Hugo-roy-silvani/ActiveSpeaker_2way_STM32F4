/*
 * dsp.c
 *
 *  Created on: 5 déc. 2025
 *      Author: hugo
 */

#include <dsp.h>
#include "arm_math.h"
#include <math.h>

// Constante locale pour le DSP
#define TWO_PI_DSP       6.28318530718f

// Limiteur : état par voie (pour plus tard, pas encore utilisé ici)
typedef struct {
    float env;           // enveloppe actuelle
    float thresh_lin;    // seuil en linéaire
    float makeup_lin;    // gain de makeup linéaire
    float attackCoeff;   // coef pour le one-pole d'attaque
    float releaseCoeff;  // coef pour le one-pole de release
} DspLimiterState;

// Delay
typedef struct {
    float    buffer[DSP_MAX_DELAY_SAMPLES];
    uint32_t writeIdx;
} DspDelayState;

static DspDelayState g_delayState[DSP_CH_COUNT];
static uint32_t      g_delayLen[DSP_CH_COUNT];   // délai effectif (clampé)

// Copie locale de la config active
static DspConfig g_cfg;

// crossover LR24
#define XO_BIQUAD_STAGES   2

static arm_biquad_cascade_df2T_instance_f32 biquadXO[DSP_CH_COUNT];
static float32_t biquad_coeffsXO[DSP_CH_COUNT][5 * XO_BIQUAD_STAGES];
static float32_t biquad_stateXO[DSP_CH_COUNT][4 * XO_BIQUAD_STAGES];

// EQ
static arm_biquad_cascade_df2T_instance_f32 biquadEq[DSP_CH_COUNT];
static float32_t biquad_coeffsEq[DSP_CH_COUNT][5 * DSP_MAX_EQ_PER_CHANNEL];
static float32_t biquad_stateEq[DSP_CH_COUNT][4 * DSP_MAX_EQ_PER_CHANNEL];

// Limiter
static DspLimiterState g_limiterState[DSP_CH_COUNT];

// HPF de protection
static arm_biquad_cascade_df2T_instance_f32 biquadHPF;
static float32_t biquad_coeffsHPF[5 * DSP_MAX_HPF_STAGES];
static float32_t biquad_stateHPF[4 * DSP_MAX_HPF_STAGES];

static float     g_inputTrimLin = 1.0f;  // trim global en linéaire


// =======================================================
//   Reconstruction du crossover LR24 à partir de g_cfg
// =======================================================

static void DSP_RebuildCrossover(void)
{
    float fs = g_cfg.sampleRate;
    float f0 = g_cfg.crossoverFreq;
    float Q  = 0.707f;

    float w0   = 2.0f * TWO_PI_DSP * f0 / fs;
    float cosw = cosf(w0);
    float sinw = sinf(w0);
    float alpha = sinw / (2.0f * Q);

    // ===== Low-pass RBJ (1 section) =====
    float b0_lp = (1.0f - cosw) * 0.5f;
    float b1_lp =  1.0f - cosw;
    float b2_lp = (1.0f - cosw) * 0.5f;
    float a0_lp =  1.0f + alpha;
    float a1_lp = -2.0f * cosw;
    float a2_lp =  1.0f - alpha;

    float b0n_lp = b0_lp / a0_lp;
    float b1n_lp = b1_lp / a0_lp;
    float b2n_lp = b2_lp / a0_lp;
    float a1n_lp = a1_lp / a0_lp;
    float a2n_lp = a2_lp / a0_lp;

    float secLP[5] = {
        b0n_lp,
        b1n_lp,
        b2n_lp,
        -a1n_lp,
        -a2n_lp
    };

    // ===== High-pass RBJ (1 section) =====
    float b0_hp =  (1.0f + cosw) * 0.5f;
    float b1_hp = -(1.0f + cosw);
    float b2_hp =  (1.0f + cosw) * 0.5f;
    float a0_hp =   1.0f + alpha;
    float a1_hp =  -2.0f * cosw;
    float a2_hp =   1.0f - alpha;

    float b0n_hp = b0_hp / a0_hp;
    float b1n_hp = b1_hp / a0_hp;
    float b2n_hp = b2_hp / a0_hp;
    float a1n_hp = a1_hp / a0_hp;
    float a2n_hp = a2_hp / a0_hp;

    float secHP[5] = {
        b0n_hp,
        b1n_hp,
        b2n_hp,
        -a1n_hp,
        -a2n_hp
    };

    // ===== Dupliquer les sections pour LR24 sur chaque voie =====
    for (int s = 0; s < XO_BIQUAD_STAGES; s++) {
        for (int k = 0; k < 5; k++) {
            biquad_coeffsXO[DSP_CH_LOW ][5*s + k] = secLP[k];
            biquad_coeffsXO[DSP_CH_HIGH][5*s + k] = secHP[k];
        }
    }

    // États à zéro + init CMSIS par voie
    for (int ch = 0; ch < DSP_CH_COUNT; ch++) {
        for (int i = 0; i < 4 * XO_BIQUAD_STAGES; i++) {
            biquad_stateXO[ch][i] = 0.0f;
        }

        arm_biquad_cascade_df2T_init_f32(
            &biquadXO[ch],
            XO_BIQUAD_STAGES,
            &biquad_coeffsXO[ch][0],
            &biquad_stateXO[ch][0]
        );
    }
}


// =================== Build EQ ====================

static void DSP_RebuildEq(void)
{
    for (int ch = 0; ch < DSP_CH_COUNT; ch++)
    {
        DspEqConfig *cfgEq = &g_cfg.eq[ch];

        uint32_t num = cfgEq->numEq;
        if (num == 0 || num > DSP_MAX_EQ_PER_CHANNEL)
        {
            // Aucun EQ pour cette voie (ou clamp)
            if (num > DSP_MAX_EQ_PER_CHANNEL) {
                num = DSP_MAX_EQ_PER_CHANNEL;
                cfgEq->numEq = num;
            }

            continue;
        }

        // Copier les coeffs de la config vers le buffer plat CMSIS
        for (uint32_t s = 0; s < num; s++)
        {
            for (int k = 0; k < 5; k++)
            {
                biquad_coeffsEq[ch][5*s + k] = cfgEq->eqCoeffs[s][k];
            }
        }

        // Remise à zéro des états pour ces sections
        for (uint32_t i = 0; i < 4 * num; i++)
        {
            biquad_stateEq[ch][i] = 0.0f;
        }

        // Init de la cascade d’EQ pour cette voie
        arm_biquad_cascade_df2T_init_f32(
            &biquadEq[ch],
            num,
            &biquad_coeffsEq[ch][0],
            &biquad_stateEq[ch][0]
        );
    }
}

// ===================Limiter ====================

static float dB_to_lin(float dB)
{
    return powf(10.0f, dB / 20.0f);
}

static void DSP_RebuildLimiter(void)
{
    float fs = g_cfg.sampleRate;

    for (int ch = 0; ch < DSP_CH_COUNT; ch++)
    {
        DspLimiterConfig *cfgL = &g_cfg.limiter[ch];
        DspLimiterState  *st   = &g_limiterState[ch];

        st->env = 0.0f;

        st->thresh_lin = dB_to_lin(cfgL->threshold_dB);
        st->makeup_lin = dB_to_lin(cfgL->makeupGain_dB);

        float attackSec  = cfgL->attack_ms  / 1000.0f;
        float releaseSec = cfgL->release_ms / 1000.0f;

        if (attackSec  < 1e-6f) attackSec  = 1e-6f;
        if (releaseSec < 1e-6f) releaseSec = 1e-6f;

        st->attackCoeff  = expf(-1.0f / (attackSec  * fs));
        st->releaseCoeff = expf(-1.0f / (releaseSec * fs));
    }
}

static float DSP_ApplyLimiterSample(int ch, float x)
{
    DspLimiterConfig *cfgL = &g_cfg.limiter[ch];
    DspLimiterState  *st   = &g_limiterState[ch];

    if (!cfgL->enabled) {
        return x; // bypass
    }

    float ax = fabsf(x);

    // enveloppe attaque/release
    if (ax > st->env) {
        st->env = st->attackCoeff * st->env + (1.0f - st->attackCoeff) * ax;
    } else {
        st->env = st->releaseCoeff * st->env + (1.0f - st->releaseCoeff) * ax;
    }

    if (st->env <= st->thresh_lin) {
        return x * st->makeup_lin;
    }

    float ratio = cfgL->ratio;
    if (ratio < 1.0f) ratio = 1.0f;

    // G = (threshold/env)^(1 - 1/ratio)
    float G = st->thresh_lin / (st->env + 1e-12f);
    float expR = 1.0f - 1.0f / ratio;
    G = powf(G, expR);

    return x * G * st->makeup_lin;
}

// =================== HPF de protection ===========

static void DSP_RebuildHpf(void)
{
    if (!g_cfg.hpf.enabled) {
        return;
    }

    float fs = g_cfg.sampleRate;
    float f0 = g_cfg.hpf.freq;
    float Q  = g_cfg.hpf.Q;
    int   stages = g_cfg.hpf.stages;

    if (f0 <= 0.0f || fs <= 0.0f) {
        g_cfg.hpf.enabled = 0;
        return;
    }

    if (stages < 1) stages = 1;
    if (stages > DSP_MAX_HPF_STAGES) stages = DSP_MAX_HPF_STAGES;
    g_cfg.hpf.stages = (uint8_t)stages;

    float w0   = 2.0f * TWO_PI_DSP * f0 / fs;
    float cosw = cosf(w0);
    float sinw = sinf(w0);
    float alpha = sinw / (2.0f * Q);

    // RBJ high-pass 2ème ordre
    float b0 =  (1.0f + cosw) * 0.5f;
    float b1 = -(1.0f + cosw);
    float b2 =  (1.0f + cosw) * 0.5f;
    float a0 =   1.0f + alpha;
    float a1 =  -2.0f * cosw;
    float a2 =   1.0f - alpha;

    float b0n = b0 / a0;
    float b1n = b1 / a0;
    float b2n = b2 / a0;
    float a1n = a1 / a0;
    float a2n = a2 / a0;

    float sec[5] = {
        b0n,
        b1n,
        b2n,
        -a1n,   // convention CMSIS: a1,a2 neg
        -a2n
    };

    // Dupliquer la même section sur `stages` biquads
    for (int s = 0; s < stages; s++) {
        for (int k = 0; k < 5; k++) {
            biquad_coeffsHPF[5*s + k] = sec[k];
        }
    }

    // Remise à zéro des états pour toutes les sections
    for (int i = 0; i < 4 * stages; i++) {
        biquad_stateHPF[i] = 0.0f;
    }

    // Init CMSIS
    arm_biquad_cascade_df2T_init_f32(
        &biquadHPF,
        stages,
        biquad_coeffsHPF,
        biquad_stateHPF
    );
}

// ================== Delay ======================

static float DSP_ApplyDelaySample(int ch, float x)
{
    uint32_t N = g_delayLen[ch];

    if (N == 0) {
        // pas de délai
        return x;
    }

    DspDelayState *st = &g_delayState[ch];

    uint32_t w = st->writeIdx;
    uint32_t readIdx = (w + DSP_MAX_DELAY_SAMPLES - N) % DSP_MAX_DELAY_SAMPLES;

    float y = st->buffer[readIdx];

    st->buffer[w] = x;
    st->writeIdx = (w + 1) % DSP_MAX_DELAY_SAMPLES;

    return y;
}


// =================== INIT DSP ===================

void DSP_Init(const DspConfig *cfg)
{
    // copie la config
    g_cfg = *cfg;

    g_inputTrimLin = dB_to_lin(g_cfg.inputTrim_dB);
    // Init des délais par voie
    for (int ch = 0; ch < DSP_CH_COUNT; ch++)
    {
        int d = g_cfg.delaySamples[ch];

        if (d < 0) {
            d = 0;
        }
        if (d >= DSP_MAX_DELAY_SAMPLES) {
            d = DSP_MAX_DELAY_SAMPLES - 1;
        }

        g_delayLen[ch] = (uint32_t)d;

        // buffer à zéro et index remis à 0
        for (int i = 0; i < DSP_MAX_DELAY_SAMPLES; i++) {
            g_delayState[ch].buffer[i] = 0.0f;
        }
        g_delayState[ch].writeIdx = 0;
    }
    DSP_RebuildCrossover();
    DSP_RebuildEq();
    DSP_RebuildLimiter();
    DSP_RebuildHpf();
}

// =================== TRAITEMENT D’UN DEMI-BUFFER ===================

void DSP_ProcessHalfBuffer(int16_t *buf, uint32_t frames)
{
    // buffers temporaires mono (taille = AUDIO_FRAMES_PER_HALF)
    static float32_t in[AUDIO_FRAMES_PER_HALF];
    static float32_t outLP[AUDIO_FRAMES_PER_HALF];
    static float32_t outHP[AUDIO_FRAMES_PER_HALF];

    // 1) Entrée mono : on prend le canal L
    for (uint32_t n = 0; n < frames; n++)
    {
        float l = (float32_t)buf[2*n + 0];
        float r = (float32_t)buf[2*n + 1];

        float mono = 0.5f * (l + r);    // mix L+R avec -6 dB

        in[n] = (mono / 32768.0f) * g_inputTrimLin;
    }

    // 1bis) HPF de protection global (subsonique)
    if (g_cfg.hpf.enabled)
    {
        arm_biquad_cascade_df2T_f32(&biquadHPF, in, in, frames);
    }

    // 2) Filtrage crossover
    arm_biquad_cascade_df2T_f32(&biquadXO[DSP_CH_LOW],  in, outLP, frames);
    arm_biquad_cascade_df2T_f32(&biquadXO[DSP_CH_HIGH], in, outHP, frames);

    // 3) Gains par voie (issus de la config)
    float gainLP = g_cfg.gain[DSP_CH_LOW];
    float gainHP = g_cfg.gain[DSP_CH_HIGH];

    for (uint32_t n = 0; n < frames; n++)
    {
        outLP[n] *= gainLP;
        outHP[n] *= gainHP;
    }

    // (plus tard : EQ par voie, limiteur par voie, délais, etc.)
    // 3bis) EQ par voie (si numEq > 0)
        if (g_cfg.eq[DSP_CH_LOW].numEq > 0)
        {
            arm_biquad_cascade_df2T_f32(&biquadEq[DSP_CH_LOW],
                                        outLP, outLP, frames);
        }

        if (g_cfg.eq[DSP_CH_HIGH].numEq > 0)
        {
            arm_biquad_cascade_df2T_f32(&biquadEq[DSP_CH_HIGH],
                                        outHP, outHP, frames);
        }

    // 4) Remapping vers le buffer I2S : L = grave, R = aigu
    for (uint32_t n = 0; n < frames; n++)
    {
        float yL = outLP[n];
        float yH = outHP[n];

        yL = DSP_ApplyDelaySample(DSP_CH_LOW,  yL);
        yH = DSP_ApplyDelaySample(DSP_CH_HIGH, yH);

        yL = DSP_ApplyLimiterSample(DSP_CH_LOW,  yL);
        yH = DSP_ApplyLimiterSample(DSP_CH_HIGH, yH);

        if (g_cfg.mute[DSP_CH_LOW])  yL = 0.0f;
        if (g_cfg.mute[DSP_CH_HIGH]) yH = 0.0f;

        if (g_cfg.invert[DSP_CH_LOW])  yL = -yL;
        if (g_cfg.invert[DSP_CH_HIGH]) yH = -yH;

        float xL = yL * 32767.0f;
        float xR = yH * 32767.0f;

        if (xL > 32767.0f) xL = 32767.0f;
        if (xL < -32768.0f) xL = -32768.0f;
        if (xR > 32767.0f) xR = 32767.0f;
        if (xR < -32768.0f) xR = -32768.0f;

        buf[2*n + 0] = (int16_t)xL;   // voie grave
        buf[2*n + 1] = (int16_t)xR;   // voie aigu
    }
}
