# Seymour - Technical Architecture Document

**Author:** Neal
**Date:** 2025-12-20
**Version:** 1.0
**PRD Reference:** prd.md

---

## Overview

Seymour is a disting NT C++ plugin implementing a multi-input feedback mixer with per-channel feedback control and a safety limiter. This document specifies the technical architecture for implementation.

---

## System Architecture

### High-Level Block Diagram

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              SEYMOUR PLUGIN                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│  ┌──────────┐   ┌──────────────┐   ┌─────────┐                              │
│  │ Input 1  │──▶│ Feedback     │──▶│ Panner  │──┐                           │
│  │ (+ FB)   │   │ + DC Block   │   │         │  │                           │
│  └──────────┘   └──────────────┘   └─────────┘  │                           │
│       ▲                                          │    ┌──────────────────┐   │
│       └──────────────────────────────────────────┼────│ Stereo Mix Bus   │   │
│                                                  │    │                  │   │
│  ┌──────────┐   ┌──────────────┐   ┌─────────┐  │    │  L accumulator   │   │
│  │ Input 2  │──▶│ Feedback     │──▶│ Panner  │──┼───▶│  R accumulator   │   │
│  │ (+ FB)   │   │ + DC Block   │   │         │  │    │                  │   │
│  └──────────┘   └──────────────┘   └─────────┘  │    └────────┬─────────┘   │
│       ▲                                          │             │             │
│       └──────────────────────────────────────────┘             ▼             │
│                                                        ┌───────────────┐     │
│  ... (N inputs based on specification)                 │ Master Level  │     │
│                                                        └───────┬───────┘     │
│                                                                │             │
│                                                                ▼             │
│                                                   ┌─────────────────────┐   │
│                                                   │  Lookahead Limiter  │   │
│                                                   │  + Saturator        │   │
│                                                   └──────────┬──────────┘   │
│                                                              │              │
│                                                              ▼              │
│                                                   ┌─────────────────────┐   │
│                                                   │   Output L / R      │   │
│                                                   └─────────────────────┘   │
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Plugin Identity

```cpp
// GUID: "NsSy" = Nealsanche + Seymour
.guid = NT_MULTICHAR('N', 's', 'S', 'y')
.name = "Seymour"
.description = "Feedback mixer with safety limiter"
.tags = kNT_tagEffect | kNT_tagUtility
```

---

## Specification

```cpp
static const _NT_specification specifications[] = {
    {
        .name = "Inputs",
        .min = 1,
        .max = 8,
        .def = 2,
        .type = kNT_typeGeneric
    },
};
```

User selects input count (1-8) at instance creation. This determines:
- Number of input channels
- Parameter count
- Memory allocation

---

## Data Structures

### Main Algorithm Struct (SRAM)

```cpp
struct _seymourAlgorithm : public _NT_algorithm {
    _seymourAlgorithm() {}
    ~_seymourAlgorithm() {}

    // Configuration (set once in construct)
    int numInputs;

    // Pointers to DTC/DRAM memory
    struct _seymourDTC* dtc;
    float* lookaheadBuffer;  // DRAM - stereo interleaved

    // Parameter cache (smoothed values)
    float* feedbackSmoothed;    // [numInputs]
    float* panSmoothed;         // [numInputs]
    float masterLevelSmoothed;

    // Feedback state (per-channel)
    float* feedbackState;       // [numInputs] - previous output sample

    // DC blocker state (per-channel)
    float* dcBlockerState;      // [numInputs * 2] - x[n-1], y[n-1] per channel
};
```

### DTC Struct (Fast Access)

```cpp
struct _seymourDTC {
    // Limiter state
    float envelopeL;
    float envelopeR;
    float gainReduction;

    // Lookahead buffer state
    uint32_t writeIndex;
    uint32_t lookaheadSamples;

    // Saturation mode (cached)
    int saturationMode;

    // Precomputed coefficients
    float dcBlockerCoeff;       // ~0.995 for 5Hz HPF @ 48kHz
    float envelopeAttack;       // Fast attack for limiter
    float envelopeRelease;      // Slower release
    float smoothingCoeff;       // Parameter smoothing
};
```

---

## Parameter System

### Parameter Indices

```cpp
enum ParameterIndex {
    // Per-channel parameters (repeated numInputs times)
    kParamInputBase = 0,        // Input bus selector
    kParamFeedbackBase,         // Feedback amount 0-100%
    kParamFeedbackCVBase,       // Feedback CV input
    kParamFeedbackCVDepthBase,  // Feedback CV depth 0-100%
    kParamPanBase,              // Pan -100 to +100
    kParamPanCVBase,            // Pan CV input
    kParamPanCVDepthBase,       // Pan CV depth 0-100%
    kParamsPerChannel = 7,

    // Global parameters (after per-channel)
    // Indices calculated as: numInputs * kParamsPerChannel + offset
    kParamOutputL = 0,          // Offset from global base
    kParamOutputR,
    kParamOutputMode,
    kParamMasterLevel,
    kParamLookahead,
    kParamSaturation,
    kNumGlobalParams = 6,
};

// Helper macros
#define CHANNEL_PARAM(ch, param) ((ch) * kParamsPerChannel + (param))
#define GLOBAL_PARAM(numInputs, param) ((numInputs) * kParamsPerChannel + (param))
```

### Parameter Definitions

```cpp
// Per-channel parameters (template - instantiated per channel)
static const _NT_parameter channelParams[] = {
    NT_PARAMETER_AUDIO_INPUT("Input", 0, 1),
    { "Feedback", 0, 100, 0, kNT_unitPercent, 0, NULL },
    NT_PARAMETER_CV_INPUT("FB CV", 0, 0),
    { "FB Depth", 0, 100, 50, kNT_unitPercent, 0, NULL },
    { "Pan", -100, 100, 0, kNT_unitNone, 0, NULL },
    NT_PARAMETER_CV_INPUT("Pan CV", 0, 0),
    { "Pan Depth", 0, 100, 50, kNT_unitPercent, 0, NULL },
};

// Global parameters
static const char* saturationStrings[] = { "Soft", "Tube", "Hard", NULL };

static const _NT_parameter globalParams[] = {
    NT_PARAMETER_AUDIO_OUTPUT("Out L", 1, 13),
    NT_PARAMETER_AUDIO_OUTPUT("Out R", 1, 14),
    { "Mode", 0, 1, 1, kNT_unitOutputMode, 0, NULL },
    { "Level", 0, 100, 80, kNT_unitPercent, 0, NULL },
    { "Lookahead", 5, 200, 50, kNT_unitMs, kNT_scaling10, NULL },  // 0.5-20ms
    { "Saturation", 0, 2, 0, kNT_unitEnum, 0, saturationStrings },
};
```

### Parameter UI Prefix

```cpp
int parameterUiPrefix(_NT_algorithm* self, int p, char* buff) {
    _seymourAlgorithm* pThis = (_seymourAlgorithm*)self;

    if (p < pThis->numInputs * kParamsPerChannel) {
        int ch = p / kParamsPerChannel;
        int len = NT_intToString(buff, 1 + ch);
        buff[len++] = ':';
        buff[len] = 0;
        return len;
    }
    return 0;  // No prefix for global params
}
```

---

## Memory Requirements

### calculateRequirements

```cpp
void calculateRequirements(_NT_algorithmRequirements& req,
                          const int32_t* specifications)
{
    int numInputs = specifications[0];
    int numParams = numInputs * kParamsPerChannel + kNumGlobalParams;

    // Calculate lookahead buffer size (max 20ms @ 96kHz, stereo)
    int maxLookaheadSamples = (96000 * 20) / 1000;  // 1920 samples
    int lookaheadBufferSize = maxLookaheadSamples * 2 * sizeof(float);  // stereo

    req.numParameters = numParams;
    req.sram = sizeof(_seymourAlgorithm) +
               numInputs * sizeof(float) * 4;  // smoothed + state arrays
    req.dram = lookaheadBufferSize;
    req.dtc = sizeof(_seymourDTC);
    req.itc = 0;
}
```

### Memory Layout

| Region | Contents | Size |
|--------|----------|------|
| SRAM | Algorithm struct + per-channel arrays | ~200 + 32*N bytes |
| DTC | Limiter state, coefficients | ~64 bytes |
| DRAM | Lookahead delay buffer (stereo) | ~15KB max |

---

## DSP Algorithms

### 1. DC Blocker (Per-Channel)

Simple 1-pole high-pass filter at ~5Hz to prevent DC buildup in feedback.

```cpp
// Coefficient: R = 1 - (2 * pi * fc / fs)
// At 48kHz, fc=5Hz: R ≈ 0.9993
// At 96kHz, fc=5Hz: R ≈ 0.9997

inline float dcBlock(float input, float& x1, float& y1, float R) {
    float output = input - x1 + R * y1;
    x1 = input;
    y1 = output;
    return output;
}
```

### 2. Feedback Loop

```cpp
// Per sample, per channel:
float processChannel(float input, float feedbackAmount,
                     float& feedbackState, float& dcX1, float& dcY1, float R) {
    // Mix input with previous feedback
    float mixed = input + feedbackState * feedbackAmount;

    // DC block the result
    float output = dcBlock(mixed, dcX1, dcY1, R);

    // Store for next iteration
    feedbackState = output;

    return output;
}
```

### 3. Equal Power Panner

```cpp
// Pan range: -100 to +100 (param value)
// Convert to 0-1 range, then to angle

inline void equalPowerPan(float pan, float& gainL, float& gainR) {
    float p = (pan + 100.0f) / 200.0f;  // 0 to 1
    float angle = p * (M_PI / 2.0f);     // 0 to π/2
    gainL = cosf(angle);
    gainR = sinf(angle);
}
```

### 4. Saturation Modes

```cpp
enum SaturationMode {
    kSaturationSoft = 0,
    kSaturationTube,
    kSaturationHard,
};

// Soft (tanh) - symmetric, smooth
inline float saturateSoft(float x) {
    return tanhf(x);
}

// Tube - asymmetric, even harmonics, warm
inline float saturateTube(float x) {
    // Attempt to model 2nd harmonic presence
    // Soft positive, softer negative
    if (x >= 0.0f) {
        return tanhf(x * 0.8f) * 1.1f;
    } else {
        return tanhf(x * 1.2f) * 0.9f;
    }
}

// Hard - aggressive clipping
inline float saturateHard(float x) {
    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    // Slight curve near threshold for less harsh edge
    if (x > 0.8f) return 0.8f + (x - 0.8f) * 0.5f;
    if (x < -0.8f) return -0.8f + (x + 0.8f) * 0.5f;
    return x;
}

inline float saturate(float x, int mode) {
    switch (mode) {
        case kSaturationSoft: return saturateSoft(x);
        case kSaturationTube: return saturateTube(x);
        case kSaturationHard: return saturateHard(x);
        default: return saturateSoft(x);
    }
}
```

### 5. Lookahead Limiter

The limiter uses a delay line to "look ahead" and pre-emptively reduce gain before peaks hit.

```cpp
struct LimiterState {
    float envelopeL;
    float envelopeR;
    float gainReduction;
    float* delayBuffer;     // Stereo interleaved
    uint32_t writeIndex;
    uint32_t delaySamples;
    float attackCoeff;      // Fast attack
    float releaseCoeff;     // Slower release
};

// Threshold in normalized units (10V peak → 1.0)
const float LIMITER_THRESHOLD = 1.0f;

void processLimiter(float inL, float inR,
                   float& outL, float& outR,
                   LimiterState& state, int satMode) {

    // Write current input to delay buffer
    uint32_t writeIdx = state.writeIndex * 2;
    state.delayBuffer[writeIdx] = inL;
    state.delayBuffer[writeIdx + 1] = inR;

    // Read from delay line (lookahead)
    uint32_t readIndex = (state.writeIndex + state.delaySamples) % state.delaySamples;
    uint32_t readIdx = readIndex * 2;
    float delayedL = state.delayBuffer[readIdx];
    float delayedR = state.delayBuffer[readIdx + 1];

    // Envelope follower on input (not delayed)
    float absL = fabsf(inL);
    float absR = fabsf(inR);
    float peakIn = fmaxf(absL, absR);

    // Attack/release envelope
    float coeff = (peakIn > state.envelopeL) ? state.attackCoeff : state.releaseCoeff;
    state.envelopeL += coeff * (peakIn - state.envelopeL);

    // Calculate gain reduction needed
    float targetGain = 1.0f;
    if (state.envelopeL > LIMITER_THRESHOLD) {
        targetGain = LIMITER_THRESHOLD / state.envelopeL;
    }

    // Smooth gain changes
    state.gainReduction += 0.01f * (targetGain - state.gainReduction);

    // Apply gain to delayed signal
    float limitedL = delayedL * state.gainReduction;
    float limitedR = delayedR * state.gainReduction;

    // Apply saturation for any remaining peaks
    outL = saturate(limitedL, satMode);
    outR = saturate(limitedR, satMode);

    // Advance write index
    state.writeIndex = (state.writeIndex + 1) % state.delaySamples;
}
```

---

## Step Function Implementation

```cpp
void step(_NT_algorithm* self, float* busFrames, int numFramesBy4) {
    _seymourAlgorithm* pThis = (_seymourAlgorithm*)self;
    _seymourDTC* dtc = pThis->dtc;

    int numFrames = numFramesBy4 * 4;
    int numInputs = pThis->numInputs;

    // Get output busses
    int outLBus = pThis->v[GLOBAL_PARAM(numInputs, kParamOutputL)] - 1;
    int outRBus = pThis->v[GLOBAL_PARAM(numInputs, kParamOutputR)] - 1;
    bool replace = pThis->v[GLOBAL_PARAM(numInputs, kParamOutputMode)];

    float* outL = busFrames + outLBus * numFrames;
    float* outR = busFrames + outRBus * numFrames;

    // Get global params
    float masterTarget = pThis->v[GLOBAL_PARAM(numInputs, kParamMasterLevel)] / 100.0f;
    int satMode = pThis->v[GLOBAL_PARAM(numInputs, kParamSaturation)];

    // Update lookahead if changed
    int lookaheadMs = pThis->v[GLOBAL_PARAM(numInputs, kParamLookahead)];  // 0.1ms units
    dtc->lookaheadSamples = (NT_globals.sampleRate * lookaheadMs) / 10000;

    // Process each sample
    for (int i = 0; i < numFrames; ++i) {
        float mixL = 0.0f;
        float mixR = 0.0f;

        // Process each input channel
        for (int ch = 0; ch < numInputs; ++ch) {
            // Get input sample
            int inBus = pThis->v[CHANNEL_PARAM(ch, kParamInputBase)] - 1;
            float input = (inBus >= 0) ? busFrames[inBus * numFrames + i] : 0.0f;

            // Get feedback amount (with CV modulation)
            float fbBase = pThis->v[CHANNEL_PARAM(ch, kParamFeedbackBase)] / 100.0f;
            int fbCVBus = pThis->v[CHANNEL_PARAM(ch, kParamFeedbackCVBase)] - 1;
            float fbCVDepth = pThis->v[CHANNEL_PARAM(ch, kParamFeedbackCVDepthBase)] / 100.0f;

            if (fbCVBus >= 0) {
                float cv = busFrames[fbCVBus * numFrames + i] / 5.0f;  // ±5V → ±1
                cv = fmaxf(0.0f, fminf(1.0f, cv * 0.5f + 0.5f));  // Unipolar 0-1
                fbBase = fbBase * (1.0f - fbCVDepth) + cv * fbCVDepth;
            }

            // Smooth feedback
            pThis->feedbackSmoothed[ch] += dtc->smoothingCoeff *
                (fbBase - pThis->feedbackSmoothed[ch]);

            // Process feedback + DC block
            int dcIdx = ch * 2;
            float processed = input + pThis->feedbackState[ch] * pThis->feedbackSmoothed[ch];
            processed = dcBlock(processed,
                               pThis->dcBlockerState[dcIdx],
                               pThis->dcBlockerState[dcIdx + 1],
                               dtc->dcBlockerCoeff);
            pThis->feedbackState[ch] = processed;

            // Get pan (with CV modulation)
            float panBase = pThis->v[CHANNEL_PARAM(ch, kParamPanBase)];
            int panCVBus = pThis->v[CHANNEL_PARAM(ch, kParamPanCVBase)] - 1;
            float panCVDepth = pThis->v[CHANNEL_PARAM(ch, kParamPanCVDepthBase)] / 100.0f;

            if (panCVBus >= 0) {
                float cv = busFrames[panCVBus * numFrames + i] / 5.0f;
                panBase += cv * 100.0f * panCVDepth;
                panBase = fmaxf(-100.0f, fminf(100.0f, panBase));
            }

            // Smooth pan
            pThis->panSmoothed[ch] += dtc->smoothingCoeff *
                (panBase - pThis->panSmoothed[ch]);

            // Apply panning
            float gainL, gainR;
            equalPowerPan(pThis->panSmoothed[ch], gainL, gainR);

            mixL += processed * gainL;
            mixR += processed * gainR;
        }

        // Apply master level (smoothed)
        pThis->masterLevelSmoothed += dtc->smoothingCoeff *
            (masterTarget - pThis->masterLevelSmoothed);

        mixL *= pThis->masterLevelSmoothed;
        mixR *= pThis->masterLevelSmoothed;

        // Lookahead limiter + saturation
        float limitedL, limitedR;
        processLimiter(mixL, mixR, limitedL, limitedR,
                      dtc->envelopeL, dtc->envelopeR, dtc->gainReduction,
                      pThis->lookaheadBuffer, dtc->writeIndex,
                      dtc->lookaheadSamples, satMode);

        // Output
        if (replace) {
            outL[i] = limitedL;
            outR[i] = limitedR;
        } else {
            outL[i] += limitedL;
            outR[i] += limitedR;
        }
    }
}
```

---

## Initialization

### construct()

```cpp
_NT_algorithm* construct(const _NT_algorithmMemoryPtrs& ptrs,
                        const _NT_algorithmRequirements& req,
                        const int32_t* specifications)
{
    int numInputs = specifications[0];

    // Place main struct in SRAM
    _seymourAlgorithm* alg = new (ptrs.sram) _seymourAlgorithm();

    // Setup parameters (dynamically built)
    alg->parameters = buildParameters(numInputs);
    alg->parameterPages = buildParameterPages(numInputs);

    // Store config
    alg->numInputs = numInputs;

    // Allocate per-channel arrays in SRAM (after struct)
    uint8_t* sramPtr = ptrs.sram + sizeof(_seymourAlgorithm);
    alg->feedbackSmoothed = (float*)sramPtr; sramPtr += numInputs * sizeof(float);
    alg->panSmoothed = (float*)sramPtr; sramPtr += numInputs * sizeof(float);
    alg->feedbackState = (float*)sramPtr; sramPtr += numInputs * sizeof(float);
    alg->dcBlockerState = (float*)sramPtr; sramPtr += numInputs * 2 * sizeof(float);

    // Zero all state
    memset(alg->feedbackSmoothed, 0, numInputs * sizeof(float));
    memset(alg->panSmoothed, 0, numInputs * sizeof(float));
    memset(alg->feedbackState, 0, numInputs * sizeof(float));
    memset(alg->dcBlockerState, 0, numInputs * 2 * sizeof(float));
    alg->masterLevelSmoothed = 0.8f;

    // Setup DTC
    alg->dtc = (_seymourDTC*)ptrs.dtc;
    _seymourDTC* dtc = alg->dtc;
    memset(dtc, 0, sizeof(_seymourDTC));

    // Precompute coefficients
    float sr = NT_globals.sampleRate;
    dtc->dcBlockerCoeff = 1.0f - (2.0f * M_PI * 5.0f / sr);
    dtc->smoothingCoeff = 1.0f - expf(-2.0f * M_PI * 50.0f / sr);  // ~50Hz smoothing
    dtc->envelopeAttack = 1.0f - expf(-2.0f * M_PI * 1000.0f / sr);  // 1ms attack
    dtc->envelopeRelease = 1.0f - expf(-2.0f * M_PI * 50.0f / sr);   // ~20ms release

    // Setup DRAM (lookahead buffer)
    alg->lookaheadBuffer = (float*)ptrs.dram;
    int maxSamples = (96000 * 20) / 1000;
    memset(alg->lookaheadBuffer, 0, maxSamples * 2 * sizeof(float));

    // Default lookahead
    dtc->lookaheadSamples = (sr * 50) / 10000;  // 5ms default

    return alg;
}
```

---

## Parameter Pages

```cpp
// Built dynamically based on numInputs

_NT_parameterPages* buildParameterPages(int numInputs) {
    // Page 1-N: One page per input channel
    // Page N+1: Output/Master

    static uint8_t* channelPages[8];  // Max 8 inputs
    static uint8_t outputPage[kNumGlobalParams];
    static _NT_parameterPage pages[9];  // Max 8 + 1
    static _NT_parameterPages parameterPages;

    // Build channel pages
    for (int ch = 0; ch < numInputs; ++ch) {
        channelPages[ch] = new uint8_t[kParamsPerChannel];
        for (int p = 0; p < kParamsPerChannel; ++p) {
            channelPages[ch][p] = CHANNEL_PARAM(ch, p);
        }
        pages[ch].name = channelPageNames[ch];  // "Input 1", "Input 2", etc.
        pages[ch].numParams = kParamsPerChannel;
        pages[ch].params = channelPages[ch];
    }

    // Build output page
    for (int p = 0; p < kNumGlobalParams; ++p) {
        outputPage[p] = GLOBAL_PARAM(numInputs, p);
    }
    pages[numInputs].name = "Output";
    pages[numInputs].numParams = kNumGlobalParams;
    pages[numInputs].params = outputPage;

    parameterPages.numPages = numInputs + 1;
    parameterPages.pages = pages;

    return &parameterPages;
}
```

---

## Performance Considerations

| Concern | Mitigation |
|---------|------------|
| Per-sample loop overhead | Keep inner loop minimal, precompute coefficients |
| Parameter smoothing | Use simple one-pole filters, ~50Hz bandwidth |
| Memory access | Hot data in DTC, cold data in SRAM |
| Lookahead buffer | Up to 20ms @ 96kHz = 3840 samples stereo = ~30KB DRAM |
| tanhf calls | Consider lookup table if CPU constrained |
| CV modulation | Only read CV bus if assigned (check > 0) |

---

## Testing Checklist

- [ ] All inputs can be routed and heard in output
- [ ] Feedback at 0% = clean passthrough
- [ ] Feedback at 100% = sustained/oscillating but limited
- [ ] DC blocker prevents DC buildup over time
- [ ] Pan hard left/right produces mono output on correct channel
- [ ] CV modulation affects feedback and pan correctly
- [ ] Lookahead parameter changes smoothly without clicks
- [ ] All three saturation modes produce distinct character
- [ ] Output never exceeds ±10V regardless of input/feedback
- [ ] CPU usage acceptable at 8 inputs

---

## File Structure

```
seymour/
├── seymour.cpp          # Main plugin implementation
├── seymour_dsp.h        # DSP helper functions
├── seymour_params.h     # Parameter definitions
├── Makefile             # Build configuration
└── README.md            # Plugin documentation
```

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-20 | Neal | Initial architecture |
