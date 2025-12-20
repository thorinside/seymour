/*
MIT License

Copyright (c) 2024 Neal Sanche

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/**
 * Seymour - Feedback Mixer with Safety Limiter
 *
 * "Feed me, Seymour!" - A multi-input feedback mixer that won't hurt you.
 *
 * Features:
 * - Configurable 1-8 mono inputs via specification
 * - Per-input feedback control with CV modulation
 * - Per-input stereo panning with CV modulation
 * - Lookahead limiter with selectable saturation (Soft/Tube/Hard)
 * - DC blocker in feedback path to prevent runaway
 */

#include <distingnt/api.h>
#include <math.h>
#include <string.h>
#include <new>

// ============================================================================
// CONSTANTS
// ============================================================================

static const int kMaxInputs = 8;
static const int kParamsPerChannel = 7;
static const int kNumGlobalParams = 6;

// Limiter threshold (10V peak normalized to 1.0)
static const float kLimiterThreshold = 1.0f;

// Maximum lookahead buffer size (20ms @ 96kHz, stereo interleaved)
static const int kMaxLookaheadSamples = (96000 * 20) / 1000;  // 1920 samples

// ============================================================================
// PARAMETER INDICES
// ============================================================================

// Per-channel parameter offsets
enum ChannelParams {
    kChParamInput = 0,
    kChParamFeedback,
    kChParamFeedbackCV,
    kChParamFeedbackCVDepth,
    kChParamPan,
    kChParamPanCV,
    kChParamPanCVDepth,
};

// Global parameter offsets (added after all channel params)
enum GlobalParams {
    kGlobalParamOutputL = 0,
    kGlobalParamOutputR,
    kGlobalParamOutputMode,
    kGlobalParamMasterLevel,
    kGlobalParamLookahead,
    kGlobalParamSaturation,
};

// Saturation modes
enum SaturationMode {
    kSaturationSoft = 0,
    kSaturationTube,
    kSaturationHard,
};

// Helper macros
#define CHANNEL_PARAM(ch, param) ((ch) * kParamsPerChannel + (param))
#define GLOBAL_PARAM_IDX(numInputs, param) ((numInputs) * kParamsPerChannel + (param))

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * DTC (Data Tightly Coupled) memory - fast access for hot path
 */
struct _seymourDTC {
    // Limiter state
    float envelope;
    float gainReduction;

    // Lookahead buffer state
    uint32_t writeIndex;
    uint32_t lookaheadSamples;
    uint32_t bufferSize;  // Actual buffer size in samples

    // Precomputed coefficients
    float dcBlockerCoeff;
    float envelopeAttack;
    float envelopeRelease;
    float smoothingCoeff;
    float gainSmoothingCoeff;
};

/**
 * Main algorithm structure
 */
struct _seymourAlgorithm : public _NT_algorithm {
    _seymourAlgorithm() {}
    ~_seymourAlgorithm() {}

    // Configuration
    int numInputs;

    // Memory pointers
    _seymourDTC* dtc;
    float* lookaheadBuffer;  // DRAM - stereo interleaved [L0,R0,L1,R1,...]

    // Per-channel state (in SRAM after this struct)
    float* feedbackSmoothed;    // [numInputs]
    float* panSmoothed;         // [numInputs]
    float* feedbackState;       // [numInputs] - previous output for feedback
    float* dcBlockerX1;         // [numInputs] - DC blocker x[n-1]
    float* dcBlockerY1;         // [numInputs] - DC blocker y[n-1]

    // Master level smoothing
    float masterLevelSmoothed;
};

// ============================================================================
// SPECIFICATIONS
// ============================================================================

static const _NT_specification specifications[] = {
    {
        .name = "Inputs",
        .min = 1,
        .max = 8,
        .def = 2,
        .type = kNT_typeGeneric
    },
};

// ============================================================================
// STATIC PARAMETER STORAGE
// ============================================================================

// We need to build parameters dynamically based on numInputs
// These are static storage for the maximum case
static _NT_parameter allParameters[kMaxInputs * kParamsPerChannel + kNumGlobalParams];
static uint8_t channelPageParams[kMaxInputs][kParamsPerChannel];
static uint8_t outputPageParams[kNumGlobalParams];
static _NT_parameterPage allPages[kMaxInputs + 1];
static _NT_parameterPages parameterPages;

static const char* channelPageNames[kMaxInputs] = {
    "Input 1", "Input 2", "Input 3", "Input 4",
    "Input 5", "Input 6", "Input 7", "Input 8"
};

static const char* saturationStrings[] = { "Soft", "Tube", "Hard", NULL };

// ============================================================================
// DSP FUNCTIONS
// ============================================================================

/**
 * DC Blocker - simple 1-pole high-pass filter
 * y[n] = x[n] - x[n-1] + R * y[n-1]
 */
static inline float dcBlock(float input, float& x1, float& y1, float R) {
    float output = input - x1 + R * y1;
    x1 = input;
    y1 = output;
    return output;
}

/**
 * Equal power panner
 * pan: -100 to +100
 */
static inline void equalPowerPan(float pan, float& gainL, float& gainR) {
    float p = (pan + 100.0f) / 200.0f;  // 0 to 1
    float angle = p * 1.5707963f;        // 0 to π/2
    gainL = cosf(angle);
    gainR = sinf(angle);
}

/**
 * Soft saturation (tanh) - symmetric, smooth
 */
static inline float saturateSoft(float x) {
    return tanhf(x);
}

/**
 * Tube saturation - asymmetric, even harmonics
 */
static inline float saturateTube(float x) {
    if (x >= 0.0f) {
        return tanhf(x * 0.8f) * 1.1f;
    } else {
        return tanhf(x * 1.2f) * 0.9f;
    }
}

/**
 * Hard saturation - aggressive with soft knee
 */
static inline float saturateHard(float x) {
    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    if (x > 0.8f) return 0.8f + (x - 0.8f) * 0.5f;
    if (x < -0.8f) return -0.8f + (x + 0.8f) * 0.5f;
    return x;
}

/**
 * Saturation dispatcher
 */
static inline float saturate(float x, int mode) {
    switch (mode) {
        case kSaturationSoft: return saturateSoft(x);
        case kSaturationTube: return saturateTube(x);
        case kSaturationHard: return saturateHard(x);
        default: return saturateSoft(x);
    }
}

// ============================================================================
// PARAMETER BUILDING
// ============================================================================

/**
 * Build the parameter array based on numInputs
 */
static void buildParameters(int numInputs) {
    int idx = 0;

    // Per-channel parameters
    for (int ch = 0; ch < numInputs; ++ch) {
        // Input bus selector
        allParameters[idx++] = {
            .name = "Input",
            .min = 0,
            .max = 28,
            .def = (int16_t)(ch + 1),  // Default to sequential inputs
            .unit = kNT_unitAudioInput,
            .scaling = 0,
            .enumStrings = NULL
        };

        // Feedback amount
        allParameters[idx++] = {
            .name = "Feedback",
            .min = 0,
            .max = 100,
            .def = 0,
            .unit = kNT_unitPercent,
            .scaling = 0,
            .enumStrings = NULL
        };

        // Feedback CV input
        allParameters[idx++] = {
            .name = "FB CV",
            .min = 0,
            .max = 28,
            .def = 0,
            .unit = kNT_unitAudioInput,
            .scaling = 0,
            .enumStrings = NULL
        };

        // Feedback CV depth
        allParameters[idx++] = {
            .name = "FB Depth",
            .min = 0,
            .max = 100,
            .def = 50,
            .unit = kNT_unitPercent,
            .scaling = 0,
            .enumStrings = NULL
        };

        // Pan
        allParameters[idx++] = {
            .name = "Pan",
            .min = -100,
            .max = 100,
            .def = 0,
            .unit = kNT_unitNone,
            .scaling = 0,
            .enumStrings = NULL
        };

        // Pan CV input
        allParameters[idx++] = {
            .name = "Pan CV",
            .min = 0,
            .max = 28,
            .def = 0,
            .unit = kNT_unitAudioInput,
            .scaling = 0,
            .enumStrings = NULL
        };

        // Pan CV depth
        allParameters[idx++] = {
            .name = "Pan Depth",
            .min = 0,
            .max = 100,
            .def = 50,
            .unit = kNT_unitPercent,
            .scaling = 0,
            .enumStrings = NULL
        };
    }

    // Global parameters

    // Output L
    allParameters[idx++] = {
        .name = "Out L",
        .min = 1,
        .max = 28,
        .def = 13,
        .unit = kNT_unitAudioOutput,
        .scaling = 0,
        .enumStrings = NULL
    };

    // Output R
    allParameters[idx++] = {
        .name = "Out R",
        .min = 1,
        .max = 28,
        .def = 14,
        .unit = kNT_unitAudioOutput,
        .scaling = 0,
        .enumStrings = NULL
    };

    // Output Mode
    allParameters[idx++] = {
        .name = "Mode",
        .min = 0,
        .max = 1,
        .def = 1,
        .unit = kNT_unitOutputMode,
        .scaling = 0,
        .enumStrings = NULL
    };

    // Master Level
    allParameters[idx++] = {
        .name = "Level",
        .min = 0,
        .max = 100,
        .def = 80,
        .unit = kNT_unitPercent,
        .scaling = 0,
        .enumStrings = NULL
    };

    // Lookahead (0.5ms to 20ms, stored as 5-200 with scaling/10)
    allParameters[idx++] = {
        .name = "Lookahead",
        .min = 5,
        .max = 200,
        .def = 50,
        .unit = kNT_unitMs,
        .scaling = kNT_scaling10,
        .enumStrings = NULL
    };

    // Saturation type
    allParameters[idx++] = {
        .name = "Saturation",
        .min = 0,
        .max = 2,
        .def = 0,
        .unit = kNT_unitEnum,
        .scaling = 0,
        .enumStrings = saturationStrings
    };
}

/**
 * Build parameter pages based on numInputs
 */
static void buildParameterPages(int numInputs) {
    // Build channel pages
    for (int ch = 0; ch < numInputs; ++ch) {
        for (int p = 0; p < kParamsPerChannel; ++p) {
            channelPageParams[ch][p] = CHANNEL_PARAM(ch, p);
        }
        allPages[ch].name = channelPageNames[ch];
        allPages[ch].numParams = kParamsPerChannel;
        allPages[ch].params = channelPageParams[ch];
    }

    // Build output page
    for (int p = 0; p < kNumGlobalParams; ++p) {
        outputPageParams[p] = GLOBAL_PARAM_IDX(numInputs, p);
    }
    allPages[numInputs].name = "Output";
    allPages[numInputs].numParams = kNumGlobalParams;
    allPages[numInputs].params = outputPageParams;

    // Setup parameter pages struct
    parameterPages.numPages = numInputs + 1;
    parameterPages.pages = allPages;
}

// ============================================================================
// FACTORY FUNCTIONS
// ============================================================================

/**
 * Calculate memory requirements
 */
void calculateRequirements(_NT_algorithmRequirements& req, const int32_t* specs) {
    int numInputs = specs[0];
    int numParams = numInputs * kParamsPerChannel + kNumGlobalParams;

    // SRAM: algorithm struct + per-channel arrays
    // feedbackSmoothed, panSmoothed, feedbackState, dcBlockerX1, dcBlockerY1
    int perChannelArrays = numInputs * sizeof(float) * 5;

    req.numParameters = numParams;
    req.sram = sizeof(_seymourAlgorithm) + perChannelArrays;
    req.dram = kMaxLookaheadSamples * 2 * sizeof(float);  // Stereo interleaved
    req.dtc = sizeof(_seymourDTC);
    req.itc = 0;
}

/**
 * Construct algorithm instance
 */
_NT_algorithm* construct(const _NT_algorithmMemoryPtrs& ptrs,
                        const _NT_algorithmRequirements& req,
                        const int32_t* specs) {
    int numInputs = specs[0];

    // Build parameters and pages for this instance
    buildParameters(numInputs);
    buildParameterPages(numInputs);

    // Create algorithm in SRAM
    _seymourAlgorithm* alg = new (ptrs.sram) _seymourAlgorithm();

    alg->parameters = allParameters;
    alg->parameterPages = &parameterPages;
    alg->numInputs = numInputs;

    // Allocate per-channel arrays after struct
    uint8_t* sramPtr = ptrs.sram + sizeof(_seymourAlgorithm);

    alg->feedbackSmoothed = (float*)sramPtr;
    sramPtr += numInputs * sizeof(float);

    alg->panSmoothed = (float*)sramPtr;
    sramPtr += numInputs * sizeof(float);

    alg->feedbackState = (float*)sramPtr;
    sramPtr += numInputs * sizeof(float);

    alg->dcBlockerX1 = (float*)sramPtr;
    sramPtr += numInputs * sizeof(float);

    alg->dcBlockerY1 = (float*)sramPtr;
    sramPtr += numInputs * sizeof(float);

    // Zero all per-channel state
    for (int ch = 0; ch < numInputs; ++ch) {
        alg->feedbackSmoothed[ch] = 0.0f;
        alg->panSmoothed[ch] = 0.0f;
        alg->feedbackState[ch] = 0.0f;
        alg->dcBlockerX1[ch] = 0.0f;
        alg->dcBlockerY1[ch] = 0.0f;
    }
    alg->masterLevelSmoothed = 0.8f;

    // Setup DTC
    alg->dtc = (_seymourDTC*)ptrs.dtc;
    _seymourDTC* dtc = alg->dtc;

    dtc->envelope = 0.0f;
    dtc->gainReduction = 1.0f;
    dtc->writeIndex = 0;

    // Precompute coefficients based on sample rate
    float sr = NT_globals.sampleRate;

    // DC blocker: R = 1 - (2*pi*fc/sr), fc ~= 5Hz
    dtc->dcBlockerCoeff = 1.0f - (6.28318f * 5.0f / sr);

    // Parameter smoothing: ~50Hz
    dtc->smoothingCoeff = 1.0f - expf(-6.28318f * 50.0f / sr);

    // Limiter envelope: fast attack (~1ms), slower release (~20ms)
    dtc->envelopeAttack = 1.0f - expf(-6.28318f * 1000.0f / sr);
    dtc->envelopeRelease = 1.0f - expf(-6.28318f * 50.0f / sr);

    // Gain smoothing (slightly slower than parameter smoothing)
    dtc->gainSmoothingCoeff = 1.0f - expf(-6.28318f * 30.0f / sr);

    // Default lookahead: 5ms
    dtc->lookaheadSamples = (uint32_t)(sr * 0.005f);
    dtc->bufferSize = kMaxLookaheadSamples;

    // Setup DRAM (lookahead buffer)
    alg->lookaheadBuffer = (float*)ptrs.dram;
    memset(alg->lookaheadBuffer, 0, kMaxLookaheadSamples * 2 * sizeof(float));

    return alg;
}

/**
 * Parameter UI prefix callback - adds "N:" prefix to channel parameters
 */
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

/**
 * Parameter changed callback
 */
void parameterChanged(_NT_algorithm* self, int p) {
    _seymourAlgorithm* pThis = (_seymourAlgorithm*)self;
    _seymourDTC* dtc = pThis->dtc;

    int globalBase = pThis->numInputs * kParamsPerChannel;

    // Check if it's the lookahead parameter
    if (p == globalBase + kGlobalParamLookahead) {
        // Lookahead is stored as 5-200 (0.5ms to 20ms with scaling/10)
        float lookaheadMs = pThis->v[p] / 10.0f;
        uint32_t samples = (uint32_t)(NT_globals.sampleRate * lookaheadMs / 1000.0f);

        // Clamp to buffer size
        if (samples > dtc->bufferSize) samples = dtc->bufferSize;
        if (samples < 1) samples = 1;

        dtc->lookaheadSamples = samples;
    }
}

/**
 * Main audio processing
 */
void step(_NT_algorithm* self, float* busFrames, int numFramesBy4) {
    _seymourAlgorithm* pThis = (_seymourAlgorithm*)self;
    _seymourDTC* dtc = pThis->dtc;

    int numFrames = numFramesBy4 * 4;
    int numInputs = pThis->numInputs;
    int globalBase = numInputs * kParamsPerChannel;

    // Get output busses (1-based in params, 0-based for array access)
    int outLBus = pThis->v[globalBase + kGlobalParamOutputL] - 1;
    int outRBus = pThis->v[globalBase + kGlobalParamOutputR] - 1;
    bool replace = pThis->v[globalBase + kGlobalParamOutputMode];

    float* outL = busFrames + outLBus * numFrames;
    float* outR = busFrames + outRBus * numFrames;

    // Get global parameters
    float masterTarget = pThis->v[globalBase + kGlobalParamMasterLevel] / 100.0f;
    int satMode = pThis->v[globalBase + kGlobalParamSaturation];

    // Lookahead buffer
    float* delayBuf = pThis->lookaheadBuffer;
    uint32_t bufSize = dtc->bufferSize;
    uint32_t lookahead = dtc->lookaheadSamples;

    // Coefficients
    float dcCoeff = dtc->dcBlockerCoeff;
    float smoothCoeff = dtc->smoothingCoeff;
    float gainSmoothCoeff = dtc->gainSmoothingCoeff;
    float attackCoeff = dtc->envelopeAttack;
    float releaseCoeff = dtc->envelopeRelease;

    // Process each sample
    for (int i = 0; i < numFrames; ++i) {
        float mixL = 0.0f;
        float mixR = 0.0f;

        // Process each input channel
        for (int ch = 0; ch < numInputs; ++ch) {
            int paramBase = ch * kParamsPerChannel;

            // Get input sample
            int inBus = pThis->v[paramBase + kChParamInput] - 1;
            float input = (inBus >= 0) ? busFrames[inBus * numFrames + i] : 0.0f;

            // Get feedback amount with CV modulation
            float fbBase = pThis->v[paramBase + kChParamFeedback] / 100.0f;
            int fbCVBus = pThis->v[paramBase + kChParamFeedbackCV] - 1;
            float fbCVDepth = pThis->v[paramBase + kChParamFeedbackCVDepth] / 100.0f;

            if (fbCVBus >= 0) {
                float cv = busFrames[fbCVBus * numFrames + i] / 5.0f;  // ±5V → ±1
                cv = cv * 0.5f + 0.5f;  // Convert to 0-1
                if (cv < 0.0f) cv = 0.0f;
                if (cv > 1.0f) cv = 1.0f;
                fbBase = fbBase * (1.0f - fbCVDepth) + cv * fbCVDepth;
            }

            // Smooth feedback
            pThis->feedbackSmoothed[ch] += smoothCoeff * (fbBase - pThis->feedbackSmoothed[ch]);
            float feedback = pThis->feedbackSmoothed[ch];

            // Apply feedback + DC block
            float feedbackSample = pThis->feedbackState[ch] * feedback;
            float mixed = input + feedbackSample;
            float processed = dcBlock(mixed,
                                      pThis->dcBlockerX1[ch],
                                      pThis->dcBlockerY1[ch],
                                      dcCoeff);
            pThis->feedbackState[ch] = processed;

            // Get pan with CV modulation
            float panBase = (float)pThis->v[paramBase + kChParamPan];
            int panCVBus = pThis->v[paramBase + kChParamPanCV] - 1;
            float panCVDepth = pThis->v[paramBase + kChParamPanCVDepth] / 100.0f;

            if (panCVBus >= 0) {
                float cv = busFrames[panCVBus * numFrames + i] / 5.0f;  // ±5V → ±1
                panBase += cv * 100.0f * panCVDepth;
                if (panBase < -100.0f) panBase = -100.0f;
                if (panBase > 100.0f) panBase = 100.0f;
            }

            // Smooth pan
            pThis->panSmoothed[ch] += smoothCoeff * (panBase - pThis->panSmoothed[ch]);

            // Apply panning
            float gainL, gainR;
            equalPowerPan(pThis->panSmoothed[ch], gainL, gainR);

            mixL += processed * gainL;
            mixR += processed * gainR;
        }

        // Apply master level (smoothed)
        pThis->masterLevelSmoothed += smoothCoeff * (masterTarget - pThis->masterLevelSmoothed);
        mixL *= pThis->masterLevelSmoothed;
        mixR *= pThis->masterLevelSmoothed;

        // === Lookahead Limiter ===

        // Write current sample to delay buffer
        uint32_t writeIdx = dtc->writeIndex;
        delayBuf[writeIdx * 2] = mixL;
        delayBuf[writeIdx * 2 + 1] = mixR;

        // Read delayed sample
        uint32_t readIdx = (writeIdx + bufSize - lookahead) % bufSize;
        float delayedL = delayBuf[readIdx * 2];
        float delayedR = delayBuf[readIdx * 2 + 1];

        // Envelope follower on input (not delayed) - use max of L/R
        float absL = mixL > 0 ? mixL : -mixL;
        float absR = mixR > 0 ? mixR : -mixR;
        float peakIn = absL > absR ? absL : absR;

        // Attack/release envelope
        float envCoeff = (peakIn > dtc->envelope) ? attackCoeff : releaseCoeff;
        dtc->envelope += envCoeff * (peakIn - dtc->envelope);

        // Calculate target gain reduction
        float targetGain = 1.0f;
        if (dtc->envelope > kLimiterThreshold) {
            targetGain = kLimiterThreshold / dtc->envelope;
        }

        // Smooth gain changes
        dtc->gainReduction += gainSmoothCoeff * (targetGain - dtc->gainReduction);

        // Apply gain reduction to delayed signal
        float limitedL = delayedL * dtc->gainReduction;
        float limitedR = delayedR * dtc->gainReduction;

        // Apply saturation for any remaining peaks
        float finalL = saturate(limitedL, satMode);
        float finalR = saturate(limitedR, satMode);

        // Advance write index
        dtc->writeIndex = (writeIdx + 1) % bufSize;

        // Output
        if (replace) {
            outL[i] = finalL;
            outR[i] = finalR;
        } else {
            outL[i] += finalL;
            outR[i] += finalR;
        }
    }
}

// ============================================================================
// FACTORY DEFINITION
// ============================================================================

static const _NT_factory factory = {
    .guid = NT_MULTICHAR('N', 's', 'S', 'y'),  // Nealsanche + Seymour
    .name = "Seymour",
    .description = "Feedback mixer with safety limiter",
    .numSpecifications = ARRAY_SIZE(specifications),
    .specifications = specifications,
    .calculateStaticRequirements = NULL,
    .initialise = NULL,
    .calculateRequirements = calculateRequirements,
    .construct = construct,
    .parameterChanged = parameterChanged,
    .step = step,
    .draw = NULL,
    .midiRealtime = NULL,
    .midiMessage = NULL,
    .tags = kNT_tagEffect | kNT_tagUtility,
    .hasCustomUi = NULL,
    .customUi = NULL,
    .setupUi = NULL,
    .serialise = NULL,
    .deserialise = NULL,
    .midiSysEx = NULL,
    .parameterUiPrefix = parameterUiPrefix,
};

// ============================================================================
// PLUGIN ENTRY POINT
// ============================================================================

uintptr_t pluginEntry(_NT_selector selector, uint32_t data) {
    switch (selector) {
        case kNT_selector_version:
            return kNT_apiVersion9;
        case kNT_selector_numFactories:
            return 1;
        case kNT_selector_factoryInfo:
            return (uintptr_t)((data == 0) ? &factory : NULL);
    }
    return 0;
}
