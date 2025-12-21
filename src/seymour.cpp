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
 * - Lookahead limiter (0.5-20ms) with selectable saturation
 * - DC blocker in feedback path to prevent runaway
 */

#include <distingnt/api.h>
#include <math.h>
#include <string.h>
#include <new>

// ============================================================================
// CONSTANTS
// ============================================================================

enum { kMaxChannels = 8 };

// Busses are in volts (see distingNT examples).
static const float kLimiterThresholdMaxVolts = 10.0f; // least squash
static const float kLimiterThresholdMinVolts = 1.0f;  // most squash

// Maximum lookahead buffer size (20ms @ 96kHz, stereo interleaved)
static const int kMaxLookaheadSamples = (96000 * 20) / 1000;
// Maximum feedback delay buffer size (20ms @ 96kHz, per channel)
static const int kMaxFeedbackDelaySamples = (96000 * 20) / 1000;

// ============================================================================
// PARAMETER INDICES
// ============================================================================

// Global parameters (at the end)
enum GlobalParams {
    kParamOutputL,
    kParamOutputR,
    kParamOutputMode,
    kParamMasterLevel,
    kParamLookahead,
    kParamSaturation,
    kParamFeedbackDelay,
    kParamSquash,

    kNumGlobalParameters,
};

// Per-channel parameters
enum ChannelParams {
    kChParamInput,
    kChParamFeedback,
    kChParamFeedbackCV,
    kChParamFeedbackCVDepth,
    kChParamPan,
    kChParamPanCV,
    kChParamPanCVDepth,

    kNumPerChannelParameters,
};

// Saturation modes
enum SaturationMode {
    kSaturationSoft = 0,
    kSaturationTube,
    kSaturationHard,
};

// ============================================================================
// PARAMETER TEMPLATES
// ============================================================================

static const char* saturationStrings[] = { "Soft", "Tube", "Hard", NULL };

// Global parameters template
static const _NT_parameter globalParameters[] = {
    { .name = "Out L", .min = 1, .max = 28, .def = 13, .unit = kNT_unitAudioOutput, .scaling = 0, .enumStrings = NULL },
    { .name = "Out R", .min = 1, .max = 28, .def = 14, .unit = kNT_unitAudioOutput, .scaling = 0, .enumStrings = NULL },
    { .name = "Mode", .min = 0, .max = 1, .def = 1, .unit = kNT_unitOutputMode, .scaling = 0, .enumStrings = NULL },
    { .name = "Level", .min = 0, .max = 100, .def = 100, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    { .name = "Lookahead", .min = 5, .max = 200, .def = 50, .unit = kNT_unitMs, .scaling = kNT_scaling10, .enumStrings = NULL },
    { .name = "Saturation", .min = 0, .max = 2, .def = 0, .unit = kNT_unitEnum, .scaling = 0, .enumStrings = saturationStrings },
    { .name = "FB Delay", .min = 5, .max = 200, .def = 50, .unit = kNT_unitMs, .scaling = kNT_scaling10, .enumStrings = NULL },
    // 0% = least squash (higher threshold), 100% = most squash (lower threshold)
    { .name = "Squash", .min = 0, .max = 100, .def = 56, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
};

// Per-channel parameters template
static const _NT_parameter perChannelParameters[] = {
    { .name = "Input", .min = 0, .max = 28, .def = 1, .unit = kNT_unitAudioInput, .scaling = 0, .enumStrings = NULL },
    { .name = "Feedback", .min = 0, .max = 100, .def = 0, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    { .name = "FB CV", .min = 0, .max = 28, .def = 0, .unit = kNT_unitCvInput, .scaling = 0, .enumStrings = NULL },
    { .name = "FB Depth", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
    { .name = "Pan", .min = -100, .max = 100, .def = 0, .unit = kNT_unitNone, .scaling = 0, .enumStrings = NULL },
    { .name = "Pan CV", .min = 0, .max = 28, .def = 0, .unit = kNT_unitCvInput, .scaling = 0, .enumStrings = NULL },
    { .name = "Pan Depth", .min = 0, .max = 100, .def = 50, .unit = kNT_unitPercent, .scaling = 0, .enumStrings = NULL },
};

// Channel page names
static char const * const channelPageNames[] = {
    "Channel 1", "Channel 2", "Channel 3", "Channel 4",
    "Channel 5", "Channel 6", "Channel 7", "Channel 8",
};

// ============================================================================
// DSP FUNCTIONS
// ============================================================================

/**
 * DC Blocker - simple 1-pole high-pass filter
 */
static inline float dcBlock(float input, float& x1, float& y1, float R) {
    float output = input - x1 + R * y1;
    x1 = input;
    y1 = output;
    return output;
}

/**
 * Equal power panner
 */
static inline void equalPowerPan(float pan, float& gainL, float& gainR) {
    float p = (pan + 100.0f) / 200.0f;
    float angle = p * 1.5707963f;  // π/2
    gainL = cosf(angle);
    gainR = sinf(angle);
}

/**
 * Saturation functions
 */
static inline float saturateSoft(float x) {
    return tanhf(x);
}

static inline float saturateTube(float x) {
    if (x >= 0.0f) {
        return tanhf(x * 0.8f) * 1.1f;
    } else {
        return tanhf(x * 1.2f) * 0.9f;
    }
}

static inline float saturateHard(float x) {
    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    if (x > 0.8f) return 0.8f + (x - 0.8f) * 0.5f;
    if (x < -0.8f) return -0.8f + (x + 0.8f) * 0.5f;
    return x;
}

static inline float saturate(float x, int mode) {
    switch (mode) {
        case kSaturationSoft: return saturateSoft(x);
        case kSaturationTube: return saturateTube(x);
        case kSaturationHard: return saturateHard(x);
        default: return saturateSoft(x);
    }
}

// ============================================================================
// DATA STRUCTURES
// ============================================================================

/**
 * DTC memory - fast access for limiter state
 */
struct _seymourDTC {
    float envelope;
    float gainReduction;
    uint32_t writeIndex;
    uint32_t lookaheadSamples;
    uint32_t bufferSize;
    uint32_t feedbackWriteIndex;
    uint32_t feedbackDelaySamples;
    uint32_t feedbackBufferSize;
    float dcBlockerCoeff;
    float envelopeAttack;
    float envelopeRelease;
    float smoothingCoeff;
    float gainSmoothingCoeff;
};

/**
 * Main algorithm structure - contains parameter storage
 */
struct _seymourAlgorithm : public _NT_algorithm
{
    _seymourAlgorithm(int32_t numChannels_);
    ~_seymourAlgorithm() {}

    // Configuration
    int32_t numChannels;

    // Memory pointers
    _seymourDTC* dtc;
    float* lookaheadBuffer;
    float* feedbackDelayBuffer;

    // Per-channel DSP state
    float feedbackSmoothed[kMaxChannels];
    float panSmoothed[kMaxChannels];
    float feedbackState[kMaxChannels];
    float dcBlockerX1[kMaxChannels];
    float dcBlockerY1[kMaxChannels];
    float masterLevelSmoothed;

    // Parameter storage - INSIDE the struct (key difference!)
    _NT_parameter       parameterDefs[kMaxChannels * kNumPerChannelParameters + kNumGlobalParameters];
    _NT_parameterPages  pagesDefs;
    _NT_parameterPage   pageDefs[kMaxChannels + 2];  // +2 for Seymour + Routing pages
    uint8_t             channelPageParams[kMaxChannels][kNumPerChannelParameters];
    uint8_t             seymourPageParams[kNumGlobalParameters - 3];
    uint8_t             routingPageParams[3];
};

/**
 * Constructor - builds parameters dynamically based on numChannels
 */
_seymourAlgorithm::_seymourAlgorithm(int32_t numChannels_)
    : numChannels(numChannels_)
{
    // Initialize DSP state
    for (int i = 0; i < kMaxChannels; ++i) {
        feedbackSmoothed[i] = 0.0f;
        panSmoothed[i] = 0.0f;
        feedbackState[i] = 0.0f;
        dcBlockerX1[i] = 0.0f;
        dcBlockerY1[i] = 0.0f;
    }
    masterLevelSmoothed = 0.8f;

    // Build per-channel parameters
    for (int32_t ch = 0; ch < numChannels; ++ch) {
        int baseIdx = ch * kNumPerChannelParameters;

        // Copy per-channel parameter templates
        memcpy(parameterDefs + baseIdx, perChannelParameters,
               kNumPerChannelParameters * sizeof(_NT_parameter));

        // Set default input to sequential busses
        parameterDefs[baseIdx + kChParamInput].def = ch + 1;

        // Build channel page
        pageDefs[ch].name = channelPageNames[ch];
        pageDefs[ch].numParams = kNumPerChannelParameters;
        pageDefs[ch].params = channelPageParams[ch];

        for (int p = 0; p < kNumPerChannelParameters; ++p) {
            channelPageParams[ch][p] = baseIdx + p;
        }
    }

    // Add global parameters at the end
    int globalBase = numChannels * kNumPerChannelParameters;
    memcpy(parameterDefs + globalBase, globalParameters,
           kNumGlobalParameters * sizeof(_NT_parameter));

    // Build Seymour (algorithm-global) page
    pageDefs[numChannels].name = "Seymour";
    pageDefs[numChannels].numParams = ARRAY_SIZE(seymourPageParams);
    pageDefs[numChannels].params = seymourPageParams;
    seymourPageParams[0] = globalBase + kParamMasterLevel;
    seymourPageParams[1] = globalBase + kParamLookahead;
    seymourPageParams[2] = globalBase + kParamSaturation;
    seymourPageParams[3] = globalBase + kParamFeedbackDelay;
    seymourPageParams[4] = globalBase + kParamSquash;

    // Build routing page (I/O and output mode)
    pageDefs[numChannels + 1].name = "Routing";
    pageDefs[numChannels + 1].numParams = ARRAY_SIZE(routingPageParams);
    pageDefs[numChannels + 1].params = routingPageParams;
    routingPageParams[0] = globalBase + kParamOutputL;
    routingPageParams[1] = globalBase + kParamOutputR;
    routingPageParams[2] = globalBase + kParamOutputMode;

    // Setup pages structure
    pagesDefs.numPages = numChannels + 2;
    pagesDefs.pages = pageDefs;

    // Set _NT_algorithm members
    parameters = parameterDefs;
    parameterPages = &pagesDefs;
}

// ============================================================================
// SPECIFICATIONS
// ============================================================================

static const _NT_specification specifications[] = {
    { .name = "Inputs", .min = 1, .max = kMaxChannels, .def = 2, .type = kNT_typeGeneric },
};

// ============================================================================
// FACTORY FUNCTIONS
// ============================================================================

void calculateRequirements(_NT_algorithmRequirements& req, const int32_t* specs) {
    int32_t numChannels = specs[0];

    req.numParameters = numChannels * kNumPerChannelParameters + kNumGlobalParameters;
    req.sram = sizeof(_seymourAlgorithm);
    req.dram = (kMaxLookaheadSamples * 2 + kMaxFeedbackDelaySamples * kMaxChannels) * sizeof(float);
    req.dtc = sizeof(_seymourDTC);
    req.itc = 0;
}

_NT_algorithm* construct(const _NT_algorithmMemoryPtrs& ptrs,
                         const _NT_algorithmRequirements& req,
                         const int32_t* specs) {
    int32_t numChannels = specs[0];

    // Create algorithm with constructor that builds parameters
    _seymourAlgorithm* alg = new (ptrs.sram) _seymourAlgorithm(numChannels);

    // Setup DTC
    alg->dtc = (_seymourDTC*)ptrs.dtc;
    _seymourDTC* dtc = alg->dtc;

    dtc->envelope = 0.0f;
    dtc->gainReduction = 1.0f;
    dtc->writeIndex = 0;
    dtc->bufferSize = kMaxLookaheadSamples;
    dtc->feedbackWriteIndex = 0;
    dtc->feedbackBufferSize = kMaxFeedbackDelaySamples;

    // Precompute coefficients
    float sr = NT_globals.sampleRate;
    dtc->dcBlockerCoeff = 1.0f - (6.28318f * 5.0f / sr);
    dtc->smoothingCoeff = 1.0f - expf(-6.28318f * 50.0f / sr);
    dtc->envelopeAttack = 1.0f - expf(-6.28318f * 1000.0f / sr);
    dtc->envelopeRelease = 1.0f - expf(-6.28318f * 50.0f / sr);
    dtc->gainSmoothingCoeff = 1.0f - expf(-6.28318f * 30.0f / sr);
    dtc->lookaheadSamples = (uint32_t)(sr * 0.005f);  // 5ms default
    dtc->feedbackDelaySamples = (uint32_t)(sr * 0.005f);  // 5ms default

    // Setup lookahead buffer
    alg->lookaheadBuffer = (float*)ptrs.dram;
    memset(alg->lookaheadBuffer, 0, kMaxLookaheadSamples * 2 * sizeof(float));
    alg->feedbackDelayBuffer = (float*)(ptrs.dram) + kMaxLookaheadSamples * 2;
    memset(alg->feedbackDelayBuffer, 0, kMaxFeedbackDelaySamples * kMaxChannels * sizeof(float));

    return alg;
}

int parameterUiPrefix(_NT_algorithm* self, int p, char* buff) {
    _seymourAlgorithm* pThis = (_seymourAlgorithm*)self;
    int globalBase = pThis->numChannels * kNumPerChannelParameters;

    // Only add prefix for per-channel parameters
    if (p < globalBase) {
        int ch = p / kNumPerChannelParameters;
        int len = NT_intToString(buff, 1 + ch);
        buff[len++] = ':';
        buff[len] = 0;
        return len;
    }
    return 0;
}

void parameterChanged(_NT_algorithm* self, int p) {
    _seymourAlgorithm* pThis = (_seymourAlgorithm*)self;
    _seymourDTC* dtc = pThis->dtc;
    int globalBase = pThis->numChannels * kNumPerChannelParameters;

    // Check if lookahead changed
    if (p == globalBase + kParamLookahead) {
        float lookaheadMs = pThis->v[p] / 10.0f;
        uint32_t samples = (uint32_t)(NT_globals.sampleRate * lookaheadMs / 1000.0f);
        if (samples > dtc->bufferSize) samples = dtc->bufferSize;
        if (samples < 1) samples = 1;
        dtc->lookaheadSamples = samples;
    } else if (p == globalBase + kParamFeedbackDelay) {
        float delayMs = pThis->v[p] / 10.0f;
        uint32_t samples = (uint32_t)(NT_globals.sampleRate * delayMs / 1000.0f);
        if (samples >= dtc->feedbackBufferSize) samples = dtc->feedbackBufferSize - 1;
        if (samples < 1) samples = 1;
        dtc->feedbackDelaySamples = samples;
    }
}

void step(_NT_algorithm* self, float* busFrames, int numFramesBy4) {
    _seymourAlgorithm* pThis = (_seymourAlgorithm*)self;
    _seymourDTC* dtc = pThis->dtc;

    int numFrames = numFramesBy4 * 4;
    int32_t numChannels = pThis->numChannels;
    int globalBase = numChannels * kNumPerChannelParameters;

    // Get output busses
    int outLBus = pThis->v[globalBase + kParamOutputL] - 1;
    int outRBus = pThis->v[globalBase + kParamOutputR] - 1;
    bool replace = pThis->v[globalBase + kParamOutputMode];

    float* outL = busFrames + outLBus * numFrames;
    float* outR = busFrames + outRBus * numFrames;

    // Get global parameters
    float masterTarget = pThis->v[globalBase + kParamMasterLevel] / 100.0f;
    int satMode = pThis->v[globalBase + kParamSaturation];
    float squash = pThis->v[globalBase + kParamSquash] / 100.0f;
    if (squash < 0.0f) squash = 0.0f;
    if (squash > 1.0f) squash = 1.0f;
    float limiterThresholdVolts =
        kLimiterThresholdMaxVolts - (kLimiterThresholdMaxVolts - kLimiterThresholdMinVolts) * squash;

    // Coefficients
    float dcCoeff = dtc->dcBlockerCoeff;
    float smoothCoeff = dtc->smoothingCoeff;
    float gainSmoothCoeff = dtc->gainSmoothingCoeff;
    float attackCoeff = dtc->envelopeAttack;
    float releaseCoeff = dtc->envelopeRelease;

    // Lookahead buffer
    float* delayBuf = pThis->lookaheadBuffer;
    uint32_t bufSize = dtc->bufferSize;
    uint32_t lookahead = dtc->lookaheadSamples;

    // Feedback delay buffer (shared write index, per-channel lanes)
    float* feedbackDelayBuf = pThis->feedbackDelayBuffer;
    uint32_t fbBufSize = dtc->feedbackBufferSize;
    uint32_t fbDelay = dtc->feedbackDelaySamples;

    // Process each sample
    for (int i = 0; i < numFrames; ++i) {
        float mixL = 0.0f;
        float mixR = 0.0f;

        uint32_t fbWriteIdx = dtc->feedbackWriteIndex;
        uint32_t fbReadIdx = (fbWriteIdx + fbBufSize - fbDelay) % fbBufSize;

        // Process each channel
        for (int32_t ch = 0; ch < numChannels; ++ch) {
            int paramBase = ch * kNumPerChannelParameters;

            // Get input
            int inBus = pThis->v[paramBase + kChParamInput] - 1;
            float input = (inBus >= 0) ? busFrames[inBus * numFrames + i] : 0.0f;

            // Get feedback with CV
            float fbBase = pThis->v[paramBase + kChParamFeedback] / 100.0f;
            int fbCVBus = pThis->v[paramBase + kChParamFeedbackCV] - 1;
            float fbCVDepth = pThis->v[paramBase + kChParamFeedbackCVDepth] / 100.0f;

            if (fbCVBus >= 0) {
                float cv = busFrames[fbCVBus * numFrames + i] / 5.0f;
                cv = cv * 0.5f + 0.5f;
                if (cv < 0.0f) cv = 0.0f;
                if (cv > 1.0f) cv = 1.0f;
                fbBase = fbBase * (1.0f - fbCVDepth) + cv * fbCVDepth;
            }

            // Smooth feedback
            pThis->feedbackSmoothed[ch] += smoothCoeff * (fbBase - pThis->feedbackSmoothed[ch]);

            // Apply feedback (DC-blocked in the feedback path only)
            float feedbackTap = feedbackDelayBuf[fbReadIdx * kMaxChannels + ch];
            float feedbackFiltered = dcBlock(feedbackTap, pThis->dcBlockerX1[ch], pThis->dcBlockerY1[ch], dcCoeff);
            float processed = input + feedbackFiltered * pThis->feedbackSmoothed[ch];
            feedbackDelayBuf[fbWriteIdx * kMaxChannels + ch] = processed;

            // Get pan with CV
            float panBase = (float)pThis->v[paramBase + kChParamPan];
            int panCVBus = pThis->v[paramBase + kChParamPanCV] - 1;
            float panCVDepth = pThis->v[paramBase + kChParamPanCVDepth] / 100.0f;

            if (panCVBus >= 0) {
                float cv = busFrames[panCVBus * numFrames + i] / 5.0f;
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

        dtc->feedbackWriteIndex = (dtc->feedbackWriteIndex + 1) % fbBufSize;

        // Master level
        pThis->masterLevelSmoothed += smoothCoeff * (masterTarget - pThis->masterLevelSmoothed);
        mixL *= pThis->masterLevelSmoothed;
        mixR *= pThis->masterLevelSmoothed;

        // Lookahead limiter
        uint32_t writeIdx = dtc->writeIndex;
        delayBuf[writeIdx * 2] = mixL;
        delayBuf[writeIdx * 2 + 1] = mixR;

        uint32_t readIdx = (writeIdx + bufSize - lookahead) % bufSize;
        float delayedL = delayBuf[readIdx * 2];
        float delayedR = delayBuf[readIdx * 2 + 1];

        float absL = mixL > 0 ? mixL : -mixL;
        float absR = mixR > 0 ? mixR : -mixR;
        float peakIn = absL > absR ? absL : absR;

        float envCoeff = (peakIn > dtc->envelope) ? attackCoeff : releaseCoeff;
        dtc->envelope += envCoeff * (peakIn - dtc->envelope);

        float targetGain = 1.0f;
        if (dtc->envelope > limiterThresholdVolts) {
            targetGain = limiterThresholdVolts / dtc->envelope;
        }

        dtc->gainReduction += gainSmoothCoeff * (targetGain - dtc->gainReduction);

        float limitedL = delayedL * dtc->gainReduction;
        float limitedR = delayedR * dtc->gainReduction;

        float finalL = limitedL;
        float finalR = limitedR;
        if (dtc->gainReduction < 0.9999f || dtc->envelope > limiterThresholdVolts) {
            float normL = limitedL / limiterThresholdVolts;
            float normR = limitedR / limiterThresholdVolts;
            finalL = saturate(normL, satMode) * limiterThresholdVolts;
            finalR = saturate(normR, satMode) * limiterThresholdVolts;
        }

        dtc->writeIndex = (writeIdx + 1) % bufSize;

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
// FACTORY
// ============================================================================

static const _NT_factory factory = {
    .guid = NT_MULTICHAR('T', 'h', 'S', 'y'),  // Thorinside + Seymour
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
// ENTRY POINT
// ============================================================================

uintptr_t pluginEntry(_NT_selector selector, uint32_t data) {
    switch (selector) {
        case kNT_selector_version:
            return kNT_apiVersionCurrent;
        case kNT_selector_numFactories:
            return 1;
        case kNT_selector_factoryInfo:
            return (uintptr_t)((data == 0) ? &factory : NULL);
    }
    return 0;
}
