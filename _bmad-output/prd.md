---
stepsCompleted: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
inputDocuments: []
documentCounts:
  briefs: 0
  research: 0
  brainstorming: 0
  projectDocs: 0
workflowType: 'prd'
lastStep: 11
project_name: 'seymour'
user_name: 'Neal'
date: '2025-12-20'
---

# Product Requirements Document - Seymour

**Author:** Neal
**Date:** 2025-12-20
**Version:** 1.0

---

## Executive Summary

**Seymour** is a disting NT C++ plugin that provides a multi-input feedback mixer with built-in safety limiting. It eliminates the need to patch together separate feedback send/receive algorithms by providing self-contained, per-input feedback loops with a brick wall limiter at the output.

The name "Seymour" is a play on "Feed me, Seymour" — you feed signals in, and those signals feed back based on configurable feedback parameters.

**Target Users:** Sound design explorers frustrated by patching complexity who want to experiment with feedback-based sound design safely.

### What Makes This Special

**One plugin, built-in feedback, can't hurt you.**

- **Simplification** — No more patching two algorithms together for feedback routing
- **Safety** — Brick wall limiter/saturator ensures the signal can't run away; creative distortion, never pain
- **Immediacy** — Per-input feedback dials with CV control; experiment fast, get results fast

---

## Project Classification

| Attribute | Value |
|-----------|-------|
| **Technical Type** | Embedded Plugin (disting NT C++ for Eurorack) |
| **Domain** | Audio/Music Production |
| **Complexity** | Low (regulatory) / Medium (technical - DSP, real-time) |
| **Project Context** | Greenfield - new plugin |
| **Target Platform** | Expert Sleepers disting NT module |
| **Language** | C++ |
| **API Version** | v9 (kNT_apiVersion9) |

---

## Problem Statement

### Current State

Creating feedback loops on the disting NT currently requires:
1. Adding a **Feedback Send** algorithm to capture audio
2. Adding a **Feedback Receive** algorithm to inject it back
3. Patching these together correctly
4. Managing two separate algorithms for a single effect

This is fiddly, uses multiple slots, and doesn't provide easy per-channel feedback control or safety limiting.

### Desired State

A single algorithm that:
- Accepts multiple audio inputs
- Provides per-input feedback amount control
- Mixes inputs to a stereo output
- Prevents runaway feedback with a transparent safety limiter
- Offers CV control over all parameters

---

## Functional Requirements

### FR-1: Configurable Input Count

| ID | Requirement |
|----|-------------|
| FR-1.1 | Input count shall be user-configurable via NT specification (1-8 inputs) |
| FR-1.2 | Default input count shall be 2 |
| FR-1.3 | Each input shall be a mono audio input with selectable bus |

### FR-2: Per-Input Feedback Control

| ID | Requirement |
|----|-------------|
| FR-2.1 | Each input shall have an independent feedback amount parameter (0-100%) |
| FR-2.2 | Feedback topology shall be A→A (each input feeds back into itself only) |
| FR-2.3 | Feedback shall be applied before the input enters the mix |
| FR-2.4 | Each feedback amount shall be CV-controllable (0-100% depth, unipolar) |

### FR-3: Per-Input Panning

| ID | Requirement |
|----|-------------|
| FR-3.1 | Each input shall have a pan parameter (-100 to +100, center = 0) |
| FR-3.2 | Each pan parameter shall be CV-controllable (0-100% depth, unipolar) |
| FR-3.3 | Panning shall use equal-power panning law |

### FR-4: Stereo Output with Safety Limiter

| ID | Requirement |
|----|-------------|
| FR-4.1 | Plugin shall output a single stereo pair (L/R) |
| FR-4.2 | Output shall pass through a brick wall limiter before final output |
| FR-4.3 | Limiter shall have a configurable lookahead parameter (0.5ms - 20ms) |
| FR-4.4 | Limiter shall produce no audible clicks or artifacts |
| FR-4.5 | Limiter threshold shall be fixed at safe Eurorack levels (~10V peak) |
| FR-4.6 | When limiting engages, signal shall saturate/distort rather than hard clip |
| FR-4.7 | Saturation type shall be selectable (e.g., Soft/Tanh, Tube, Hard) |

### FR-5: Master Controls

| ID | Requirement |
|----|-------------|
| FR-5.1 | Plugin shall have a master output level parameter |
| FR-5.2 | Plugin shall have output bus selectors for L and R |
| FR-5.3 | Plugin shall have output mode (add/replace) |

---

## Non-Functional Requirements

### NFR-1: Performance

| ID | Requirement |
|----|-------------|
| NFR-1.1 | Plugin shall run within typical disting NT CPU budget |
| NFR-1.2 | Plugin shall process audio without dropouts at all supported sample rates |
| NFR-1.3 | Parameter changes shall be smoothed to prevent zipper noise |

### NFR-2: Safety

| ID | Requirement |
|----|-------------|
| NFR-2.1 | Output shall never exceed safe Eurorack voltage levels |
| NFR-2.2 | Feedback loops shall be inherently stable (cannot self-oscillate indefinitely without input) |
| NFR-2.3 | DC offset shall be filtered to prevent DC buildup in feedback path |

### NFR-3: Usability

| ID | Requirement |
|----|-------------|
| NFR-3.1 | Parameter pages shall be logically organized |
| NFR-3.2 | Parameter names shall use channel prefix (e.g., "1:Feedback", "2:Pan") |
| NFR-3.3 | Default preset shall produce audio immediately when inputs are patched |

---

## User Interface

### Parameter Organization

**Page 1: Inputs** (per-channel, repeated for each input)
- N:Input — Audio input bus selector
- N:Feedback — Feedback amount (0-100%)
- N:Feedback CV — CV input for feedback modulation
- N:Pan — Stereo pan position
- N:Pan CV — CV input for pan modulation

**Page 2: Output**
- Output L — Left output bus selector
- Output R — Right output bus selector
- Output Mode — Add/Replace
- Master Level — Output gain (0-100%)
- Lookahead — Limiter lookahead time (0.5-20ms)
- Saturation — Saturation type (Soft, Tube, Hard)

### Display

- Standard parameter display (no custom UI required for v1)
- Future consideration: level meters, limiting indicator

---

## Technical Architecture

### Signal Flow

```
Input 1 ──┬──[Feedback Loop 1]──┬──[Pan]──┐
          │         ↑           │         │
          └─────────┘           │         │
                                │         ▼
Input 2 ──┬──[Feedback Loop 2]──┬──[Pan]──┬──[Stereo Mix]──[DC Block]──[Limiter]──► Output L/R
          │         ↑           │         │
          └─────────┘           │         │
                                ▼         │
Input N ──┬──[Feedback Loop N]──┬──[Pan]──┘
          │         ↑           │
          └─────────┘           │
```

### Memory Architecture

| Memory Type | Usage |
|-------------|-------|
| SRAM | Algorithm struct, per-channel state |
| DTC | Limiter state, filter coefficients (hot path) |
| DRAM | Lookahead delay buffer |

### Key DSP Components

1. **Feedback Loop** — Simple multiply-accumulate with input
2. **DC Blocker** — High-pass filter (~5Hz) in feedback path
3. **Equal Power Panner** — `L = cos(θ), R = sin(θ)`
4. **Lookahead Limiter** — Delay line (up to 20ms) + envelope follower + gain reduction
5. **Saturation Modes:**
   - Soft (tanh) — smooth, symmetric clipping
   - Tube — asymmetric, even harmonics, warm character
   - Hard — aggressive clipping, odd harmonics

---

## Specifications (NT Respec)

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

---

## Success Criteria

| Criterion | Measurement |
|-----------|-------------|
| **Functional** | All inputs can be routed, feedback applied, and output heard |
| **Safety** | 100% feedback on all channels produces distortion, not pain |
| **Performance** | Runs without dropout at 8 inputs |
| **Usability** | User can dial in feedback and hear results in < 30 seconds |

---

## Out of Scope (v1)

- Effect chain reordering (Ghost-style)
- Matrix routing (A→B, B→C, etc.)
- Built-in effects (delay, filter, etc.)
- Custom UI / level meters
- Per-input output routing

---

## Open Questions

None — all resolved.

---

## Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-20 | Neal | Initial PRD |
