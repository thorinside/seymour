# Seymour (distingNT plug-in)

"Feed me, Seymour!" - A cascade feedback mixer with safety limiter.

Seymour is a multi-input mixer with a unique ring feedback topology: each channel receives delayed feedback from the previous channel, creating cascading echoes and self-oscillation when pushed.

## Features

- **Cascade Feedback**: Ring topology where Ch1→Ch2→Ch3→Ch1 (configurable 1-8 inputs)
- **Global Cascade Control**: 0-150% feedback amount (>100% for self-oscillation)
- **Safety Limiter**: Lookahead limiter with selectable saturation prevents damage
- **Per-Channel Panning**: Equal-power stereo panning with CV modulation
- **DC Blocker**: Prevents DC buildup in the feedback loop

## Pages / Parameters

### `Channel N` pages (per-input)
- `Input`: Audio input bus (0 = None)
- `Pan`: -100..+100 equal-power stereo pan
- `Pan CV`: CV input bus for pan modulation (0 = None)
- `Pan Depth`: 0–100% pan CV modulation depth

### `Seymour` page (algorithm-global)
- `Level`: Output gain (0–100%)
- `Cascade`: Feedback amount (0–150%); >100% allows self-oscillation
- `Lookahead`: Limiter lookahead time (0.5–20ms)
- `Saturation`: Limiter character - `Soft` / `Tube` / `Hard`
- `FB Delay`: Feedback loop delay time (0.5–20ms)
- `Squash`: Limiter threshold; 0% = least limiting (~10V), 100% = most limiting (~1V)

### `Routing` page
- `Out L`, `Out R`: Audio output busses
- `Out L Mode`, `Out R Mode`: Add/Replace output mode

## How It Works

The cascade topology creates a feedback ring:
- Channel 1 receives delayed feedback from Channel N (last channel)
- Channel 2 receives delayed feedback from Channel 1
- Channel 3 receives delayed feedback from Channel 2
- ...and so on, with the last channel feeding back to Channel 1

With `Cascade` at 0%, Seymour acts as a simple mixer. As you increase Cascade toward 100%, signals echo through the ring. Above 100%, the feedback builds into self-oscillation - the limiter and saturation keep it from destroying your ears.

## Build

- Hardware (`.o` for SD card): `make hardware`
- Desktop (`.dylib` for `nt_emu`): `make test`

## Installation

Copy `seymour.o` to your distingNT SD card in the `programs/plug-ins/` directory.
