# Seymour (distingNT plug-in)

Feedback mixer with a safety limiter (“squash”) and per-channel panning.

## Pages / Parameters

### `Channel N` pages (per-input)
- `Input`: audio input bus (0 = None)
- `Feedback`: 0–100% feedback amount
- `FB CV`: CV input bus (0 = None)
- `FB Depth`: 0–100% blend between `Feedback` and `FB CV` (only used when `FB CV` is set)
- `Pan`: -100..+100 equal-power pan
- `Pan CV`: CV input bus (0 = None)
- `Pan Depth`: 0–100% pan modulation depth

#### CV polarity/scaling
- `FB CV` is treated as bipolar ±5V and mapped to a unipolar 0..1 control: `-5V → 0`, `0V → 0.5`, `+5V → 1` (clamped).

### `Seymour` page (algorithm-global)
- `Level`: output gain (0–100%)
- `Lookahead`: limiter lookahead (0.5–20ms)
- `Saturation`: `Soft` / `Tube` / `Hard` (applied when limiting is active)
- `FB Delay`: feedback loop delay (0.5–20ms)
- `Squash`: limiter threshold; `0%` = least squash (≈10V peak), `100%` = most squash (≈1V peak)

### `Routing` page
- `Out L`, `Out R`: audio output busses
- `Out L mode`, `Out R mode`: Add/Replace output mode for each output

## Notes
- With `Feedback = 0`, the dry input path is intended to be essentially unchanged (no “always-on” DC blocking of the dry signal).
- The limiter operates in “bus volts”; pushing signal into the limiter (or increasing `Squash`) produces the distortion/character.

## Build
- Hardware (`.o` for SD card): `make hardware`
- Desktop (`.dylib` for `nt_emu`): `make test`
