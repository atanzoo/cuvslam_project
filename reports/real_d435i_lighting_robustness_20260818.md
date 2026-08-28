# Real D435i lighting robustness check — 2026-08-18

## Scope

This check keeps the thesis configuration as **stereo + IMU fusion** and
changes only the RealSense lighting/exposure strategy. It is intended to
explain why a slow 60 cm move can show reduced odometry growth in one location.

## Implemented R2 profiles

| Profile | Image denoise | Auto exposure | IR projector | Manual exposure | Use |
|---|---:|---:|---:|---:|---|
| `official` | on | on | on | n/a | formal NVIDIA low-light baseline |
| `robust` | on | on | off | n/a | diagnostic comparison for projector artifacts |
| `low_light` | on | on | on | n/a | alias of `official` for exploratory comparison |
| `high_contrast` | on | off | off | 8000 / gain 16 | diagnostic only; rejected for formal mapping |
| `baseline` | off | on | on | n/a | regression comparison |

The Jetson launch uses RealSense 4.54.1 / librealsense 2.54.2 and cuVSLAM
11.4. The emitter parameter is passed as integer `0/1`, which is required by
this RealSense wrapper.

## Measurements at the current location

The earlier `robust` profile produced:

- left/right image rate: approximately 30 Hz;
- IMU fusion log: `Enable IMU Fusion: true`;
- `vo_state=1`;
- GUI odometry rate: approximately 20–29 Hz after startup;
- IR mean: approximately 108–110;
- IR standard deviation: approximately 88;
- saturated pixels: approximately 22%;
- gradient proxy: approximately 12.

The image stream and estimator remained live, but the saturation warning shows
that this scene contains a large high-intensity region. This is consistent with
the location-dependent behavior reported by the operator.

The `low_light` exploratory profile increased the gradient proxy to roughly
15.5 but did not remove saturation. Manual exposure tests reduced saturation,
but exposure 5000–7000 made the images too weak for stable cuVSLAM output;
exposure 8000 also reduced the observed input rate. Manual exposure is therefore
not selected as the default.

## Emitter-off comparison at the reflective-metal location

At the operator's current test position, the same camera and auto-exposure
settings were restarted with `robust` (`emitter=0`, denoise on, IMU fusion on).
The result was:

| Profile | Left bright % | Right bright % | Left gradient | Right gradient | Image rate |
|---|---:|---:|---:|---:|---:|
| `official` emitter on | ~20.9 | ~21.6 | ~15.6 | ~17.0 | ~30 Hz |
| `robust` emitter off | ~20.8 | ~21.6 | ~12.3 | ~13.2 | ~30 Hz |

The clipped ratio did not materially change, while the gradient proxy became
lower with the emitter off. This location's clipping is therefore not solved
by disabling the projector alone; it is more consistent with a reflective
scene surface and/or ambient infrared returning to the sensor. The formal
profile was restored to `official` after this comparison.

## Covering test at the same location

The reflective strip was then covered while keeping the official profile and
the camera at the test location. The short live sample still showed stable
30 Hz input, but the global bright-pixel ratio increased to approximately
24.6% on the left and 23.6% on the right. This does not prove that the strip
was unrelated: auto exposure can increase exposure after a bright object is
covered, and the covering material itself may reflect infrared. The current
global metric is therefore a scene-level warning, not a localized reflector
classifier.

## Current decision

Use `official` for the next controlled motion tests. Treat the GUI
image-quality panel as a gate:

1. image rate should be near 30 Hz;
2. `vo_state` should remain `1`;
3. odometry should be above roughly 20 Hz after warm-up;
4. record the mean/std, saturated-pixel percentage, gradient proxy, and path
   metrics together.

Each GUI motion recording now writes a JSON evidence file under
`logs/real_d435i_quality/`, so two locations can be compared directly.

## Formal R2 decision

The formal R2 baseline is now the NVIDIA low-light strategy:

```text
IMU fusion = true
denoise_input_images = true
auto exposure = true
IR projector = on
raw, uncompressed left/right IR images
profile = 640x360x30
```

The local integration keeps the custom topic/frame mapping, IMU noise model,
GUI telemetry, motion evidence JSON, and mapping controls. Manual exposure is
diagnostic-only. `robust` remains available as an emitter-off comparison when
projector dots or reflective-surface artifacts are suspected.

## Interpretation

The current evidence does not support an IMU-failure diagnosis. The practical
software mitigation is denoising plus an explicit projector policy and image
quality telemetry. If a second location again shows high saturation or low
texture, the remaining fix is environmental: avoid direct sunlight/window
glare and glossy surfaces, or add diffuse, uniform illumination. Do not hide
the effect with a fixed odometry scale multiplier.
