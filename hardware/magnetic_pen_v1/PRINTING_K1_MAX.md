# K1 Max · eSUN PLA Basic · First print

For a **0.4 mm nozzle** and **1.75 mm PLA Basic**. These are conservative starting settings for this unprinted prototype, not a validated machine preset.

**Choose a PLA filament profile.** The initial OrcaSlicer screenshot had `Generic TPU` selected. Duplicate `Generic PLA`, or the matching eSUN PLA Basic preset if available, and set the temperatures below.

## Print order and supports

1. Print **only** `bore_fit_coupon.stl`, `closure_fit_grip.stl` and `closure_fit_cap.stl` first. Keep the same settings for the full pen after checking bore fit and the twist closure.
2. **Supports OFF for the first trial.** The supplied orientations are correct: lower grip tip up/opening down; upper barrel rounded tail up/joint down; coupons and spacers flat.
3. **Uncheck `washer_1p2mm.stl` on the PLA plate.** It is a shape template for a compressible washer. Use the documented soft foam/compliant elastomer instead; a solid PLA washer prevents the intended closure.

| Part | Orientation supplied in STL | Support approach |
| --- | --- | --- |
| Bore coupon | Flat base; holes up | Off |
| Closure coupons | Flat joint/base ends down | Off; inspect slot bridges |
| Lower grip | Magnet opening down; tip up | Off initially; approximately 10.4 mm chamber bridge |
| Upper barrel | Joint end down; rounded tail up | Off initially; short slot bridges and approximately 7 mm handle bridge |
| 5/10/15/20 mm spacers | Flat end down | Off |

The barrel has a long internal cavity with a narrow vent. **Keep support out of that cavity**: it would be difficult to extract. Inspect the sliced layer preview around the bayonet slots, the chamber ceiling and the handle ceiling. Orca supports leaving short bridges unsupported. [Official bridge/support guidance](https://github.com/OrcaSlicer/OrcaSlicer/wiki/support_settings_advanced#dont-support-bridges)

If a coupon's exterior slot roof sags, first check bridge speed and cooling. If needed, choose **Normal (manual)** supports and paint only that accessible exterior underside; enable **On build plate only**. Avoid global automatic supports throughout the pen. [Official support controls](https://github.com/OrcaSlicer/OrcaSlicer/wiki/support_settings_support)

## Starting settings

| OrcaSlicer setting | Starting value |
| --- | --- |
| Nozzle temperature | **220°C** |
| Bed temperature | **55°C** |
| Layer height / first layer | **0.16 / 0.20 mm** |
| Wall generator | Arachne |
| Wall loops | **4** |
| Top / bottom shell layers | **6 / 6** |
| Sparse infill | **25% gyroid** |
| Outer / inner wall speed | **40 / 70 mm/s** |
| First layer / bridge speed | **20 / 25 mm/s** |
| Maximum volumetric speed | **8 mm³/s** for this fit trial |
| Cooling | Off for first 2 layers; **100% part fan by layer 4** |
| Small-layer cooling | Enable slowdown; aim for approximately **8–10 s** per small layer |
| Brim on upright parts | **Outer only, 5 mm**, approximately **0.15 mm gap** |
| Raft / spiral vase | Off |

The requested four wall loops cannot create four lines in the **0.9 mm neck wall**. Inspect the wall preview around the lugs: thin sections must still have continuous material. Keep inner brims out of the fit surfaces and magnet bore. [Orca brim controls](https://github.com/OrcaSlicer/OrcaSlicer/wiki/others_settings_brim), [cooling controls](https://github.com/OrcaSlicer/OrcaSlicer/wiki/material_cooling)

The temperature choices lie within eSUN's **210–230°C nozzle / 45–60°C bed / 100% fan** guidance. The slower speeds above prioritise the small fitting parts; they are our trial settings. [eSUN PLA Basic specifications](https://www.esun3d.com/pla-basic-product)

For these PLA trials, start with the top cover removed and keep the room reasonably temperate. Creality specifically instructs removing the cover for PLA/flexible filament when **room temperature exceeds 30°C**; this is not a requirement to keep the door open for every print. [K1 Max manual](https://wiki.creality.com/en/k1-flagship-series/k1-max/quick-start-guide/users-manual)

## After the coupons

- Pick the smallest tested bore that lets the actual disk slide without force. Measure it, then adjust `parameters.json` and rebuild if necessary; keep spacer and washer diameters compatible with the chosen bore.
- Check that the closure rotates fully, resists a straight pull and stays closed during a trial stroke. Remove brim burrs before judging its fit.
- Print the full pen and the spacer for the intended stack. Assemble with the soft washer, then test the assembled pen's new height and tilt configuration before participant collection.

[Pen design and assembly](README.md) · [Dimensioned drawing](drawing.pdf) · [Printable bundle](magnetic_pen_v1.zip)
