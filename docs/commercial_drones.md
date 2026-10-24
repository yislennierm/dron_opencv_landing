# Commercial precedents for natural-ground landing

Research checked 2026-10-24 against manufacturer product information and manuals.
These are documented capabilities, not independent landing-accuracy measurements.
Models and firmware differ; generic return-to-home does not establish visual
precision landing.

## Closest matches to this project

| Aircraft | Manufacturer-documented capability | Relationship to our concept |
| --- | --- | --- |
| DJI Air 3S | Precision landing on the takeoff point using distinctive, largely unchanged home terrain. Requires a home point recorded at takeoff, at least 7 m of vertical ascent before horizontal movement, and suitable lighting. | Close functional precedent for learning natural ground during takeoff and recognizing it during final approach. [Manual, pp. 47–48](https://dl.djicdn.com/downloads/DJI_Air_3S/UM/20260423/DJI_Air_3S_um_en1.pdf#page=48). |
| DJI Mavic 4 Pro | The manual documents precision landing with similar terrain, light, and ascent conditions. It also describes a conditional vision-supported return when GNSS was unavailable at takeoff, involving retracing the outbound trajectory near home. | Both a close landing precedent and a broader visual-navigation example. The latter requires a much larger navigation system than our local landing matcher. [Manual v1.2, pp. 44–48](https://dl.djicdn.com/downloads/mavic-4-pro/20250925/DJI_Mavic_4_Pro_User_Manual_v1.2_en.pdf#page=45). |
| Autel EVO Lite series | Downward binocular cameras record takeoff-point information and vision algorithms estimate displacement from it during return/landing. Precision landing depends on preserving the home point, environment, and usable ground texture. | Especially direct confirmation of the remembered-ground concept; Autel has stereo sensing whereas our baseline uses one camera plus supplied height and attitude. [Manual, printed pp. 43–44 and 85–86](https://www.autelrobotics.com/wp-content/uploads/2024/08/EN_EVO-Lite-Series-Aircraft-User-Manual_V3.0.9.pdf#page=54). |
| Autel EVO II series | Autel explicitly attributes precision landing to downward binocular observation of ground texture at takeoff and says it can fail without texture. | Another natural-ground commercial precedent. [Manual, FAQ Q10](https://www.autelrobotics.com/wp-content/themes/autel/userfiles/files/2021/10/26/EVO%20II%20User%20Manual.pdf). |

## Broader visual navigation and return

| Aircraft | Documented capability | What it establishes |
| --- | --- | --- |
| DJI Mini 5 Pro | DJI advertises flight-route memorization with sufficient light, allowing takeoff and return without satellite signal. | Visual route-memory RTH is commercially deployed even in a small consumer platform. This claim alone does not disclose the final landing algorithm or demonstrate markerless landing accuracy. [DJI product page](https://store.dji.com/sg/product/dji-mini-5-pro). |
| Skydio X10 | Skydio lists Visual Return-to-Home and GPS-denied autonomy. Its return instructions distinguish Launch from a map-set Home point, which requires GPS. | A broader multi-camera navigation system, rather than evidence that the same function can fit an ESP32-P4. Do not equate arbitrary map-point return with return to a visually remembered launch. [X10 FAQ](https://www.skydio.com/x10/faqs), [return instructions](https://support.skydio.com/hc/en-us/articles/20463874306203-How-to-return-and-land-Skydio-X10). |
| Autel EVO Max 4T | Autel advertises non-GPS return-to-home on its autonomy page, including a manufacturer claim of return-point horizontal error under 30 cm within its stated height conditions. | Broader autonomy precedent. This is an advertised system-level figure, not a tested accuracy bound for our method or every EVO Max firmware version. [Autel autonomy page](https://shop.autelrobotics.com/pages/evo-max-4t-autonomy). |

## Engineering implications

Natural-ground precision landing is an established commercial function. This
supports investigating the concept, but does not validate our specific matcher,
thresholds, single-camera geometry, or embedded resource budget. Manufacturers
do not disclose enough algorithm detail in these sources to infer that they use
ORB, FAST, a particular homography model, or our keyframe strategy.

The recurring terrain and lighting conditions should become explicit test
dimensions: distinguishable texture, appearance change, repetitive patterns,
vertical ascent overlap, and loss of visual support. DJI's 7 m ascent requirement
belongs to those products; it is not a justified threshold for our camera.

Keep three functions distinct when defining the prototype:

1. **Visual precision landing:** refine alignment to the remembered takeoff area
   after that area is visible. This is our initial target.
2. **Visual return navigation:** find a route back using a maintained visual map
   or route history. This is a separate, larger problem.
3. **Landing protection:** assess whether the surface is suitable to touch down.
   Recognizing home does not establish surface safety. Autel's manual explicitly
   describes landing protection separately from accurate landing.

The strongest immediate comparison is with DJI Air 3S / Mavic 4 Pro precision
landing and Autel EVO Lite / EVO II takeoff-ground matching. Skydio X10, EVO Max,
and newer DJI route-memory RTH are useful references for later navigation work.
