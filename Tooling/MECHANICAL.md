# Mechanical prototype and assembly register

All dimensions are millimetres. The STEP files are editable solids; the Python
generator is the dimensional source. All new parts are fit prototypes. Select
real fasteners, bearings, spring, blade and camera before any load testing.

## Receiver

The existing wrist-roll horn interface was extracted from cylindrical faces in
`STEP/SO101/Follower_Specific/Wrist_Roll_Follower_SO101.step`. The four 3.2 mm
holes lie at ±4.949747 mm on X/Y around their source centre: a 14 mm pitch circle
at 45°. The generator checks this against the source STEP on every run. The new
receiver provides 3.3 mm clearance bores, a horn recess and head access from the
socket. Screw length must be selected against the actual horn and printed stack;
do not bottom screws in the servo or substitute a printed spline.

The receiver is 24 mm long. A clipped-corner tapered male key enters 9 mm into
the matching socket. Nominal clearance is 0.25 mm per side. The clipped corner
establishes one tool orientation; seating faces carry compression and the key
is intended to resist twist. Neither its stiffness nor pull-out resistance is
currently rated.

A 3 mm steel sliding pin at receiver Z=20 engages the cross-hole in the module
key at module Z=-4. Its intended retraction is 7 mm. A compression spring biases
the pin toward engagement; an external collar allows the dock to pull it back.
Use a smooth steel rod/pin, not a printed load-bearing pin. The housing has a
removable side cover. The printed collar is a fit part and needs positive
retention on the actual rod. Spring dimensions, preload, friction and pin shear
loads must be measured before this mechanism carries a tool.

The receiver/module CAD intersection check covers the nominal seated key and
plate. It does not establish tolerance-stack clearance, strength or the swept
envelope of every screw, wire and spring.

## Dock and release mechanism

Use the **revision 2** rack files. The original dock uprights intersected the
rendered tool heads and must not be used. The revised stations sit on a wider
arc, with the ears rotated across the station and the uprights outside the tool
path. The rack poses and joint angles are in the manifest. The arm aligns 150 mm
from the seating plane and follows successive poses along the dock axis rather
than taking a single joint-space shortcut through the approach and withdrawal.

Two front ledges and the rear retaining gate surround the module ears at X=±55
mm. At the nominal parked pose the front ledges are 2.5 mm from the ear faces
and the closed gate is 2.0 mm behind them. These are prototype geometry gaps,
not a physical locating tolerance. The gate slides 20 mm along the guides, with
5 mm nominal transverse clearance to the ears when fully open. It closes
before wrist release and finishes opening before withdrawal.

The simulation uses a weld to hold the parked tool at the nominal pose.
The printed rack alone does not establish that pose or its repeatability.
On the bench, establish how the tool seats under gravity, how a compliant
locator or shims take up the slack without binding, and how it reacts to
insertion and pullout forces. Recheck clearance with those parts installed.

The rack releases the arm's pin; release and gate servos stay off the wrist. The
initial bench arrangement uses two actuators per station, one per function. The
gate needs a separate guided 20 mm stroke and closed/open feedback. Its printed
guides/pads are included, but its actuator linkage is not a constrained assembly.
Included
bench components are `dock_servo_cradle` (reused upstream wrist servo holder),
`dock_servo_crank` and `dock_release_fork`. The 12 mm crank radius and ±17° arc
give approximately 7.02 mm horizontal travel. The fork slot accommodates the
small transverse arc component. A bench assembly must establish the servo/crank
axis, guided fork position, hard stops and collar engagement; these components
are **not a fully constrained, tolerance-qualified actuator assembly**. Do not
assume the pedestal's general mounting pad fixes that alignment automatically.

Each pedestal is split at Z=120 for the A1 mini. Print its lower section and two
upper uprights, then use two `dock_post_splice_print_eight` collars per station.
The collars bridge the butt joints; M3 through-bolts at global Z=110 and 130
join the sections. Use nuts and washers, then bolt the pedestal to the table.
The full `dock_*` STEP/STL files are visualization assemblies and exceed the
printer height. They are intentionally absent from `print/`.

## Four modules

| Module | Parts and actuator | Assembly/validation requirements |
|---|---|---|
| Two-finger | `gripper_body`, `gripper_jaw_1`, original STS3215 servo/horns | Existing jaw and servo pocket geometry are retained. The common key/plate is integrated behind them. Verify servo fit, wire exits, jaw travel and horn screw clearance. |
| Four-finger | Palm, lid, guard, four TPU fingers, spool, one primary and two secondary equalizer bars, STS3215 | Shared tendon with a floating two-level equalizer. Separate flexure fingers from the rigid body. The supplied servo horn receives the spool; the design does not print a spline. Measure real shaft alignment, tendon routes, pretension and flexure response. |
| Spatula | Holder, front clamp, purchased blade/tang | Proposed tang envelope: 12 × 6 mm, seated against the pocket bottom, with a 3.2 mm retention hole 6 mm from its inserted end. Four M3 clamp bolts lie outside the tang. A separate M3 cross-bolt is positive pull-out retention. Select or machine the actual tang to this interface; arbitrary utensils do not automatically fit. |
| Stirrer | Back/front shell, bearing cap, purchased N20 gearmotor, metal coupling, 4 mm shaft and 4×10×4 bearing | Split shell permits motor installation. Motor body starts near Z=10, coupling occupies Z=38–56, bearing seats at Z≈57–61; the cap retains it. Verify motor length, shaft engagement, coupler OD, bearing fit and axial load path against actual parts. The printed coupling is a fit coupon, not an operating shaft coupling. |

For the four-finger module, a continuous tendon pulls the primary equalizer;
its two branches pull the centres of the secondary bars, whose ends connect to
the four fingers. Leave the bars free to translate/tilt within the guard. This
is a scaled custom mechanism inspired by underactuated hands, not a copy of a
validated hand or a claim of force-controlled grasping. The simulator's hinged
fingers/tendon are only a motion approximation.

The powered tool starts as a slow paddle (0–60 RPM requested). A whisk tip can
use the same motor module after shaft retention, balance and reaction loads
are measured. This motor choice is for low-load stirring experiments, not thick
batter or chopping. No food-contact blade, whisk or spinning paddle STL is
provided. The rendered paddle is a purchased-tool envelope.

## Fit-first printing

Print `test_plug`, the 0.15/0.25/0.35 mm socket coupons, contact carrier coupon
and one receiver before a complete set. Choose the clearance that slides without
binding while keeping acceptable play. Check the pin at full insertion and
deliberate misalignment. A key must not bind in order for the pin to look locked.

`print/` places individual components on Z=0 without changing their modeled
orientation. Supports may be needed for horizontal holes, internal pockets,
upright finger flexures and fork overhangs. Bed fitting is not a slicing check.
Use the A1 mini profile in Bambu Studio and review every support/brim preview.
TPU flexure tests and rigid receiver/rack tests need separate material profiles.

The camera remains undecided. `camera_bracket_32mm` is a 28 mm hole-pitch nominal
carrier for a 32 mm board, not verified compatibility with every 32 mm module.
Confirm board holes, lens, USB connector and focus clearance first.
