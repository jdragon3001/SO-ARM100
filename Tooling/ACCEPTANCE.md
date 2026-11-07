# Physical acceptance plan

This is the next evidence to collect, not a report of completed physical tests.
The supplied CAD is for fit prototypes; the simulator and reference protocol do
not drive a real robot. Results must be recorded before deciding operating loads.

## 1. Interface fit without electronics

Print the plug, three clearance coupons and receiver. Check the actual horn
pattern, recess, screw engagement and tool-side access. Select clearance using
the intended A1 mini/material profile. Ream the pin passages to a smooth sliding
fit; confirm the steel pin enters the tool hole at full seating and cannot show
locked with the key mis-seated. Test deliberate lateral/rotational error. Inspect
for cracking or rubbing after at least 100 unloaded hand insertions.

Do not print the complete kit before this fit test. New part STLs are translated
to the bed, but supports/brims and local screw-hole accuracy need slicer review.

## 2. One stationary dock

Assemble the segmented stand with its splice collars, through-bolts and table
fasteners. Bench-assemble the gate actuator and pin-release linkage with real
servos, horns, steel pin, spring and switches. Establish full constrained
dimensions and hard stops from those actual parts. The included cradle/crank/fork
are starting components, not a prevalidated actuator installation.

Use rack revision 2. Hand-seat a dummy module and measure how the front ledges
and closed gate restrain it in both axial directions. Confirm that gravity and
axial play leave it aligned for pickup; establish real locating pads or shims
and recheck the path if needed. Confirm that
gate motion does not strike the receiver, camera, ears, fingers or wires. Inspect
the full physical tool envelope, including a powered-off/open jaw. Confirm lock
release cannot occur while the tool is unsupported. Measure pin retraction force
through the entire stroke, gate force and insertion/extraction force separately.

## 3. Electrical bench test

Use a current-limited bench supply and test plug before installing a motor.
Continuity-check both mating faces and every pin number. Verify polarity, passive
ID windows, unknown/open/short rejection and presence independently. Check each
power/ground path with one parallel contact deliberately unavailable. Measure
contact resistance, heating, voltage drop and mechanical working stroke.

Verify supply-off discharge below 0.5 V, no data-line back-powering, current-limit
operation, boot-disabled behavior and communications watchdog. Implement and
flash the module controller and base release interlocks; the Python endpoint is
only their reference behavior. Confirm a motor cannot restart after a timeout
without an explicit new arm command. Check braking/regeneration behavior.

## 4. Retention and stiffness

With the receiver rigidly fixed on a bench, incrementally apply small known forces
and moments to a sacrificial plug. A proposed exploratory sequence is 1, 2 and
5 N axial pulls and 0.02, 0.05 and 0.1 Nm bending/twist. These are test inputs,
not approved working loads. Stop for permanent deformation, cracks, unwanted
release or unacceptable play. Set a working envelope only from measured results,
including repeated loading, material orientation and a chosen engineering margin.

Test loss of tool power and dock-actuator power while a dummy module is retained.
Verify pin/gate sensor faults, contradictory indications, stalled actuators and
interrupted communications. A servo's reported angle is not proof of engagement.

## 5. Slow robot commissioning

Calibrate physical dock positions and orientations. Check actual reach and
joint limits with an empty test plug. Compare measured pose error and compliance
with the spring contact/key tolerance; the simulation's 3 mm seating gate is not
an electrical-contact acceptance specification. Measure insertion force and motor
tracking at the actual rack radius. The current model omits contact-spring forces
and cannot establish available mating force.

Commission approach, seat, support transfer, release and withdrawal one phase at
a time at low speed. Then perform 100 empty-plug exchanges, logging independent
lock, gate, support, identity, voltage and actuator-fault signals. Investigate every
failure; do not mask it by loosening a sensor threshold. Only then repeat with
one real module, followed by all four and measured payloads.

## 6. Module-specific work

- Grippers: measure jaw travel, tendon/flexure motion, retention of small test
  objects, current draw and actual grasp force. Confirm a parked tool's fingers
  remain inside its cleared rack envelope when unpowered.
- Spatula: verify the selected metal tang and cross-pin, clamp and anti-rotation
  features. A generic utensil without the specified tang is not a fit.
- Stirrer: verify motor installation, metal coupler clamping, shaft/bearing
  alignment and axial retention. Test balance and no-load rotation before a
  controlled low-load task. Add an encoder for actual closed-loop RPM control.
- Camera: choose the module, verify its bracket and cable path, then calibrate
  intrinsics/extrinsics with each tool in view. Measure occlusion and cable forces.

Heat, liquids, food-contact suitability and cooking are not established by these
mechanical/electrical tests. No knife/chopper module is included.
