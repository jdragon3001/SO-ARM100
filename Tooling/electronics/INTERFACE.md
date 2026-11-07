# Proposed ATC-02 power and signal interface

**Architecture and bench interface, not a routed PCB or flashable controller.**
The workbench shows the contact row apart from the parts. It illustrates
which contacts would mate; it does not define an installed position, contact
compression, or a usable wiring harness. No wiring in this package has been
physically energized. Pin names below define
our interface; they are not a manufacturer's connector numbering convention.

## Separate tool and arm systems

Keep the arm's existing Feetech TTL servo chain unchanged. Feed the interchangeable
tool from a separate **regulated 6.0 V rail**, initially current-limited to 1.0 A
for bench work. This is a design target, not a proven operating current budget.
The same 6 V rail can serve the selected servo class and the low-power 6 V N20
motor. Size the actual supply from simultaneous/starting current measurements.
Do not connect a 12 V arm supply straight to the tool rail or its logic.

A dedicated RS-485 link connects a base tool controller to a small controller
inside each powered module. The gripper module translates to the servo's native
TTL protocol; the stirring module translates to a brushed-motor driver. RS-485
is not electrically interchangeable with Feetech TTL. A transceiver and logic
interface are required. This avoids changing the arm bus protocol whenever a
different tool is installed.

Suggested reference parts: TI THVD1450-class half-duplex transceivers, a local
3.3 V regulator/controller, a TPS25940-class protected power switch at the base,
and a DRV8833 carrier for the N20. These are researched candidates. A PCB design,
decoupling/protection layout, MCU port and physical test remain outstanding.
DRV8833 logic inputs are not motor power inputs. Its current limiting is not
automatically enabled on the referenced carrier; fitting sense resistors and
checking their effect is a separate step.

## Eight blind-mating contacts

Use spring contacts and metal landing pads, not a manually latched cable plug.
The first prototype footprint is a single row on 2.54 mm pitch. The printed
carrier is a fit coupon for a nominal 1.9 mm contact body; verify the exact
selected Mill-Max-style part and working stroke before making the real carrier.
The landing strip is a separate PCB/contact component, not printed conductive PLA.
The +6 V pair and ground pair are duplicated for contact reliability; the 1 A
bench limit must remain safe if one of the parallel contacts fails to mate.

| Our pin | Function | Connection |
|---|---|---|
| 1 | TOOL_6V_A | Switched, protected 6 V |
| 2 | TOOL_6V_B | Same switched 6 V net |
| 3 | GND_A | Tool power/signal return |
| 4 | GND_B | Same ground net |
| 5 | RS485_A | Dedicated tool bus; twisted with B along arm |
| 6 | RS485_B | Dedicated tool bus |
| 7 | TOOL_ID | Passive resistor to ground; base ADC reads with tool power off |
| 8 | PRESENT | Passive ground strap, separately sensed at base |

Pin 1 is marked on the clipped-corner end of our carrier drawing. View both
halves from the mating face and mirror the PCB layout appropriately; do not
wire two identical left-to-right drawings without checking continuity.

ID resistors: gripper 1.0 kΩ; four-finger 2.2 kΩ; spatula 4.7 kΩ; stirrer 10 kΩ.
Use a 3.3 V ADC divider and define non-overlapping measurement windows. An open,
short or unrecognized value must be UNKNOWN. A valid ID is necessary but does
not prove the lock engaged. These resistor values identify the tool type, not
unique serial numbers. The reference classifier allows ±10% resistance windows.

Both passive ID/presence are available before applying motor power. Keep data
drivers disabled/high impedance while a tool is unpowered or contacts are moving.
Select powered-off-protected transceivers and verify no back-power path through
data, ID or presence. Provide suitable termination/bias for the dedicated cable;
place it according to the actual topology, not arbitrarily at every module.

## Required signals outside the interchangeable contacts

The permanent receiver needs independent **LOCKED**, **UNLOCKED** and
**SEATED/PRESENT** sensing. The rack needs **TOOL_SUPPORTED** at each station.
The base measures tool rail voltage and current, controls tool-power enable,
and commands each stationary release actuator. These sensors/cables are not
replaced by the eight mating contacts. Do not infer LOCKED solely from a servo
angle or an elapsed delay. Physical switch/flag selection and brackets remain
bench integration work.

Power removal sequence: command stop → confirm stopped → support in dock →
disable data drivers → switch tool supply off → measure rail <0.5 V → unlock.
The supply needs an actual discharge path; software waiting alone does not
discharge a capacitor. A 1 kΩ bleeder with a 100 µF local capacitor has a nominal
0.1 s time constant, but the controller must measure voltage rather than assume
that timing. Verify regeneration/backfeed from a rotating motor as well.

Pickup sequence: fully seat → engage pin → confirm lock and passive identity →
enable protected power with zero motor demand → identify the controller and
confirm readiness → withdraw. Passive spatula has no motor demand. The hardware
must default to disabled on boot, loss of communications and contradictory sensors.

## Tool bus reference

`protocol.py` implements a proposed framing/endpoint contract for testing:
ATC2 magic, sequence, bounded JSON payload, CRC32. Boot is disabled. Explicit
arming requires lock and power-good. A monotonic sequence rejects replay. A
250 ms command watchdog stops output and requires rearming. Units are 0–1
gripper travel/tendon demand and 0–60 RPM stirring target. Actual speed feedback
requires an encoder; the current N20 candidate has none. Without one, hardware
must report duty/open-loop status rather than claim regulated RPM.

The endpoint is a Python reference model. It does not implement MCU UARTs,
RS-485 direction timing, a Feetech driver, encoder feedback or motor hardware.
No physical auto-changing should run until that firmware, the hardwired release
permissions and the acceptance tests exist. The desktop simulator does not send
this protocol to any connected device.
