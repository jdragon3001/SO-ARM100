# Research register — 6 November 2026

These are primary sources consulted for ATC-02. Published performance belongs
to those systems, not to this prototype. No external validated performance is
assigned to the new receiver, fingers or rack.

| Source | Finding used | Design consequence |
|---|---|---|
| [SO-101 assembly](https://huggingface.co/docs/lerobot/so101) and local upstream STEP/MJCF at `a758567` | Five positioning joints plus the gripper; native daisy-chained addressed servos | Preserve the arm geometry/limits; move the complete gripper into a module. Extract actual mounting axes from STEP. Hardware software will need a five-axis arm plus tool adapter, rather than the stock six-servo assumption. |
| [Jubilee remote elastic lock](https://jubilee3d.com/index.php?title=The_Remote_Elastic_Lock) | Locating and retaining are distinct functions. Remote actuation reduces carried mass; wear and printed locking surfaces matter. | Put release actuators on the dock. Use a steel pin for retention and independent confirmation. Do not assert that printed tapers establish repeatability or reliable locking. |
| [Berkeley passive modular tool changer, CASE 2018](https://goldberg.berkeley.edu/pubs/CASE2018-ron-tool-changer-submitted.pdf) | Robot-side coupling, tool-side part and tool housing form a system; positioning accuracy is central to change reliability. | Include a rack, oriented approach poses and physical acceptance work. A universal wrist plate alone cannot make changes automatic. |
| [ATI tool changer operation manual](https://ati.novanta.com/wp-content/uploads/sites/2/2025/05/9610-20-3818.pdf) | Support and de-energizing before release, lock/unlock feedback and correct tool storage are explicit operating concerns. | Gate unlocking on support, seating, stopped tool and measured voltage. Confirm tool capture before transferring support; do not use a timer as lock confirmation. |
| [Yale OpenHand Model T](https://www.eng.yale.edu/grablab/openhand/model_t.html) | Four compliant fingers can be driven through a single-actuator pulley-tree differential. The original hand is much larger/heavier than this arm's intended tool. | Create a small custom equalizer/finger prototype; do not transplant its 490 g assembly. Do not copy its CAD or imply the scaled design inherits its force performance. |
| [Pololu motor #992](https://www.pololu.com/product/992) | LP 6 V gearmotor: 10×12 mm gearbox cross-section, 3 mm D shaft, 130 RPM no-load; stall figures are not operating ratings. | Reserve the N20 envelope and use a conservative low-load, 0–60 RPM stirring experiment. Use a metal coupling and bearing; measure current and speed under load. |
| [Pololu DRV8833 carrier](https://www.pololu.com/product/2130) | Motor voltage and logic are distinct; carrier has optional current limiting that needs modifications. | Specify a local driver and require actual current-limit/supply testing. No claim of automatic force/current control from the stock carrier. |
| [TI TPS25940](https://www.ti.com/product/TPS25940) | Protected switching with adjustable current limiting and power-good/current-monitor functions | Use a separate switched tool rail, measured discharge and fault feedback. The part is a reference candidate, not a completed PCB. |
| [TI THVD1450](https://www.ti.com/product/THVD1450) | Half-duplex RS-485, 3.3/5 V logic supply, controlled power-up/down behavior | Propose a dedicated differential tool link with module controllers; retain native arm TTL wiring separately. |
| [Mill-Max spring-contact catalog](https://www.mill-max.com/sites/default/files/external/catalog/2019-04/023M-028M.pdf) | Spring contacts have specific body dimensions, working stroke, force and current limits | A generic contact hole is a fit coupon only. Confirm the exact selected part's drawing and insertion force before final carrier/PCB design. Full PDF access was blocked in this research session; indexed catalog dimensions are not treated as a qualified footprint. |
| [SwapBot, University of Washington](https://personalrobotics.cs.washington.edu/SwapBot/) | Common tool interfaces can support demonstration collection and autonomous household-tool swaps | Keep the complete-tool concept and task-specific data. Its hardware guide was marked forthcoming; its electromagnetic coupling is not imported or represented as available source CAD. |
| [build123d import/export documentation](https://build123d.readthedocs.io/en/latest/import_export.html) | Editable STEP and manufacturing meshes can be generated from scripted solid geometry | Keep parametric source, validate solid topology, export both STEP and STL, and record source/output hashes. |

## Decisions specific to this prototype

- No always-energized magnet is used as the intended load-retaining mechanism.
  A spring-biased pin is the proposed retention element. Real power-loss behavior
  still depends on its actual geometry, spring, wear, sensor and actuator tests.
- Rack ears support the module front faces; a separate sliding gate captures the
  rear faces. The gate closes before wrist release and opens only after wrist
  attachment is confirmed. This is represented as a rack weld in MuJoCo.
- Pin release and rack capture are separate functions. Plan for two stationary
  actuators per station in the first bench rig; later simplification is possible
  after measuring forces and validating the sequence.
- The electrical design uses a proposed 6 V tool rail and eight mating contacts.
  It is deliberately separate from the existing arm's TTL servo bus. This adds
  a small controller to active modules and leaves PCB/firmware integration work.
- The first build target is an empty test plug and receiver on a bench, followed
  by one dock. Four-station autonomous operation comes after that evidence.

## Outstanding evidence

Actual printed fit, horn screw engagement, tool/receiver mass and inertia,
spring-contact insertion force, lock pin preload/friction, gate stiffness,
actuator mounting alignment, electrical contact stroke, rail behavior, camera
compatibility, MCU implementation, gripper force, shaft retention and repeated
physical docking. The simulation does not supply those measurements.
