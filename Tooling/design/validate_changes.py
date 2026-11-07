"""Reproduce the nominal four-tool audit trace; does not test physical hardware."""

import json, sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "Simulation/Workbench"))
from engine import Simulation, TOOLS


def main():
    sim = Simulation()
    report = {
        "revision": "ATC-02",
        "physical_tests_completed": False,
        "coupling": "ideal weld",
        "sensors": "synthetic",
        "changes": [],
    }
    for name in TOOLS[1:] + TOOLS[:1]:
        start = sim.data.time
        begin = len(sim.change_log)
        sim.change_tool(name)
        peak = np.zeros(5)
        saturated = np.zeros(5)
        samples = 0
        maximum_contact = 0.0
        contact_samples = []
        while sim.busy and sim.data.time - start < 60:
            sim.step(20)
            tau = abs(sim.data.actuator_force[:5])
            peak = np.maximum(peak, tau)
            saturated += tau >= 0.99 * sim.config.torque_nm
            samples += 1
            contacts = sim.contacts()
            maximum_contact = max(
                maximum_contact, max([c["penetration_mm"] for c in contacts] + [0])
            )
            if contacts and len(contact_samples) < 5:
                contact_samples.append(
                    {
                        "time_s": float(sim.data.time),
                        "phase": sim.phase,
                        "contacts": contacts,
                    }
                )
        report["changes"].append(
            {
                "tool": name,
                "result": sim.phase,
                "duration_s": round(float(sim.data.time - start), 3),
                "peak_arm_torque_nm": peak.tolist(),
                "fraction_samples_at_ceiling": (saturated / max(samples, 1)).tolist(),
                "maximum_reported_contact_mm": maximum_contact,
                "contact_samples": contact_samples,
                "trace": sim.change_log[begin:],
            }
        )
        print(name, sim.phase, round(sim.data.time - start, 2), flush=True)
        if sim.phase != "READY":
            raise RuntimeError(sim.error)
    (ROOT / "Build/Tooling/validation.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )


if __name__ == "__main__":
    main()
