# CyberKnife-Style Tumor Motion Tracking (MuJoCo)

A six-revolute-joint robot arm tracks a synthetic breathing-driven target in
MuJoCo. The project includes student-implemented forward position kinematics,
the translational Jacobian, two numerical inverse-kinematics methods, a
joint-space feedback controller, repeatable experiments, CSV metrics, and
plots. It is an educational simulation only; it is not a clinical or treatment
planning system.

## Run it

Python 3.10 or later is recommended. From the repository directory:

```powershell
python -m pip install -r requirements.txt
python robot_simulation.py
```

The default command runs five experiments and writes sample logs,
`results/metrics.csv`, `results/metrics.json`, and
`results/tracking_results.png`.

To run one case:

```powershell
python robot_simulation.py --experiment nominal
python robot_simulation.py --experiment challenge
python robot_simulation.py --experiment pseudoinverse
python robot_simulation.py --experiment low-gain
python robot_simulation.py --experiment mid-gain
```

For the live MuJoCo demonstration:

```powershell
python robot_simulation.py --view
```

This opens a continuous simulation plus a control panel. Use **Play breathing**
or **Play circle** to run a repeating target trajectory, **Hold** to stop
tracking at the current joint positions, and **Go home** to return to the
initial pose. The six joint sliders set manual joint targets in degrees; moving
any slider switches to manual control. Close either window to end the live
session. The finite headless experiments remain available without `--view`.

Run the focused checks with:

```powershell
python -m pytest
```

## Model, methods, and assumptions

The six-joint serial chain and its fixed transforms, joint axes, limits, and
tool offset come from [`cyberknife_6dof.xml`](cyberknife_6dof.xml). STL meshes
are display geometry, scaled from millimetres to metres. The code uses MuJoCo
for model loading, rigid-body dynamics, and reference/validation positions;
it does not call MuJoCo's inverse kinematics or Jacobian routines.
The existing [`src/forward_kinematics.py`](src/forward_kinematics.py) remains a
separate DH reference implementation; the active simulation uses
[`src/kinematics.py`](src/kinematics.py) so its FK matches the CAD-derived
MuJoCo chain without guessing DH link dimensions.

For a joint on a body, the implemented transform rotates about its specified
local pivot:

`T_body(q) = T_fixed T(pivot) R(axis, q) T(-pivot)`.

Chaining these transforms gives the tool position `p(q)`. For revolute joint
`i`, the implemented translational Jacobian column is
`J_i = axis_i x (p - pivot_i)`. The tool point is position-controlled; tool
orientation is not constrained, leaving a redundant six-joint solution.

The inverse kinematics minimizes Cartesian position error. Damped least
squares uses `dq = J^T (J J^T + lambda^2 I)^-1 e`; the comparison method uses
the Moore-Penrose pseudoinverse. Both methods limit update size, enforce joint
limits, and use a short error-reducing line search. The damping is fixed at
0.04; a small singular value therefore does not cause unbounded steps.

The target follows the supplied chest/lung motion notes. Chest and lung
translations use `sin^(2n)(pi t/T)` and
`sin^(2n)(pi t/T - phi)`, respectively, and the lung displacement is rotated
by a small chest rotation about X. Default amplitudes, period, and phase are
explicitly documented in `src/tumor_trajectory.py`; they are illustrative,
not patient data.

At each 20 ms control update, position IK produces a desired joint vector.
The implemented saturated joint PD law is
`tau = sat(qfrc_bias + Kp(q_des - q) - Kd qdot)`. `qfrc_bias` is MuJoCo's
model-based bias-force term; PD feedback, torque saturation, IK, and the
Jacobian are implemented in Python. Gains and torque limits are defined near
the top of `src/simulation.py`; the fixed gains are a demonstrator setting,
not clinically validated tuning.

## Experiments and measures

1. **Nominal DLS:** nominal initial condition, 4 s breathing period, and
   default motion amplitudes.
2. **Challenging DLS:** perturbed initial joints, 1.5x motion amplitude,
   3.2 s breathing period, and a 0.8 Nm sinusoidal joint-three disturbance.
3. **Nominal pseudoinverse:** same initial condition and trajectory as the
   nominal DLS run, changing only the IK algorithm.
4. **Low-gain tuning:** nominal trajectory with both PD gain matrices scaled
   to 0.5.
5. **Mid-gain tuning:** nominal trajectory with both PD gain matrices scaled
   to 1.0; the nominal case uses the selected 1.5 scale.

`metrics.csv` reports RMS, 95th-percentile, peak, and final tool-to-target
position error; mean IK solve time; mean Jacobian condition number; and
post-1-second RMS tracking error and integrated squared actuator torque.
Per-sample CSVs retain target/tool positions, joint states, IK convergence
effort, and applied torque. Nominal DLS versus pseudoinverse is the IK
comparison. The gain sweep scales `Kp` and `Kd` together to preserve their
ratio. The selected 1.5 scale has lower measured tracking error with nearly
unchanged torque effort than the 1.0 setting; it is checked again in the
challenging case.

## Limitations and report

The robot inertial properties are those inferred by MuJoCo from the supplied
mesh/model; the model has no calibrated actuator identification or validated
patient coordinate registration. Robot-link collision is disabled in the XML,
and no collision avoidance, radiation dose, image guidance, breathing
estimation, or safety interlocks are simulated. Results must not be interpreted
as evidence of treatment accuracy or safety.
No separate obstacle-avoidance planner is used: the task prescribes the
Cartesian target path and online IK generates the desired joint positions.

See [`docs/TECHNICAL_REPORT.md`](docs/TECHNICAL_REPORT.md) for the concise
submission report, including generated quantitative results and a group
contribution section to complete before submission.
