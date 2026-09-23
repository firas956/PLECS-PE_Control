# PLECS & Power Electronics Control

A personal learning repository documenting my work on power electronics: converter
theory, PLECS simulation, and the control techniques applied on top of them — from
classical linear PI/PID loops up to sliding mode control (SMC) of a fourth-order
Ćuk converter, with the SMC gains tuned automatically by a genetic algorithm
driving PLECS from Python over XML-RPC.

Each folder is a step in that progression: open-loop behaviour first, then
closed-loop regulation, then nonlinear control and optimization.

---

## Repository layout

```
PLECS&PE_Control/
├── buck_converter/
│   └── buck_conv.plecs              # Step-down converter, closed-loop PI voltage regulation
├── boost_converter/
│   ├── boost_conv.plecs             # Step-up converter, open loop (fixed duty cycle)
│   └── boost_conv_rectifier.plecs   # Diode-bridge rectifier + boost stage, PI regulated
└── cuk_converter/
    ├── cuk_conv.plecs               # Ćuk converter with sliding mode controller (C-Script)
    └── optimize_ga.py               # Genetic algorithm tuning the SMC gains via PLECS XML-RPC
```

---

## Requirements

- **PLECS Standalone** — the models use the *C-Script*, *Symmetrical PWM* and
  *Continuous PID Controller* blocks from the standard library.
- **Python 3.9+** with:

  ```bash
  pip install numpy scipy
  ```

  `xmlrpc.client` ships with the standard library.

---

## 1. Buck converter — `buck_converter/buck_conv.plecs`

A step-down converter regulated in closed loop by a continuous PI controller
feeding a symmetrical PWM modulator.

| Parameter | Value |
|---|---|
| Input voltage `V_dc` | 12 V |
| Output reference | 5 V |
| Inductor `L1` | 1 mH |
| Capacitor `C1` | 100 µF |
| Load `R1` | 10 Ω |
| Switching frequency | 10 kHz |
| Controller | PI, `kp = 0.1`, `ki = 60`, output saturated to [0, 1] |

**Theory.** In continuous conduction mode (CCM) the static conversion ratio is

```
Vout / Vin = D        ->   D = 5 / 12 ~= 0.417
```

and the ripples follow

```
dIL   = Vout (1 - D) / (L * fsw)
dVout = dIL / (8 * C * fsw)
```

The model also computes the periodic average of the input and output power, so
efficiency and averaged waveforms can be compared against the analytical CCM
predictions instead of only looking at the instantaneous switching waveforms.

---

## 2. Boost converter — `boost_converter/`

### 2.1 Open loop — `boost_conv.plecs`

The step-up stage driven directly by a pulse generator, used to observe the
uncontrolled behaviour: start-up transient, the right-half-plane zero of the
boost topology, and the inductor/capacitor ripple.

| Parameter | Value |
|---|---|
| Input voltage `V_dc` | 12 V |
| Inductor `L1` | 30 mH |
| Capacitor `C1` | 6 mF |
| Load `R1` | 10 Ω |
| Duty cycle | 0.5 (fixed) |
| Switching frequency | 10 kHz |

**Theory.** In CCM,

```
Vout / Vin = 1 / (1 - D)      ->   12 / (1 - 0.5) = 24 V
```

with `dIL = Vin * D / (L * fsw)` and an output ripple dominated by the capacitor
discharge during the on-time, `dVout = Iout * D / (C * fsw)`.

### 2.2 Rectifier + boost — `boost_conv_rectifier.plecs`

A full AC-DC chain: single-phase AC source → four-diode bridge rectifier → boost
converter → regulated DC bus. The power stage is wrapped in a `PLANT` subsystem
so the control loop can be designed against it as a single block.

| Parameter | Value |
|---|---|
| AC source | 12 V, 50 Hz |
| Rectifier | 4-diode full bridge (`D1`–`D4`) |
| Boost stage | `L1 = 30 mH`, `C1 = 6 mF`, `R1 = 10 Ω` |
| DC bus reference | 24 V |
| Switching frequency | 10 kHz |
| Controller | PI, `kp = 0.01`, `ki = 0.06`, duty clamped to [0.05, 0.85] with anti-windup |

The duty-cycle clamp matters here: the rectified input is a 100 Hz pulsating
voltage, so without saturation plus back-calculation anti-windup the integrator
winds up during the valleys of the rectified waveform.

---

## 3. Ćuk converter with sliding mode control — `cuk_converter/cuk_conv.plecs`

The main part of this repository. The Ćuk converter is a **fourth-order,
non-minimum-phase, inverting** topology — two inductors, two capacitors, and an
output voltage of opposite polarity to the input. A single linear loop around
such a plant is hard to tune and stays valid only near one operating point,
which makes it a good candidate for sliding mode control.

### 3.1 Power stage

| Parameter | Value |
|---|---|
| Input voltage `V_dc` | 24 V |
| Output target | −15 V |
| Input inductor `L1` | 65.5 µH |
| Coupling capacitor `C1` | 148 nF |
| Output inductor `L2` | 41 µH |
| Output capacitor `C2` | 93.95 nF |
| Load `R1` | 20 Ω |
| Switching / sampling frequency | 1 MHz (`Ts = 1 µs`) |

**Theory.** In CCM the Ćuk conversion ratio is

```
Vout / Vin = - D / (1 - D)     ->   D = 15 / (24 + 15) ~= 0.385
```

and in steady state the coupling capacitor holds

```
Vc1 = Vin + |Vout| = 24 + 15 = 39 V
```

which is exactly the voltage the equivalent control divides by — hence the
zero-division guard in the controller code.

### 3.2 Sliding surface

The controller is implemented as a **C-Script** block with six inputs
(`Vref`, `Vin`, `Vout`, `Vc1`, `iL1`, `iL2`) and one output, the duty cycle.
The sliding surface combines the output-voltage error, its integral, and both
inductor currents:

```
e = Vref - Vout
S = alpha * e + alpha_i * integral(e) - beta1 * iL1 - beta2 * iL2
```

The integral term removes the steady-state error a purely proportional surface
would leave, while the two current terms damp the resonance of the fourth-order
LC network.

### 3.3 Control law

The command is split into an equivalent (continuous) part and a discontinuous
(switching) part: `u = u_eq + u_sw`.

Equivalent control, obtained from `dS/dt = 0` on the averaged state equations:

```
         alpha*(de/dt) + alpha_i*e - (beta1/L1)*(Vin - Vc1) - (beta2/L2)*(-Vout)
u_eq = ---------------------------------------------------------------------------
                        Vc1 * ( beta1/L1 + beta2/L2 )
```

Discontinuous control, with a **boundary layer** of width `phi` replacing the
pure `sign(S)` in order to suppress chattering:

```
u_sw = Ksw * sat(S / phi)       with  sat(x) = x for |x| <= 1, sign(x) otherwise
```

The resulting duty cycle is clamped to `[0.05, 0.85]` before being sent to the
1 MHz symmetrical PWM modulator.

### 3.4 Tunable gains

Six parameters are exposed to the PLECS model workspace so they can be
overridden from outside the model:

| Gain | Role |
|---|---|
| `alpha` | Output voltage error weight |
| `alpha_i` | Integral action weight — kills steady-state error |
| `beta1` | `L1` current feedback weight |
| `beta2` | `L2` current feedback weight |
| `Ksw` | Discontinuous (switching) gain — reaching speed |
| `phi` | Boundary layer width — chattering vs. precision trade-off |

---

## 4. Automatic tuning with a genetic algorithm — `cuk_converter/optimize_ga.py`

Hand-tuning six coupled nonlinear gains is impractical, so the tuning is
delegated to an evolutionary search that uses the actual PLECS switching
simulation as its fitness evaluation.

### 4.1 How it works

1. The script connects to the PLECS **XML-RPC server** at
   `http://localhost:1080/RPC2`.
2. For each candidate gain vector it calls
   `server.plecs.simulate(MODEL_NAME, {'ModelVars': ...})`, which injects the six
   gains into the model workspace and runs a full switching simulation.
3. The output voltage trace is sliced to its steady-state window (`t >= 3 ms`)
   and two metrics are extracted:
   - **peak-to-peak ripple** — `ptp(v_steady)`
   - **steady-state error** — `|mean(v_steady) - (-15 V)|`
4. Both are folded into one scalar cost:

   ```
   J = 100 * ripple_pp + 500 * steady_state_error
   ```

   The weighting deliberately prioritises accuracy over ripple.
5. `scipy.optimize.differential_evolution` — a real-coded genetic/evolutionary
   algorithm using the `best1bin` strategy — searches the bounded parameter
   space. Invalid or diverging runs return a `1e6` penalty so the population
   never collapses onto a failed simulation.

### 4.2 Search space

| Gain | Lower bound | Upper bound |
|---|---|---|
| `alpha` | 0.01 | 0.50 |
| `alpha_i` | 10.0 | 150.0 |
| `beta1` | 0.01 | 0.50 |
| `beta2` | 0.01 | 0.50 |
| `Ksw` | 0.001 | 0.20 |
| `phi` | 0.05 | 0.50 |

GA settings: `maxiter = 10` generations, `popsize = 5` (30 individuals per
generation for 6 parameters), mutation `(0.5, 1.0)`, recombination `0.7`.

### 4.3 Running it

1. Open `cuk_converter/cuk_conv.plecs` in PLECS Standalone.
2. Enable the XML-RPC interface: **File → PLECS Preferences → General → Enable
   RPC interface**, port `1080`.
3. Make sure the scope feeding the simulation output returns the output voltage
   as the first signal — the script reads `results['Values'][0]`.
4. Launch the search:

   ```bash
   python cuk_converter/optimize_ga.py
   ```

Every evaluation prints its cost and gain vector so convergence can be followed
live, and the best set is printed at the end.

### 4.4 Results obtained so far

Three optimizers were run against the same cost function. All converge to a
comparable cost around `J ≈ 25.3`, but through very different gain
combinations — a good illustration of how flat and non-unique the
sliding-surface parameter space is.

| Optimizer | `alpha` | `alpha_i` | `beta1` | `beta2` | `Ksw` | `phi` | Cost `J` |
|---|---|---|---|---|---|---|---|
| Genetic / differential evolution | 0.06697 | 72.231 | 0.48376 | 0.01460 | 0.11368 | 0.17099 | 25.26 |
| Particle swarm | 0.11245 | 150.000 | 0.80000 | 0.50000 | 0.17728 | 0.50000 | 25.26 |
| Nelder–Mead | 0.020 | 11.0 | 0.022 | 0.023 | 0.032 | 0.410 | 25.3 |

The PSO solution sits exactly on several of its bounds (`alpha_i`, `beta1`,
`beta2`, `phi`), which suggests the cost keeps improving slightly outside the
box — there the bounds, not the algorithm, are the limiting factor.

---

## Roadmap

- [ ] Widen the GA bounds and raise the generation count, now that the cost surface is known to be flat
- [ ] Add explicit overshoot and settling-time terms to the cost function
- [ ] Load-step and input-step robustness tests on the optimized SMC gains
- [ ] Compare the SMC against a linear PI designed on the small-signal Ćuk model
- [ ] Extend the sliding mode approach to the buck and boost stages for a side-by-side comparison
- [ ] Export scope traces and add the plots to the repository

---

## Notes

`.plecs` files are plain-text model descriptions, so they diff and version
reasonably well. The Ćuk model also carries the optimization results as
annotations placed directly on the schematic, next to the C-Script block.
