# Booster Landing

Can a reusable first stage falling at 280 m/s land on a pad at walking speed,
when its engine cannot throttle below 40 %? A planar 3-DOF model of a Falcon
9-class stage is built in Simulink and closed with an attitude loop on the
engine gimbal, a vertical guidance law that decides when to light the engine
and how hard to push, and a lateral controller that steers it onto the pad.
Every model is checked against closed-form solutions before it is used, and
the full landing is evaluated on 1000 Monte Carlo runs with thrust, drag,
mass, wind and initial-state errors the guidance does not know about.

![Monte Carlo touchdown dispersion](figures/montecarlo_touchdown.png)

*1000 guided landings with dispersed thrust, Isp, mass, drag, wind and initial
state: touchdown state with its 99 % covariance ellipse (top left), lateral
miss, lateral speed and propellant left.*

**Key results**

- **1000 of 1000** dispersed landings meet every touchdown requirement (95 %
  interval 99.6–100 %), with 99th percentiles of **3.8 m** from the pad,
  **1.3 m/s** vertical and 0.6 m/s lateral speed; with every dispersion
  doubled, 95 % still land, and the first requirement to fail is the lateral
  speed, driven by wind.
- A constant-deceleration guidance law in feedback lands at **1.2 m/s**; adding
  a drag feedforward cuts the burn from 34 s to **25 s** and leaves **1686 kg**
  of propellant instead of 635, and an integral term keeps it landing with a
  thrust 10 % weaker than commanded.
- With the same gimbal and the same peak effort, an **LQR** on the coupled
  lateral and pitch states lands **0.004–0.19 m** from the pad against
  0.4–1.6 m for a cascaded PD, and closes a 30 m error 5–10 s sooner.

## Contents

1. [Physics](#physics)
2. [Method](#method)
3. [Validation](#validation)
4. [Results](#results)
5. [Discussion and limitations](#discussion-and-limitations)
6. [Getting started](#getting-started)
7. [Repository layout](#repository-layout)
8. [Tests](#tests)
9. [Roadmap](#roadmap)
10. [References](#references)
11. [License](#license)

## Physics

### Equations of motion

The stage moves in a vertical plane: horizontal position $`x`$, altitude
$`z`$ above the pad, pitch $`\theta`$ measured from the vertical and positive
when the stage leans towards $`+x`$, and mass $`m`$. The engine produces a
thrust $`T`$ along the body axis, turned by the gimbal angle $`\delta`$, at a
distance $`l`$ below the centre of mass:

```math
m\ddot x = T\sin(\theta + \delta) - D_x, \qquad m\ddot z = T\cos(\theta + \delta) - mg - D_z
```

```math
I\ddot\theta = -T\,l\sin\delta, \qquad \dot m = -\frac{T}{I_{sp}\,g_0}
```

The minus sign in the pitch equation is the inverted pendulum: thrust applied
below the centre of mass pushes the tail one way and turns the nose the
other, so to tilt the stage towards $`+x`$ the gimbal must first turn towards
$`-x`$. Drag acts against the velocity relative to the air, with a constant
horizontal wind $`w`$ and an exponential atmosphere of scale height $`H`$:

```math
\mathbf D = \tfrac12\,\rho(z)\,C_D A\,\lVert\mathbf v_{rel}\rVert\,\mathbf v_{rel}, \qquad \mathbf v_{rel} = (\dot x - w,\ \dot z), \qquad \rho(z) = \rho_0\,e^{-z/H}
```

These are seven first-order states $`(x, z, \dot x, \dot z, \theta, \dot\theta, m)`$,
one Simulink integrator each; a single MATLAB Function computes their
derivatives, and the engine is cut when the mass reaches the dry mass.

### Why the landing is hard: the engine cannot hover

At its minimum throttle the engine still produces $`T_{min} = 338`$ kN, while
the empty stage weighs $`m_{dry} g = 251`$ kN: $`T_{min}/(m_{dry} g) = 1.35 \gt 1`$.
Even at minimum thrust the stage accelerates upwards, so it cannot stop in the
air and wait. It must reach zero velocity exactly when it reaches the ground,
the manoeuvre known as a hoverslam [6]. Light too early and it stops in
mid-air, climbs and falls back with empty tanks; too late and it hits the pad.

For a full-thrust burn the ignition altitude has a closed form. With
$`c = I_{sp} g_0`$ and $`\dot m = T_{max}/c`$, the rocket equation with
gravity gives the speed during the burn, and the burn time $`t_b`$ is the time
at which a stage falling at $`v_0`$ stops:

```math
0 = -v_0 - g\,t_b + c\ln\frac{m_0}{m_0 - \dot m\,t_b}
```

Integrating the velocity once more, with $`m_b = m_0 - \dot m\,t_b`$, gives the
distance covered during the burn, which is the ignition altitude:

```math
h = v_0 t_b + \tfrac12 g t_b^2 - c\left(t_b - \frac{m_b}{\dot m}\ln\frac{m_0}{m_b}\right)
```

From 280 m/s this is 2180 m and a 15.0 s burn, drag neglected. It is the
burn that uses the least propellant and keeps no margin at all: a thrust or
mass 3 % off ends in a 75 m/s impact.

### Pitch dynamics and dynamic inversion

For small gimbal angles the pitch equation is a double integrator whose gain
depends on the throttle:

```math
\ddot\theta = -K\,\delta, \qquad K = \frac{T\,l}{I}
```

$`K`$ changes by a factor of 2.5 between $`T_{min}`$ and $`T_{max}`$. Inverting
it [8], the controller chooses a pitch acceleration $`\ddot\theta_{cmd}`$ and
asks the gimbal for the angle that produces it:

```math
\delta_{cmd} = -\frac{I}{l\,\max(T, T_{min})}\,\ddot\theta_{cmd}
```

so that the loop sees $`\ddot\theta = \ddot\theta_{cmd}`$ at every throttle
setting; the $`\max`$ guards the division when the engine is off.

### Constant-deceleration guidance

A stage at altitude $`z`$ descending at speed $`u = -\dot z`$ reaches a target
speed $`u_t`$ at the ground under the constant deceleration

```math
a_{req} = \frac{u^2 - u_t^2}{2z}
```

from $`u^2 = u_t^2 + 2 a z`$. Differentiating with $`\dot z = -u`$ and
$`\dot u = -a`$, where $`a`$ is the deceleration actually achieved,

```math
\dot a_{req} = \frac{u}{z}\,\left(a_{req} - a\right)
```

If the stage decelerates exactly as required, $`a_{req}`$ stays constant. If
it decelerates more, for instance because drag helps, $`a_{req}`$ shrinks and
the burn stretches; if it decelerates less, $`a_{req}`$ grows, at a rate
$`u/z`$ that diverges near the ground. These two effects shape the guidance
law of the next section.

### Lateral motion

To move sideways the stage must lean. For small angles, and with the pitch
loop inverted so that $`\ddot\theta = u`$,

```math
\ddot x \approx a\,\theta, \qquad \ddot\theta = u, \qquad a = T/m
```

With the scaled states $`(x, \dot x, a\theta, a\dot\theta)`$ and the input
$`a u`$ this is a chain of four integrators, and slowly varying $`a`$ only
rescales it. A linear law on the four states, written as a pitch reference
for the attitude loop, then takes the form

```math
\theta_{ref} = -\frac{K_1 x + K_2 \dot x}{a} + K_3\,\theta + K_4\,\dot\theta
```

which serves both controllers compared in the results.

## Method

### Vehicle

A Falcon 9-class first stage in its landing burn with one Merlin 1D lit. The
values marked *public* come from [1–3]; the others are derived and later
dispersed in the Monte Carlo.

| Quantity | Value | Source |
|---|---|---|
| maximum thrust $`T_{max}`$ | 845 kN | public [1] |
| minimum throttle | 40 % (338 kN) | public [1], sources range 40–57 % |
| specific impulse $`I_{sp}`$ | 282 s | public [1] |
| dry mass $`m_{dry}`$ | 25.6 t | public [3], with legs and grid fins |
| mass at ignition $`m_0`$ | 32 t | rocket equation for a 600 m/s braking budget |
| diameter, length | 3.66 m, 42 m | public [2] |
| drag coefficient $`C_D`$ | 0.8 | estimate, blunt body flying base first |
| gimbal-to-CoM distance $`l`$ | 16 m | estimate, 6 t of engines at the base |
| pitch inertia $`I`$ | 4.8×10⁶ kg m² | estimate, rod plus parallel-axis terms |
| gimbal limit, rate, lag | 5 deg, 10 deg/s, 0.1 s | assumed |
| initial state | 5 km, −280 m/s | close to the 320 m/s terminal speed at 5 km |

Every value lives in `params.m`; the Simulink blocks read the struct `p` and
contain no numbers.

### Models

| Model | Content |
|---|---|
| `plant_vertical.slx` | the vertical channel, $`\ddot z = T/m - g`$, built first to learn Simulink |
| `plant_3dof.slx` | the seven-state plant; inputs $`T`$ and $`\delta`$, seven outputs |
| `attitude_ctrl.slx` | Subsystem Reference: PD on $`\ddot\theta_{cmd}`$, the inversion, the gimbal servo |
| `attitude_loop.slx` | the plant (Model block) closed by `attitude_ctrl`, for step 2 |
| `landing.slx` | the plant, `attitude_ctrl`, the vertical guidance and the lateral control |

The plant is referenced, not copied, so one correction reaches every model.

### Attitude loop

A PD on the commanded pitch acceleration, $`\ddot\theta_{cmd} = K_p(\theta_{ref} - \theta) - K_d\,q`$,
inverted as above, gives the closed loop $`s^2 + K_d s + K_p`$ at every
throttle. $`\omega_n = 1`$ rad/s and $`\zeta = 0.7`$ ($`K_p = 1`$ s⁻²,
$`K_d = 1.4`$ s⁻¹): a 5 deg error then asks for 0.087 rad/s², within the
0.098 rad/s² the gimbal can give at minimum thrust. There is no integral term:
the model has no constant pitch disturbance for it to reject.

The gimbal is a first-order servo, $`\dot\delta = \mathrm{sat}_{\pm 10^\circ/\mathrm s}\big((\delta_{cmd} - \delta)/\tau\big)`$
with $`\delta_{cmd}`$ clipped to ±5 deg. Simulink's Rate Limiter block was
tried first and rejected: in continuous time it passes its first input
through, so the gimbal jumped to its limit at $`t = 0`$.

### Vertical guidance

The engine lights when $`a_{req}`$ reaches a design deceleration
$`a^* = \eta\,(T_{max}/m_0 - g)`$, a fraction $`\eta`$ of the maximum available.
After ignition the thrust command is

```math
T = \mathrm{sat}_{[T_{min},\,T_{max}]}\Big(m\,(g + a_{req} + a_I) - \hat D\Big), \qquad \hat D = \tfrac12\rho(z)\,\hat C_D A\,\dot z^2, \qquad a_I = K_i\!\int (a_{req} - a^*)\,dt
```

| Choice | Value | Reason |
|---|---|---|
| $`\eta`$ | 0.7 | about 20 % of throttle left for errors at ignition; with $`\eta = 1`$ a 3 % thrust or mass error crashes at 75 m/s |
| target touchdown speed $`u_t`$ | 1 m/s | keeps $`a_{req}`$ finite at the end; the requirement is 2 m/s |
| drag feedforward $`\hat D`$ | nominal $`\hat C_D = 0.8`$ | without it drag over-brakes the stage and $`a_{req}`$ keeps shrinking: a 34 s burn and 635 kg left instead of 25 s and 1686 kg |
| integral term $`K_i`$ | 1 s⁻¹ | with the feedforward alone a thrust 5 % low lands at 2.4 m/s, because $`a_{req}`$ grows like $`u/z`$; 0.3 s⁻¹ relights the engine with $`C_D = 1.2`$ |
| ignition logic | relay, on at $`a^*`$, off at $`0.1\,a^*`$ | the hysteresis prevents the engine from switching on and off around $`a^*`$ |

The integral only runs while the engine is lit. In the nominal case
$`a_{req}`$ then stays at $`a^* = 11.6`$ m/s² for the whole burn, which is the
planned constant deceleration.

### Lateral control

The two controllers share the actuator, the scenarios and the metrics; they
differ only in the four gains of $`\theta_{ref}`$:

| | $`K_1`$ | $`K_2`$ | $`K_3`$ | $`K_4`$ | Design |
|---|---|---|---|---|---|
| cascaded PD | 0.040 | 0.360 | 0 | 0 | outer loop on $`x`$ at 0.2 rad/s, one fifth of the attitude loop, $`\zeta = 0.9`$ |
| LQR | 0.043 | 0.306 | 0.035 | 0.011 | Bryson weights [4] (10 m, 4 m/s, 3 deg), $`R`$ set for the same peak effort as the PD |

The LQR is solved on the scaled chain and converted to the same form: from
$`u = -K_n s`$ and $`u = K_p(\theta_{ref} - \theta) - K_d q`$,
$`K = [K_{n1},\ K_{n2},\ K_p - K_{n3},\ K_d - K_{n4}]/K_p`$. Textbook weights
first produced gains asking for 25 times the gimbal authority, so the
comparison is made at equal peak effort. With the engine off the lateral
control asks for $`\theta = 0`$, and $`\theta_{ref}`$ is limited to ±10 deg.

### Monte Carlo

Each landing draws, from Gaussian distributions with seed 42:

| Dispersion | 1σ |
|---|---|
| thrust actual/commanded | 2 % |
| $`I_{sp}`$ | 1 % |
| mass at ignition | 300 kg |
| $`C_D`$ | 15 % |
| horizontal wind | 5 m/s |
| lateral position, lateral speed | 20 m, 2 m/s |
| vertical speed | 10 m/s |
| tilt | 1 deg |

The guidance and the controllers keep the nominal values. A landing succeeds
if it touches down within 120 s with $`\lvert\dot z\rvert \lt 2`$ m/s,
$`\lvert\dot x\rvert \lt 1`$ m/s, $`\lvert\theta\rvert \lt 5`$ deg,
$`\lvert x\rvert \lt 10`$ m and propellant left. The success rate is given
with the 95 % Wilson interval [7], which stays meaningful at 100 %:

```math
\frac{\hat p + \frac{z^2}{2n} \pm z\sqrt{\frac{\hat p(1 - \hat p)}{n} + \frac{z^2}{4n^2}}}{1 + \frac{z^2}{n}}, \qquad z = 1.96
```

The 99 % ellipse is the covariance ellipse of the touchdown state
$`(x, \dot z)`$ scaled by $`\sqrt{\chi^2_{2}(0.99)} = 3.03`$. The stress test
repeats the Monte Carlo with every 1σ multiplied by 1 to 4, 300 landings each.

## Validation

No model is used before it reproduces closed-form solutions, each of which
switches on a single term of the dynamics.

![Plant against closed-form solutions](figures/plant_validation.png)

| Case | Closed form | Maximum error |
|---|---|---|
| free fall | $`z = z_0 + \dot z_0 t - \tfrac12 g t^2`$ | 6×10⁻¹² m (round-off: ode45 integrates a quadratic exactly) |
| constant thrust, upright | rocket equation with gravity | 4.5×10⁻¹¹ m (solver tolerance 10⁻¹⁰) |
| fixed tilt, no gravity | rocket equation along $`\theta_0`$ | 7.2×10⁻¹² m |
| constant gimbal | $`\theta = \theta_0 - \tfrac12 (T l \sin\delta / I)\,t^2`$ | 6.7×10⁻¹⁶ rad |
| unpowered fall, constant density | $`\dot z = -v_t\tanh\!\big(g t / v_t + \mathrm{artanh}(\lvert\dot z_0\rvert / v_t)\big)`$ | 1.6×10⁻¹² m/s |
| body in a wind, no gravity | $`\dot x - w = (\dot x_0 - w)/(1 + k_d\lvert\dot x_0 - w\rvert t)`$ | 9×10⁻¹⁴ m/s |
| hoverslam ignition | lit at Tmax at $`h`$, the stage is at rest at $`t_b`$ | 4×10⁻¹⁰ m, 1.3×10⁻⁴ m/s |

The tests were themselves checked by injecting bugs into a reference model:
sine and cosine swapped, a flipped torque or drag sign, a missing ½, gravity
missing from the thrust command, a flipped lateral sign. Each makes the test
written for it fail.

The baselines of the results are the open-loop descents (no burn, and full
thrust from 5 km), the full-thrust hoverslam, and, for lateral control, the
cascaded PD flown with the same actuator and the same peak effort as the LQR.

## Results

### Without guidance, every descent crashes

![Open-loop descents](figures/openloop_descent.png)

With no burn the stage hits the ground after 17 s at **277 m/s**. With full
thrust from 5 km it stops at 3.2 km, climbs back to 4.9 km, runs out of
propellant after 21 s and lands at **215 m/s**.

### Attitude loop

![Attitude loop response](figures/attitude_response.png)

| | Value |
|---|---|
| phase margin, crossover | 56 deg at 1.53 rad/s (gimbal lag included) |
| delay margin | 0.64 s |
| 3 deg tilt, within 0.5 deg | 2.3 s at $`T_{max}`$, 2.5 s at $`T_{min}`$ |
| 10 deg step at $`T_{min}`$, gimbal saturated | 3.1 s |

The responses at full and minimum thrust overlap the ideal second-order
curve: the inversion removes the throttle dependence. The gain margin is
infinite only because the gimbal is modelled as a first-order lag; the delay
margin is the meaningful number. The margins are in
[`attitude_margins.png`](figures/attitude_margins.png).

### Guided landing

![Nominal guided landing](figures/landing_nominal.png)

The engine lights at 3600 m, the vertical speed falls linearly, which is the
constant deceleration, and the stage touches down at **1.20 m/s** after a
**25 s** burn with **1686 kg** of propellant left. The thrust rises during the
burn, from 400 to 590 kN: early on, at 290 m/s, drag does much of the braking.

| Errors unknown to the guidance | Ignition | Touchdown | Propellant left |
|---|---|---|---|
| none | 3600 m | 1.20 m/s | 1686 kg |
| thrust −5 % | 3600 m | 1.20 m/s | 1692 kg |
| thrust −10 % | 3600 m | 1.20 m/s | 1699 kg |
| $`C_D`$ = 0.4 | 3912 m | 1.33 m/s | 1071 kg |
| $`C_D`$ = 1.2 | 3117 m | 1.15 m/s | 2263 kg |
| $`I_{sp}`$ −5 % | 3600 m | 1.20 m/s | 1460 kg |
| thrust −5 %, $`C_D`$ 0.4, $`I_{sp}`$ −5 % | 3912 m | 1.38 m/s | 818 kg |

The engine lights once in every case and the thrust stays within its limits
([`landing_dispersions.png`](figures/landing_dispersions.png)).

### Lateral control: cascaded PD against LQR

![Lateral comparison](figures/lateral_comparison.png)

From four lateral errors (±30 m with ±2 m/s, 50 m, 3 m/s), each nominal and
with combined errors, both controllers meet every requirement, where the
uncontrolled stage misses the pad by 50–79 m:

| | Cascaded PD | LQR |
|---|---|---|
| miss at touchdown | 0.4–1.6 m | **0.004–0.19 m** |
| time to $`\lvert x\rvert \lt 1`$ m after ignition | 21–26 s | **15–18 s** |
| peak tilt | 6.5–8.8 deg | 6.2–9.2 deg |
| peak gimbal, propellant | same | same |

The first seconds after ignition are identical, because both controllers
drive the gimbal to its limit; the difference is in the tail, where the LQR's
small feedback of $`\theta`$ and $`q`$ into the lateral law coordinates the two
loops that the cascade keeps apart by bandwidth.

### Monte Carlo and stress test

On 1000 dispersed landings (figure at the top) all succeed: 95 % interval
99.6–100 %. At the 99th percentile the stage lands 3.84 m from the pad at
1.28 m/s vertical and 0.59 m/s lateral speed, tilted 0.15 deg; the least
propellant left is 582 kg. The spread of the miss settles at 1.35 m after
300–500 runs ([`montecarlo_convergence.png`](figures/montecarlo_convergence.png)).

![Stress test](figures/stress_test.png)

| Dispersions | ×1 | ×1.5 | ×2 | ×2.5 | ×3 | ×4 |
|---|---|---|---|---|---|---|
| success | 100 % | 99 % | 95 % | 83 % | 80 % | 57 % |

At every level the first requirement to fail is the lateral speed at
touchdown, followed by the miss distance; running out of propellant appears
only from twice the nominal dispersions. Among the early failures the most
extreme draw is most often the wind.

## Discussion and limitations

The guidance holds because it is in feedback: it does not need to know the
thrust, the drag or the Isp, only the measured altitude, speed and mass. What
mattered most was not making it land but making it land well: the first
version touched down safely while wasting a ton of propellant, and only
looking at how it landed showed that drag was over-braking it. Removing that
waste exposed a sensitivity to thrust errors that the waste had been hiding,
which an integral term fixed. The dynamic inversion is what lets one set of
gains work from full to minimum thrust, in pitch and laterally.

The comparison of step 4 needed a change of scope. On pitch alone the LQR of a
double integrator is a PD, so the comparison was moved to the coupled lateral
and pitch states, where the LQR's advantage is real but modest: both are four
gains on a linear chain, and the LQR chooses them with weights in physical
units instead of separating bandwidths.

The stress test locates the weak point: the lateral loop is proportional only,
so a constant wind leaves a residual drift, and in the worst nominal cases
the gimbal already sits at its 5 deg limit. An integral term on the lateral
error, or more gimbal range, would be the next step.

The model leaves out:

- out-of-plane motion: the 99 % ellipse is drawn in the touchdown state
  $`(x, \dot z)`$, not on the ground;
- an angle-of-attack-dependent drag and the aerodynamic pitching moment;
- throttle dynamics and the engine's response time;
- navigation: the controllers see the true state, with no sensor noise or
  estimator;
- the change of inertia and centre of mass as propellant burns;
- gusts and a wind profile with altitude.

None of this is new. Hoverslam landings fly operationally [6], and the
fuel-optimal problem with thrust bounds is solved in flight by convex
optimisation [5]; the guidance here is a simple feedback law, not an optimal
one. What the project adds is a model checked term by term, a guidance law
whose inefficiency and its fix are both measured, a like-for-like comparison
of two lateral controllers, and a dispersion analysis that says where the
design breaks.

## Getting started

### Installation

```bash
git clone https://github.com/LoreMonti/Booster_Landing.git
cd Booster_Landing
```

MATLAB with Simulink, the Control System Toolbox and the Statistics and
Machine Learning Toolbox. The results were produced with MATLAB R2026b
(26.2) on macOS. No C compiler is needed: every model runs in Normal mode.

### Usage

From the repository root in MATLAB:

```matlab
run_all              % regenerate every study and figure (about 10 minutes)
runtests('tests')    % the 26 tests
```

| Script | What it does |
|---|---|
| `run_all.m` | runs every study below and writes `figures/` |
| `studies/study_plant_validation.m` | plant against the closed-form solutions |
| `studies/study_openloop.m` | descents with no burn and with full thrust |
| `studies/study_attitude.m` | margins and time responses of the pitch loop |
| `studies/study_landing.m` | guided landings, nominal and with single errors |
| `studies/study_lateral.m` | cascaded PD against LQR |
| `studies/study_montecarlo.m` | 1000 dispersed landings, convergence |
| `studies/study_stress.m` | dispersions scaled 1 to 4 times |

As a library:

```matlab
addpath('models', 'utils');
p = params();
[h, tb] = hoverslam_ignition(p, 280);              % full-thrust ignition altitude
fprintf('Light at %.0f m, burn for %.1f s\n', h, tb);

[draws, out] = dispersed_landings(p, 20, 1, 1);    % 20 dispersed guided landings
fprintf('%d of 20 landed; worst miss %.2f m\n', sum(out.success), max(abs(out.x)));
```

## Repository layout

```
Booster_Landing/
├── README.md, ROADMAP.md, LICENSE
├── params.m                    # every parameter, dispersion and requirement
├── run_all.m                   # regenerates every figure
├── models/
│   ├── plant_vertical.slx      # vertical channel (step 1a)
│   ├── plant_3dof.slx          # planar 3-DOF plant, inputs T and delta
│   ├── attitude_ctrl.slx       # PD, dynamic inversion, gimbal servo (Subsystem Reference)
│   ├── attitude_loop.slx       # plant + attitude controller (step 2)
│   └── landing.slx             # plant + attitude + guidance + lateral control
├── utils/
│   ├── simulate_plant.m        # one open-loop run of the plant
│   ├── hoverslam_ignition.m    # closed-form full-thrust ignition altitude
│   └── dispersed_landings.m    # draws dispersions and flies landings
├── studies/                    # one script per study, each writes its figures
├── tests/                      # MATLAB unit tests, one class per model
├── figures/                    # the figures of this README
└── results/                    # raw Monte Carlo draws, git-ignored
```

The models are Simulink files, which GitHub cannot display, so every number
in this README comes from a script in `studies/` that rebuilds it from
`params.m`. The plant and the attitude controller are referenced by the
larger models rather than copied, so a correction reaches every model at
once.

## Tests

```matlab
runtests('tests')
```

26 tests pin the invariants that would catch a silent error:

- **TestPlantVertical** (2): free fall and constant thrust against the rocket
  equation; a wrong gravity sign fails both.
- **TestPlant3DOF** (5): the term-by-term closed forms of the validation table;
  swapped sine and cosine, a flipped torque or drag sign, a missing ½ in the
  drag or drag on $`\dot x`$ instead of $`\dot x - w`$ each fail one.
- **TestAttitudeLoop** (4): tilt recovery at $`T_{max}`$ and $`T_{min}`$ along
  the ideal response, gimbal angle and rate limits on a saturating step, phase
  margin above 45 deg; a wrong thrust in the inversion or a missing rate limit
  fails them.
- **TestLanding** (10): the hoverslam altitude against the plant; every
  requirement, thrust limits and a single ignition, nominally and under six
  unknown errors; a nominal burn under 27 s with over 1500 kg left, which fails
  without the drag feedforward.
- **TestLateral** (5): from 30 m and 2 m/s off the pad, the uncontrolled stage
  misses by more than 50 m and both gain sets meet every requirement, nominally
  and with combined errors; a flipped sign or a missing $`1/a`$ fails all four.

## Roadmap

How the project was built step by step, with what went wrong at each step, is
in **[ROADMAP.md](ROADMAP.md)**.

## References

1. Wikipedia, *SpaceX Merlin*: Merlin 1D sea-level thrust, throttle range and
   specific impulse.
2. Wikipedia, *Falcon 9 Full Thrust*: first-stage diameter and length.
3. Spaceflight101, *Falcon 9 v1.1 & F9R Launch Vehicle Overview*: first-stage
   inert mass with landing hardware.
4. A. E. Bryson & Y.-C. Ho, *Applied Optimal Control*, Hemisphere (1975)
5. B. Açıkmeşe & S. R. Ploen, *Convex Programming Approach to Powered Descent
   Guidance for Mars Landing*, J. Guid. Control Dyn. **30**, 1353 (2007)
6. L. Blackmore, *Autonomous Precision Landing of Space Rockets*, The Bridge
   **46**, 15 (2016)
7. E. B. Wilson, *Probable Inference, the Law of Succession, and Statistical
   Inference*, J. Am. Stat. Assoc. **22**, 209 (1927)
8. D. Enns, D. Bugajski, R. Hendrick & G. Stein, *Dynamic Inversion: an
   Evolving Methodology for Flight Control Design*, Int. J. Control **59**, 71
   (1994)

## License

MIT License, Lorenzo Monti. See [LICENSE](LICENSE).
