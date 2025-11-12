# Surface-Normal Estimator and Hybrid Controller Comparison

## Executive Summary

**Short answer:** The controller itself isn't the novelty. The paper's contribution is an online **surface-normal estimator that removes friction bias from the force sensor** and then drives a standard hybrid force/motion loop with that corrected normal, plus a null-space orientation optimizer. That's the piece your "velocity-only hybrid controller" doesn't do right now—you're implicitly taking the **raw force direction as the normal**, which is wrong whenever you're sliding (the force tilts by ≈atan μ toward the velocity). Their math shows how to separate that tangential friction from the normal component in real time and feed the clean normal into the projections and the redundancy resolution.

Below is a precise, equation-level comparison.

---

## 1) What problem are they actually solving?

When you slide on a surface with Coulomb friction μ, the wrench at the tool is:

$$\mathbf{f}_s = N\mathbf{n}_{\text{true}} - \mu N\hat{\mathbf{v}}_t$$

so the **direction** of the measured force is:

$$\frac{\mathbf{f}_s}{|\mathbf{f}_s|} = \frac{\mathbf{n}_{\text{true}} - \mu\hat{\mathbf{v}}_t}{\sqrt{1+\mu^2}}$$

which is tilted by an angle $\delta=\arctan\mu$ away from the true surface normal in the $(\mathbf{n}, \hat{\mathbf{v}}_t)$ plane. If you treat $\mathbf{f}_s/|\mathbf{f}_s|$ as "the normal," you'll project motion/force in the wrong directions and regulate the wrong normal load. That's exactly the failure mode their estimator fixes. (See the friction-compensation block and the sketch in **Fig. 3 on page 4** and **Algorithm 1 on page 5**.)

---

## 2) What's novel in the paper (mathematically)?

### 2.1 Friction-compensated surface-normal estimation (core novelty)

At every control step they:

1. **Project the sensed force** $\mathbf{f}_s$ onto the robot velocity direction $\hat{\mathbf{v}}$ and its null space:
    
    $$\Omega_v=\hat{\mathbf{v}}(\hat{\mathbf{v}}^T\hat{\mathbf{v}})^{-1}\hat{\mathbf{v}}^T$$
    
    $$\mathbf{f}_v=\Omega_v \mathbf{f}_s$$
    
    $$\mathbf{f}_\perp=(I-\Omega_v)\mathbf{f}_s$$
    
    (**Eq. (28)–(31), p.4**).
    
2. **Estimate Coulomb friction:**
    
    $$\mu_k=\frac{|\mathbf{f}_v|}{|\mathbf{f}_\perp|}$$
    
    then low-pass it by a weighted moving average $\bar{\mu}$ (**Eq. (32), (38), p.4–5**).
    
3. **Build a feedforward friction vector:**
    
    $$\mathbf{f}_\tau = -\bar{\mu}|\mathbf{f}_\perp|\frac{\hat{\mathbf{v}}}{|\hat{\mathbf{v}}|}$$
    
    and subtract it from the sensor reading to get the **corrected surface-normal force:**
    
    $$\hat{\mathbf{f}}_n = \mathbf{f}_s - \mathbf{f}_\tau$$
    
    $$\hat{\mathbf{n}}_{\text{surf}}=\frac{\hat{\mathbf{f}}_n}{|\hat{\mathbf{f}}_n|} \quad \text{(their normal)}$$
    
    (**Eq. (35)–(37), p.4**; steps are laid out explicitly in **Algorithm 1, p.5**).
    

That's the key: they don't assume the force direction is the normal; they **remove the tangential friction component first**, then normalize.

### 2.2 Use that normal inside a standard hybrid projector

They form the motion/force projectors:

$$\Omega_f = N(N^TN)^{-1}N^T, \quad \Omega_m = T(T^TT)^{-1}T^T=I-\Omega_f$$

where $N$ is the force direction (their $\hat{\mathbf{n}}_{\text{surf}}$) and $T$ spans the motion plane (**Eq. (9)–(10), p.3; diagram in Fig. 2, p.2**). They then combine a resolved-rate velocity and an admittance term, project each into its subspace, and send the summed Cartesian velocity through a Jacobian pseudo-inverse (**Eq. (11), (14)–(27), p.3**). This part is classical; the twist is feeding **the corrected normal** into the projectors every cycle.

### 2.3 Null-space orientation optimization

They add a redundancy term $\rho=\alpha\nabla_\Phi g$ that maximizes:

$$g(\Phi)=\cos\gamma=\hat{\mathbf{n}}_{\text{surf}}^T(-\mathbf{n}_{ee})$$

i.e., they align the tool axis to the updated surface normal (derive $\nabla_\Phi g = -(J_{ee,\omega})^T[\mathbf{n}_{ee}]_\times \hat{\mathbf{n}}_{\text{surf}}$; **Eq. (39)–(49), p.5–6; sketch in Fig. 4, p.4**). This keeps the probe oriented correctly while sliding.

### 2.4 Evidence

In simulation and hardware, they show smaller path error when the normal changes along the path; e.g., on dome/sine shapes the difference between target and actual is 0.3–1.0 mm, with ~5% average improvement vs. no normal updates (**Fig. 8 and text on page 7**).

---

## 3) Your plan, in math (what you described)

You propose a ROS 2 velocity-space hybrid controller that does:

- **Normal from the force direction:** $\tilde{\mathbf{n}}=\mathbf{f}_s/|\mathbf{f}_s|$
    
- **Tangential direction:** Project a user hint $\mathbf{t}_{\text{hint}}$ into the plane orthogonal to $\tilde{\mathbf{n}}$:
    
    $$\hat{\mathbf{t}}=\frac{(I-\tilde{\mathbf{n}}\tilde{\mathbf{n}}^T)\mathbf{t}_{\text{hint}}}{|(I-\tilde{\mathbf{n}}\tilde{\mathbf{n}}^T)\mathbf{t}_{\text{hint}}|}$$
    
- **Velocity law:** Command:
    
    $$\mathbf{v}=\underbrace{k_t s_{\text{remain}} \hat{\mathbf{t}}}_{\text{tangential}} + \underbrace{k_n (F_{\text{des}}-\mathbf{f}_s \cdot \tilde{\mathbf{n}})\tilde{\mathbf{n}}}_{\text{normal regulation}}$$
    
- **IK:** Map to joints with damped least squares:
    
    $$\dot{\mathbf{q}}=J^T(JJ^T+\lambda^2 I)^{-1}\mathbf{v}$$
    
    You explicitly call this a "velocity-only hybrid controller," subscribe to an FT topic, project motion onto the plane orthogonal to the **measured** normal, and convert to $\dot{\mathbf{q}}$ each cycle (design/state-machine/IK details: **plan.md** and **README**).
    

Crucially, there's **no explicit friction compensation and no independent estimation of the surface normal** beyond "point the normal where the force points."

---

## 4) Head-to-head: where they differ from your "basic" hybrid controller

|Aspect|Your plan|Paper|
|---|---|---|
|**Normal direction**|$\tilde{\mathbf{n}}=\mathbf{f}_s/\|\mathbf{f}_s\|$. Biased by friction (tilt $\delta=\arctan\mu$).|Remove friction component first; define $\hat{\mathbf{n}}_{\text{surf}}=(\mathbf{f}_s-\mathbf{f}_\tau)/\|\cdot\|$. (**Eq. (28)–(37)**).|
|**Friction handling**|None; friction is baked into "normal."|Online estimate $\mu_k=\|\mathbf{f}_v\|/\|\mathbf{f}_\perp\|$, filtered to $\bar{\mu}$; friction feed-forward $\mathbf{f}_\tau$ (**Eq. (32),(35),(38)**).|
|**Projection**|Implicitly $\Omega_f=\tilde{\mathbf{n}}\tilde{\mathbf{n}}^T$, $\Omega_m=I-\Omega_f$.|Same structure, but using **corrected** $\hat{\mathbf{n}}_{\text{surf}}$ (**Eq. (9)–(10)**).|
|**Controller core**|Cartesian velocity + simple normal regulation; DLS IK.|Same resolved-rate/admittance split, projected and summed; then IK (**Fig. 2; Eq. (11),(14)–(27)**).|
|**Redundancy / orientation**|None specified; orientation may drift.|Null-space bias aligns tool axis to $\hat{\mathbf{n}}_{\text{surf}}$ ($\rho=\alpha\nabla_\Phi g$). (**Eq. (39)–(49)**; **Fig. 4**).|
|**Contact-point error model**|Track a 5 cm tangential displacement from a captured start pose; regulate force.|Explicit error with an offset $d$: $\mathbf{e}=(\mathbf{x}_{cnt,des}-\hat{\mathbf{x}}_{cnt})-\hat{\mathbf{n}}_{\text{surf}}d$, updated as $\hat{\mathbf{n}}_{\text{surf}}$ changes (**Eq. (5)–(8)**).|
|**Evidence**|Design docs only.|Measured accuracy gains on curved paths; ~0.3–1.0 mm improvements; ≈5% average (**Fig. 8 & text, p.7**).|

**Bottom line:** your loop and theirs share the same bones (resolved-rate + projections + IK). Their "novelty" is a **practical estimator** that fixes the single assumption that usually breaks these controllers in real manufacturing: **"force direction = surface normal."** Once you're sliding, that assumption is wrong by ~11° at μ=0.2 and ~27° at μ=0.5; your projections, force regulation and "5 N normal load" all get skewed by that angle. Their Algorithm 1 removes that bias using only FT data and the commanded velocity. (**Fig. 3–5 and Algorithm 1**, pp. 4–5).

---

## 5) How far is your plan from theirs (mathematically)?

If we rewrite your plan in their notation:

- You're effectively using $\Omega_f = \tilde{\mathbf{n}}\tilde{\mathbf{n}}^T$ and $\Omega_m=I-\Omega_f$ with $\tilde{\mathbf{n}}=\mathbf{f}_s/|\mathbf{f}_s|$. With friction, $\tilde{\mathbf{n}}$ equals $(\mathbf{n}_{\text{true}}-\mu\hat{\mathbf{v}})/\sqrt{1+\mu^2}$, so both $\Omega_f$ and $\Omega_m$ are **rotated by $\delta$**. That mixes normal/tangential axes; your "normal" PI leaks motion into the plane and your tangential command leaks load into the normal. (Your design section on "Force-to-Velocity Law" confirms that the measured force defines $\hat{\mathbf{n}}$.)
    
- You don't inject any null-space bias to keep the probe axis aligned with the true normal, so orientation can lag on curved surfaces; they explicitly add $\rho=\alpha\nabla_\Phi g$ to drive $\gamma\to 0$ (**Eq. (39)–(49)**).
    
- You integrate 5 cm of tangential displacement from a captured start pose (good, simple), but you're not correcting the error model with the updated normal (their $\hat{\mathbf{n}}_{\text{surf}}$ appears inside the error term and the projector, **Eq. (8)**). On varying curvature, that matters.
    

So yes—the hybrid controller is "a solved problem." The **innovation is the estimator glued in front of it** and how they push that estimate into projection and null-space alignment. That's why they can show a small but real accuracy bump on curved/dome paths (**Fig. 7–8, p.6–7**).

---

## 6) If you want your controller to match their contribution

You can keep your architecture and just add three lines of math to your control loop:

1. **Compute** $\Omega_v$, $\mathbf{f}_v$, $\mathbf{f}_\perp$ from your current $\hat{\mathbf{v}}$ and $\mathbf{f}_s$.
    
2. **Update** $\mu_k=|\mathbf{f}_v|/|\mathbf{f}_\perp|$ and its moving average $\bar{\mu}$.
    
3. **Set** $\hat{\mathbf{n}}_{\text{surf}}=\frac{\mathbf{f}_s - \bar{\mu}|\mathbf{f}_\perp|\frac{\hat{\mathbf{v}}}{|\hat{\mathbf{v}}|}}{\left|\mathbf{f}_s - \bar{\mu}|\mathbf{f}_\perp|\frac{\hat{\mathbf{v}}}{|\hat{\mathbf{v}}|}\right|}$, then use:
    
    $$\Omega_f=\hat{\mathbf{n}}_{\text{surf}}\hat{\mathbf{n}}_{\text{surf}}^T, \quad \Omega_m=I-\Omega_f$$
    
    and project your $\mathbf{v}_{\text{adm}}$ and $\mathbf{v}_{\text{des}}$ through those. Optionally add the null-space term $\rho=\alpha\nabla_\Phi g$ with $\nabla_\Phi g = -(J_{ee,\omega})^T[\mathbf{n}_{ee}]_\times \hat{\mathbf{n}}_{\text{surf}}$. (All lifted from **Algorithm 1; Eq. (9)–(10), (35)–(37), (39)–(49)**; **Fig. 2–4**).
    

Everything else in your plan—ROS 2 node, state machine, DLS IK, "move 5 cm while holding 5 N"—can stay intact. (**plan.md**, **README.md**).

---

## 7) One-screen recap (what's new vs. your plan)

- **New idea:** friction-aware, online normal estimation using only FT and robot velocity; use it for projection and for null-space orientation. (**Algorithm 1; Eq. (28)–(38); Eq. (39)–(49)**; figures on pp. 2–5).
    
- **Problem solved:** "force direction ≠ surface normal" during sliding; naive controllers mis-project by $\arctan\mu$. Your plan still makes that assumption.
    
- **Why it counts:** a few tenths of a millimeter on curved industrial paths is a real result; their plots show it (**Fig. 8, p.7**).
    

If you implement the estimator and (optionally) the null-space orienter, you'll have essentially the same functional novelty, with your existing ROS 2 scaffolding.

**References inside the paper:** architecture and hybrid block (**Fig. 2, p.2**), friction-aware normal estimator (**Fig. 3, Algorithm 1, Eq. (28)–(38), pp. 4–5**), null-space optimizer (**Eq. (39)–(49), pp. 5–6; Fig. 4**), and results (**Fig. 7–8, pp. 6–7**).