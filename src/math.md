Here's the math version of **"what happens after first contact"** in your controller: from wrench_node → estimated surface normal on a curved surface → probe orientation → 5 cm tangential motion.

I'll ignore IK/joint space like you asked.

---

## 1. Wrench pipeline (what `wrench_node` gives you)

Let

* $\mathcal{F}_S$: F/T sensor frame
* $\mathcal{F}_B$: robot base (`base_link`) frame

Raw sensor wrench in sensor frame:
$$
w^{S}_{\text{raw}} =
\begin{bmatrix}
F^{S}_{\text{raw}} \\
\tau^{S}_{\text{raw}}
\end{bmatrix}
\in \mathbb{R}^6
$$

### 1.1 Bias and gravity compensation (in wrench_node)

`wrench_node` estimates a constant bias $w^S_{\text{bias}}$ and the wrench due to gravity on the tool $w^S_g$ using the tool mass $m$, CoM position, and gravity vector $g=[0,0,-g]^\top$ in world/base. Conceptually:

$$
w^S_{\text{proc}} = w^S_{\text{raw}} - w^S_{\text{bias}} - w^S_{g}
$$

Then it transforms to base:

$$
w^B_{\text{proc}} = 
\begin{bmatrix}
F^B_s \\
\tau^B_s
\end{bmatrix}
= \operatorname{Ad}_{T_{BS}} \cdot w^S_{\text{proc}}
$$

where $\operatorname{Ad}_{T_{BS}}$ is the 6×6 adjoint of the transform from sensor to base.

Your hybrid controller then just **takes the force part in base**:
$$
F_s(t) := F^B_s(t) \in \mathbb{R}^3
$$
(from `/netft/proc_base`).

From here on, everything is $\mathcal{F}_B$.

---

## 2. Problem statement (after contact)

Once you press into the workpiece and reach a **steady contact load** (≈ 5 N), you want to:

1. Track a **surface‑normal force**
   $$F_d = 5\ \text{N}$$
   on an **unknown curved surface**.
2. Orient the probe so its axis follows the **local surface normal**.
3. Move the end‑effector **5 cm** along a tangential direction.

The surface itself could be anything (dome, sinusoid, arbitrary height field). You do **not** know its geometry a priori.

---

## 3. Frames and basic variables

At time $t$ (or discrete step $k$):

* End‑effector pose in base:
  $$R(t)\in SO(3),\quad p(t)\in\mathbb{R}^3$$
* Probe axis in base (assume probe points along tool $-Z$):
  $$n_{\text{ee}}(t) = -R(t) \cdot e_z,\qquad e_z = [0,0,1]^\top$$
* Processed contact force from `wrench_node`:
  $$F_s(t) \in \mathbb{R}^3$$
* Commanded Cartesian twist (what your hybrid node publishes):
  $$\dot x(t) =
  \begin{bmatrix}
  v(t)\\
  \omega(t)
  \end{bmatrix}
  \in \mathbb{R}^6,\quad v,\omega \text{ in base}$$

You maintain a **search direction** (pressing direction) in base, e.g.
$$\hat s \in \mathbb{R}^3,\quad |\hat s|=1$$
(default $\hat s = [0,0,-1]^\top$).

---

## 4. Scalar normal force (consistent sign)

You define a **current estimate of the surface normal** in base, $\hat n(t)$ (unit vector). During SEEK/DWELL you **lock it** to the search direction:

$$\hat n(t) = \hat s$$

You define scalar compressive normal force as
$$F_n(t) = - F_s(t)^\top \hat n(t) \tag{1}$$

So if the environment pushes back **against** $\hat n$, then $F_n>0$.

You run the seek phase until

$$|F_n(t) - F_d| \le \Delta F$$

for some band (e.g. $\Delta F = 0.2\ \text{N}$) and dwell-time is satisfied. 

After that, you treat the system as "in stable contact" and start estimating the **true surface normal** (which will vary on a curved surface).

---

## 5. Friction‑aware surface normal estimation (curved surface)

Now we work in **discrete time** with sampling period $\Delta t$ and step index $k$.

At step $k$ you know:

* Latest processed contact force: $F_{s,k} \in \mathbb{R}^3$,
* Last commanded linear velocity: $v_{k-1} \in \mathbb{R}^3$.

### 5.1 Velocity direction

If the tool is actually sliding ($\|v_{k-1}\|$ above a threshold $v_\varepsilon$) define the unit velocity:

$$\hat v_k = \frac{v_{k-1}}{|v_{k-1}|} \tag{2}$$

If the speed is tiny you fall back to a sign‑refined tangential direction, but conceptually (2) is what you want. 

### 5.2 Force decomposition

Decompose measured force into components **along** and **orthogonal** to velocity:

$$F_{v,k} = (\hat v_k^\top F_{s,k}) \cdot \hat v_k \tag{3}$$

$$F_{\perp,k} = F_{s,k} - F_{v,k} \tag{4}$$

This is exactly $F_{v} = \Omega_v F_s$, $F_\perp = (I-\Omega_v)F_s$ with $\Omega_v = \hat v \hat v^\top$ in the paper. 

### 5.3 Coulomb friction estimate

If $|F_{\perp,k}|$ isn't degenerate, define the instantaneous Coulomb coefficient:

$$\mu_k = \frac{|F_{v,k}|}{|F_{\perp,k}|} \tag{5}$$

Filter it with an exponential moving average:

$$\bar\mu_k = \alpha \cdot \mu_k + (1-\alpha) \cdot \bar\mu_{k-1},\quad 0<\alpha\le1 \tag{6}$$

### 5.4 Friction force and corrected normal force vector

Model friction as Coulomb force opposing motion:

$$F_{\tau,k} = -\bar\mu_k \cdot |F_{\perp,k}| \cdot \hat v_k \tag{7}$$

Then define a **corrected normal force vector**:

$$F_{n,k} = F_{s,k} - F_{\tau,k} \tag{8}$$

This is (essentially) Algorithm 1 and Fig. 3 in Nasiri & Wang: remove an estimate of friction from the measured force to get the true normal reaction. 

### 5.5 Surface normal direction

Normalize:

$$\tilde n_k = \frac{F_{n,k}}{|F_{n,k}|} \tag{9}$$

To avoid 180° flips you enforce continuity relative to previous estimate $\hat n_{k-1}$:

$$\hat n_k =
\begin{cases}
\tilde n_k, & \tilde n_k^\top \hat n_{k-1} \ge 0\\[3pt]
-\tilde n_k, & \text{otherwise}
\end{cases} \tag{10}$$

This $\hat n_k$ is your **estimated surface normal** at the current contact point on the curved surface.

You also update scalar normal force with the same convention as (1):

$$F_n(k) = -F_{s,k}^\top \hat n_k \tag{11}$$

So as you slide over a dome or sinusoidal surface, $\hat n_k$ rotates gradually, tracking the local surface normal even though you never explicitly know the surface geometry. 

---

## 6. Tangential direction, contact frame and curvature

You maintain a global "hint" direction $\tilde t \in \mathbb{R}^3$ (e.g. "go roughly along base X"). At each step, project it into the **tangent plane** perpendicular to $\hat n_k$:

$$\tilde t_k = (I - \hat n_k \hat n_k^\top) \cdot \tilde t$$

$$\hat t_k = \frac{\tilde t_k}{|\tilde t_k|} \tag{12}$$

Then $\hat t_k$ is:

* Tangential to the unknown surface at the contact point,
* As close as possible to the desired direction $\tilde t$.

Construct a full **contact frame**:

$$\hat y_k = \frac{\hat n_k \times \hat t_k}{|\hat n_k \times \hat t_k|}$$

$$R_c(k) =
\begin{bmatrix}
\hat t_k & \hat y_k & \hat n_k
\end{bmatrix} \in SO(3) \tag{13}$$

This is exactly the `contact_frame` TF you publish (X = tangential, Z = normal). As curvature changes, $\hat n_k$ and thus $\hat t_k$ and $R_c$ all change smoothly along the trajectory.

---

## 7. Orientation control: probe axis follows the normal

You want the probe axis $n_{\text{ee}}(t)$ to align with $\hat n_k$. Ideally:

$$n_{\text{ee}}(t) \approx \hat n(t)$$

That's exactly the objective $g(\Phi) = \hat n_{\text{surf}}^\top (-n_{\text{ee}})$ in Eq. (40) of the paper. 

Rather than a null‑space gradient (which needs 7‑DoF), you use a resolved‑rate orientation servo:

1. **Desired orientation**:
   $$R_{\text{des}}(k) = R_c(k)$$
2. **Current orientation**: $R(k)$ from FK.
3. Orientation error:
   $$R_{\text{err}}(k) = R_{\text{des}}(k) \cdot R(k)^\top \tag{14}$$
4. Convert to axis–angle:
   $$R_{\text{err}} = \exp(\theta_k[\hat u_k]_\times),
   \quad \theta_k\in(-\pi,\pi],\ \hat u_k\in\mathbb{S}^2$$
5. Orientation control law (P in $\mathfrak{so}(3)$):
   $$\omega_k = K_R \cdot \theta_k \cdot \hat u_k,\quad
   |\omega_k| \le \omega_{\max} \tag{15}$$

This $\omega_k$ is the **angular part** of your twist, in base. The rotation dynamics

$$\dot R = R[\omega]_\times$$

make $R(t)$ converge toward $R_{\text{des}}(t)$, so the probe axis follows the estimated surface normal along the curved surface.

---

## 8. Normal force control along $\hat n_k$

Desired normal force: $F_d = 5\ \text{N}$.

Define error:
$$e_F(k) = F_d - F_n(k) \tag{16}$$

Discrete PI:

$$I_F(k) = I_F(k-1) + e_F(k) \cdot \Delta t$$

$$v_n(k) = K_P \cdot e_F(k) + K_I \cdot I_F(k),
\quad
|v_n(k)| \le v_{n,\max} \tag{17}$$

Normal linear velocity:

$$v_{\text{normal}}(k) = v_n(k) \cdot \hat n_k \tag{18}$$

So if the measured force is too low, $e_F>0 \Rightarrow v_n>0$, and you move **into** the surface along $\hat n_k$; if too high, you pull away. That's the admittance force loop described in your plan and in the paper.

---

## 9. Tangential distance and 5 cm motion

Let $p_k$ be EE position in base at step $k$.

When TANGENTIAL phase begins:

$$s_0 = 0,\quad p_0 = p(t_0)$$

Each step:

$$\Delta p_k = p_k - p_{k-1}$$

$$s_k = s_{k-1} + \max\big(0, \Delta p_k^\top \hat t_k\big) \tag{19}$$

So $s_k$ is **distance traveled from the start along the (moving) tangential direction**, not true arclength on the surface. That matches your design: 5 cm of EE motion along the tangent. 

Remaining distance:

$$s_{\text{rem},k} = s_{\text{target}} - s_k,\qquad
s_{\text{target}} = 0.05\ \text{m} \tag{20}$$

Tangential scalar speed:

$$v_t(k) =
\begin{cases}
0, & s_{\text{rem},k} \le 0\\[3pt]
\min\big(v_{t,\max}, K_t \cdot s_{\text{rem},k}\big), & s_{\text{rem},k} > 0
\end{cases} \tag{21}$$

Tangential velocity:

$$v_{\text{tan}}(k) = v_t(k) \cdot \hat t_k \tag{22}$$

As $s_k \to s_{\text{target}}$, $s_{\text{rem},k}\to 0$, so $v_t(k)\to 0$; you glide to a stop at exactly 5 cm.

---

## 10. Final twist (what you actually send each control step)

In the **sliding phase** (after dwell):

1. Estimate $\hat n_k$ from $(F_{s,k}, v_{k-1})$ using the friction‑aware method (Sec. 5). 
2. Compute $\hat t_k$ and contact frame $R_c(k)$ (Sec. 6).
3. Compute normal force $F_n(k)$ and PI normal velocity $v_n(k)$ (Sec. 8).
4. Update tangential distance $s_k$ and tangential velocity $v_t(k)$ (Sec. 9).
5. Compute orientation angular velocity $\omega_k$ using the orientation error (Sec. 7).

Then compose the twist:

$$v_k = v_{\text{normal}}(k) + v_{\text{tan}}(k)
= v_n(k) \cdot \hat n_k + v_t(k) \cdot \hat t_k \tag{23}$$

$$\omega_k = K_R \cdot \theta_k \cdot \hat u_k,\quad |\omega_k|\le\omega_{\max} \tag{24}$$

So the full Cartesian twist command is

$$\dot x_k =
\begin{bmatrix}
v_k\\[2pt]
\omega_k
\end{bmatrix} \tag{25}$$

You feed $\dot x_k$ to the Cartesian velocity controller, which does IK and joint velocity; but **all the contact geometry, curvature handling, orientation behaviour, and 5 cm logic are entirely in (2)–(25)**.

---

If you want, I can next write this as a compact pseudo‑code *Algorithm 1* in LaTeX (with "Input: F_s, v_prev, …; Output: twist") that matches exactly what you're doing now.