
# Two-Link Robotic Manipulator Control using PI, PD, and PID

## 🧠 Overview

This project models and simulates the control of a **2-link robotic manipulator** using classical control strategies — **PI**, **PD**, and **PID controllers**. The main objective is to achieve precise position control by modulating torque based on system error.

The manipulator's dynamics are modeled using **Lagrangian mechanics**, and simulations are executed using **MATLAB** and **Simulink** to evaluate controller performance.

---

## 🎯 Objectives

- Model a two-link robotic manipulator using state functions and differential equations.
- Design and compare PI, PD, and PID control strategies for position control.
- Simulate dynamic responses using MATLAB and Simulink.
- Analyze joint trajectories and error behavior under different controller gains.

---

## 🧩 System Description

- A planar two-link robotic arm with:
  - Link lengths: `l1`, `l2`
  - Masses: `m1`, `m2` at joints and ends
  - Joint angles: `q1`, `q2`

- The system dynamics follow:

```
M(q) * q̈ + C(q, q̇) * q̇ + G(q) = τ
```

where:
- `M(q)` = inertia matrix
- `C(q, q̇)` = Coriolis and centrifugal matrix
- `G(q)` = gravity torque vector
- `τ` = applied torque

After substitution:
```
M = [[M11, M12],
     [M12, M22]]

C = [[-m2*l1*l2*sin(q2)*q̇2, -m2*l1*l2*sin(q2)*(q̇1 + q̇2)],
     [0, m2*l1*l2*sin(q2)*q̇2]]

G = [m1*l1*g*cos(q1) + m2*g*(l2*cos(q1+q2) + l1*cos(q1)),
     m2*l2*g*cos(q1+q2)]
```

---

## 🛠️ Controllers Implemented

### PI Controller

Reduces **steady-state error** but may cause **oscillations** due to lack of damping.

```
f(t) = Kp * e(t) + Ki * ∫ e(t) dt
```

Discretized control inputs:
```
f1 = Kp1 * (q1f - q1) + Ki1 * ∫ (q1f - q1) dt
f2 = Kp2 * (q2f - q2) + Ki2 * ∫ (q2f - q2) dt
```

### PD Controller

Improves **response time** and reduces **overshoot**, but can leave **steady-state error**.

```
f(t) = Kp * e(t) + Kd * de(t)/dt
```

Discretized control inputs:
```
f1 = Kp1 * (q1f - q1) - Kd1 * q̇1
f2 = Kp2 * (q2f - q2) - Kd2 * q̇2
```

### PID Controller

Combines the advantages of both PI and PD controllers.

```
f(t) = Kp * e(t) + Ki * ∫ e(t) dt + Kd * de(t)/dt
```

Discretized control inputs:
```
f1 = Kp1 * (q1f - q1) - Kd1 * q̇1 + Ki1 * ∫ (q1f - q1) dt
f2 = Kp2 * (q2f - q2) - Kd2 * q̇2 + Ki2 * ∫ (q2f - q2) dt
```

---

## 🧪 Simulation

- MATLAB’s `ode45` used for numerical integration
- Control logic implemented in `.m` files for PI, PD, and PID
- Simulink diagrams used for visual simulation
- Plots generated for:
  - Joint angles over time
  - Comparative performance of controllers

---

## 📈 Observations

| Controller | Response Characteristics |
|------------|---------------------------|
| **PI**     | Low steady-state error, but high oscillation |
| **PD**     | Fast response, small overshoot, but steady-state error remains |
| **PID**    | Best overall: fast, accurate, stable |

Tuning Notes:
- Higher `Kp`: Faster response, more overshoot
- Higher `Kd`: Dampens overshoot, reduces oscillations
- Higher `Ki`: Reduces steady-state error, may cause instability

---

## 📂 Files Included

- `pi.m`, `pd.m`, `pid.m` – Controller implementations in MATLAB
- Simulink diagrams (if applicable)
- Plots and results from simulations
- `st_proj_final.pdf` – Full report with derivations and analysis

---

## ✅ Conclusion

The PID controller achieved the most reliable and accurate control among the three, with minimal overshoot and no steady-state error. These insights are foundational for extending into real-world robotic systems.

---

## 📚 References

- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*
- MATLAB & Simulink Documentation

---

## 👨‍💻 Authors

- Team: The Better Half  
- Date: October 7, 2023  
- Institution: IIIT Hyderabad
