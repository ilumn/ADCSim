# ADCSim
A simulation tool for CubeSat Attitude Determination and Control Systems using C++ extensions (physics/graphics) with a Python controller.

---

## Setup & Run

1. **Clone Repo**  
   `git clone https://github.com/ilumn/ADCSim.git`

2. **Install Build Requirements**  
   In the root directory, run:  
   ```bash
   pip install -e .
   ```

3. **Run Simulation**  
   ```bash
   python ADCSim
   ```

---

## Math

### Quaternion Operations
```
q = [w, x, y, z]

q₁ ⊗ q₂ = [ 
  w₁w₂ − x₁x₂ − y₁y₂ − z₁z₂,
  w₁x₂ + x₁w₂ + y₁z₂ − z₁y₂,
  w₁y₂ − x₁z₂ + y₁w₂ + z₁x₂,
  w₁z₂ + x₁y₂ − y₁x₂ + z₁w₂ ]
```

### Orientation Update (Exponential Map)
```
q(t+dt) = exp((dt/2) · ω̂) ⊗ q(t)
        = [ cos(|ω|·dt/2), sin(|ω|·dt/2)·(ω/|ω|) ]
```

### Reaction Wheel Dynamics
```
Torque:   T = k_motor · cmd − k_damp · ω

ω̇ (wheel):   ω̇ = T / I_wheel
```

### CubeSat Dynamics
```
ω̇ (sat):   ω̇ = -T / I_sat
Orientation update via exponential map
```

### Sensor Simulation
```
Measurement = True Value + Gaussian Noise
```

### PID Control
```
Error = (target − measured) wrapped to [−π, π] (inverted)
cmd = kp · error + ki · ∫error dt + kd · (d(error)/dt)
```

