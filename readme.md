<div align="center">

# 🤖 WRO 2026 — Autonomous Robotics Platform

**High-performance autonomous navigation, PID line tracking, and closed-loop mechanism control for World Robot Olympiad 2026.**

[![Python](https://img.shields.io/badge/Python-3.x-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![Pybricks](https://img.shields.io/badge/Pybricks-MicroPython-008559?style=for-the-badge&logo=lego&logoColor=white)](https://pybricks.com/)
[![WRO Season](https://img.shields.io/badge/WRO-2026%20Season-E65100?style=for-the-badge)](https://wro-association.org/)
[![Status](https://img.shields.io/badge/Status-Competition%20Ready-00C853?style=for-the-badge)](#)
[![License](https://img.shields.io/badge/License-MIT-7B1FA2?style=for-the-badge)](#)

</div>

---

## ⚡ Key Features

* **🧭 Gyro-Stabilized DriveBase:** Real-time IMU heading compensation, smooth S-curve acceleration/deceleration ramps, and synchronized odometry.
* **🎯 Percentage-Based Actuators:** Zero-drift closed-loop mechanism control (`0.0% – 100.0%`) with automatic endstop stall re-zeroing.
* **⚡ Predictive PID Line Following:** High-frequency derivative filtering, adaptive speed scaling, and multi-line intersection counting.
* **🎨 Statistical Color Classification:** Multi-sample voting pipeline with calibrated HSV thresholding to prevent false positives under dynamic lighting.
* **🧩 Modular OOP Architecture:** Complete decoupling between hardware (`Robot`), kinematics (`Chasis`, `Navegacion`), mechanisms (`Mecanismos`), and strategy (`Misiones`, `ArmadorMosaicos`).

---

## 📂 Architecture Overview

```
├── app.py                  # Entrypoint — Mission runner & integration tests
├── config.py               # Centralized configuration (ports, PID gains, physical constants)
├── robot.py                # Hardware abstraction layer & subsystem dependency wiring
├── Chasis.py               # DriveBase kinematics, odometry & gyro-guided straight tracking
├── Navegacion.py           # Gyro turns (point, short, arc, heading) & PID line follower
├── Mecanismos.py           # Closed-loop actuators (Torque, Elevador, Pinza)
├── Misiones.py             # Official game field routines (Sections 1 through 6)
├── ArmadorMosaicos.py      # Color scanning, matrix parsing & mosaic placement logic
├── RevisadorBateria.py     # Pre-run battery diagnostics & acoustic warnings
├── Utils.py                # Math helpers, angle normalization & audio utilities
└── odd_shit/
    ├── calibrador_mecanismos.py  # Interactive stall-torque mechanism calibrator
    └── medir_limites.py          # Traction, speed saturation & slip diagnostic suite
```

---

## 🔌 Hardware Specifications

| Component | Hardware Port | Configuration |
|---|---|---|
| **Left Drive Motor** | `Port.B` | Counter-Clockwise (`56 mm` wheel) |
| **Right Drive Motor** | `Port.E` | Clockwise (`56 mm` wheel) |
| **Torque Mechanism / Rear Cage** | `Port.F` | Range: `0%` (home) to `100%` (`-179°`) |
| **Front Lift Mechanism** | `Port.C` | Range: `0%` (up) to `100%` (`672°`) |
| **Primary Gripper / Pinza** | `Port.A` | Range: `0%` (closed) to `100%` (`798°`) |
| **Color / Line Sensor** | `Port.D` | High-frequency reflective & HSV modes |
| **Wheelbase Track** | — | `160 mm` |

---

## 🚀 Quick Start

1. **Prerequisites:** Connect your LEGO SPIKE Prime / MINDSTORMS Robot Inventor hub flashed with [Pybricks](https://pybricks.com/).
2. **Open in VS Code:** Use the [Pybricks extension](https://marketplace.visualstudio.com/items?itemName=pybricks.pybricks-code) or `pybricksdev`.
3. **Run Missions:** Open [`app.py`](app.py), uncomment the desired mission section or mosaic challenge, and press **F5**:

```python
# Execute complete tournament run
misiones.seccion_1_salida_y_cemento()
misiones.seccion_2_dejar_cemento_y_tomar_verdes()
matriz_detectada = misiones.seccion_3_escanear_matriz_y_dejar_verdes(armador)
misiones.seccion_4_amarillos_y_azules()
misiones.seccion_5_tomar_pala_y_dejar_amarillos()
misiones.seccion_6_retorno_pala_y_fin(armador, matriz_detectada)
```

---

## 👥 Collaborators

<div align="center">
<table>
  <tr>
    <td align="center">
      <a href="https://github.com/robertofabiot">
        <img src="https://avatars.githubusercontent.com/u/203884931?v=4" width="100px;" alt="Roberto F. Tercero"/><br />
        <sub><b>Roberto F. Tercero</b></sub>
      </a><br />
      <sub>Lead Developer & Robotics Architecture</sub>
    </td>
    <td align="center">
      <a href="https://github.com/uxvcharlie">
        <img src="https://avatars.githubusercontent.com/u/211022677?v=4" width="100px;" alt="Carlos Rafael Umaña Vásquez"/><br />
        <sub><b>Carlos Rafael Umaña Vásquez</b></sub>
      </a><br />
      <sub>Developer & Strategy</sub>
    </td>
  </tr>
</table>
</div>

---

<div align="center">
<sub>Built with ❤️ for WRO 2026. Powered by <a href="https://pybricks.com/">Pybricks</a>.</sub>
</div>
