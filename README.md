# 🤖 Self-Balancing Bot
*Defying gravity, one PID loop at a time*

<div align="center">
  
  [![Arduino](https://img.shields.io/badge/Arduino-Nano-00979D?style=for-the-badge&logo=arduino&logoColor=white)](https://arduino.cc)
  [![IMU](https://img.shields.io/badge/IMU-BNO055-FF6B35?style=for-the-badge)](https://www.bosch-sensortec.com/products/smart-sensors/bno055/)
  [![Control](https://img.shields.io/badge/Control-PID-4CAF50?style=for-the-badge)](https://en.wikipedia.org/wiki/PID_controller)
  [![Status](https://img.shields.io/badge/Status-Balancing-success?style=for-the-badge)](/)

</div>

---

## 🎯 What's This All About?

<img width="430" height="397" alt="image" src="https://github.com/user-attachments/assets/fbff57b9-a3b9-46a8-92e8-3fc62a8eef05" />







Meet the **Self-Balancing Bot** - a two-wheeled marvel that refuses to fall over! This isn't just another robot; it's a testament to the beautiful dance between physics, mathematics, and engineering. Using the magic of PID control and the precision of a BNO055 IMU, this little warrior constantly fights gravity and wins.

## 🧠 The Brain Behind the Balance

Our bot thinks fast and acts faster:

- **🎛️ PID Controller**: The secret sauce that keeps things upright
- **📡 BNO055 IMU**: 9-axis absolute orientation sensor providing real-time feedback
- **🔧 Arduino Nano**: The compact powerhouse processing it all
- **🎯 Closed-Loop System**: Continuous feedback for perfect balance

## ⚙️ Technical Specifications

| Component | Specification |
|-----------|--------------|
| **Microcontroller** | Arduino Nano |
| **IMU Sensor** | BNO055 9-DOF Absolute Orientation |
| **Control Algorithm** | PID (Proportional-Integral-Derivative) |
| **Communication** | I2C (IMU ↔ Arduino) |
| **Balance Range** | adjust what your bot physics sypports ! |

## 🔧 How It Works

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐
│   BNO055    │───▶│ Arduino Nano │───▶│   Motors    │
│ IMU Sensor  │    │ PID Control  │    │  & Wheels   │
└─────────────┘    └──────────────┘    └─────────────┘
       ▲                   │                    │
       │                   ▼                    │
       │            ┌──────────────┐            │
       └────────────│ Tilt Angle   │◀───────────┘
                    │  Feedback    │
                    └──────────────┘
```

### The Control Loop Dance 💃

1. **Sense**: BNO055 measures the bot's tilt angle and angular velocity
2. **Think**: Arduino calculates the error using PID algorithm
3. **Act**: Motors adjust wheel speed to counteract the tilt
4. **Repeat**: Loop runs continuously at high frequency
5. **Balance**: Bot maintains upright position like a digital tightrope walker

## 🚀 Features

- **🎪 Dynamic Balance**: Recovers from disturbances automatically
- **🔄 Real-time Processing**: Sub-10ms response time
- **📊 Sensor Fusion**: BNO055's built-in Kalman filter for smooth data
- **🎛️ Tunable Parameters**: Adjustable PID gains for optimal performance on your own bot
- **🔧 Compact Design**: Nano form factor for minimal footprint

## 📋 Bill of Materials (BOM)

| Qty | Component | Purpose |
|-----|-----------|---------|
| 1 | Arduino Nano | Main controller |
| 1 | BNO055 IMU Module | Orientation sensing |
| 2 | n20 Geared Motors | Propulsion |
| 2 | n20 Wheels | Ground contact |
| 1 | Motor Driver (TB6612FNG) | Motor control |
| 1 | liPo | Power source |
| 1 | Chassis/Frame | Mechanical structure |
| - | Jumper Wires & Headers | Connections |

## 🔌 Wiring Diagram

```
Arduino Nano          BNO055 IMU
    VIN    ────────────  VCC
    GND    ────────────  GND
     A4    ────────────  SDA
     A5    ────────────  SCL

Arduino Nano        Motor Driver
     D3     ────────────  ENA
     D4     ────────────  IN1
     D5     ────────────  IN2
     D6     ────────────  ENB
     D7     ────────────  IN3
     D8     ────────────  IN4
```

## 🎛️ PID Tuning Guide

The art of balance lies in the PID parameters:

- **Kp (Proportional)**: Start with 20-50, higher = more aggressive
- **Ki (Integral)**: Usually small, 0.1-1.0, eliminates steady-state error
- **Kd (Derivative)**: Start with 0.5-2.0, adds damping

### Tuning Process:
1. Set Ki and Kd to 0
2. Increase Kp until oscillation begins
3. Add Kd to reduce oscillation
4. Add small Ki to eliminate offset

## 🔮 Future Enhancements

- [ ] **AI Powered**: Reinforcement Learning (NO magical PID!)
- [ ] **Advanced Sensors**: Camera for line following
- [ ] **Machine Learning**: Adaptive PID parameters
- [ ] **LED Indicators**: Visual status feedback

## 🛠️ Getting Started

### 1. **Hardware Assembly**
   - Mount Arduino Nano and BNO055 on chassis
   - Connect motors to H-bridge driver
   - Wire according to pinout diagram above
   - Install push button on pin 11 for initial calibration

### 2. **Software Setup**
   - Install Adafruit BNO055 library in Arduino IDE
   - Upload the provided code to Arduino Nano
   - Open Serial Monitor (115200 baud) for debugging

### 3. **Initial Calibration**
   - Power on the bot in upright position
   - Wait for BNO055 auto-calibration (few seconds)
   - Press calibration button to set balance point
   - Observe angle readouts in Serial Monitor

### 4. **Fine Tuning** (if needed)
   - Adjust the three PID parameter sets in code
   - Modify `motorMax`, `motorMin` values for your motors
   - Change `startAngle` for different safety thresholds
   - Test in safe environment with crash protection!

### 5. **Let It Balance!** 🎉
   - Place bot upright and watch the magic
   - Gently push to test recovery response
   - Monitor serial output for performance analysis

## 🤝 Contributing

Found a bug? Have an improvement? Contributions are welcome! Whether it's:
- Code optimizations
- Documentation improvements
- Hardware modifications
- PID tuning suggestions

## 📜 License

This project is open source - feel free to learn, modify, and share!

---

<div align="center">

**"In a world full of things that fall down, be the bot that stands up!"** 🤖⚖️

*Made with ⚡ Arduino, 🧮 Math, and ❤️ Engineering*

</div>
