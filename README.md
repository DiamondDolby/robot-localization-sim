# 🔍 Particle Filter Robot Localization Simulator

This project implements a **2D robot localization system using a particle filter**, built with **Python** and **OpenCV**. The simulator visualizes a robot navigating a grayscale map using noisy motion and sensor models. It estimates the robot’s position using particles, updating its belief with every movement.

## 📁 Contents

- `robot.py`: Core particle filter simulator with circular robot marker.
- `DispUpdated.py`: Enhanced version with heading arrow and dynamic heading display.
- `map.png`: Grayscale map image used for localization.

> ✅ Use arrow keys or `W/A/D` to control the robot.
> ✅ Press `P` to toggle particle visualization.
> ✅ Press `Q` or `Esc` to quit the simulation.

---

## 🚀 Installation

### 🧱 Requirements

- Python 3.7+
- `numpy`
- `opencv-python`

### 📦 Set up environment

```bash
# 1. Clone the repository
git clone https://github.com/your-username/particle-filter-localization.git
cd particle-filter-localization

# 2. Install dependencies
pip install -r requirements.txt
```

---

## 🕹️Controls
| Key     | Action                  |
| ------- | ----------------------- |
| ↑ / W   | Move Forward            |
| ← / A   | Turn Left               |
| → / D   | Turn Right              |
| P       | Toggle particle display |
| Q / Esc | Quit the simulation     |

---

## 🤯Features
- Nosiy motion and sensor models for realism
- Particle resampling and jitter
- Real-time visualization with:
  - Green robot direction arrow
  - Red dot --> particle guess
  - (Optional) Blue particles [press ```p```]
  - Heading DEG displayed in top-right corner
 
---

##🎥Screenshots
#### Stored under ```assets/``` folder

---

##🏃‍♂️Run the Simulation
```bash
python robot.py          # Basic version
python DispUpdated.py    # Enhanced visualization with heading arrow
```

## 📜 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## 🙋‍♂️ Author
### Sharavanan Mathivanan
#### [GitHub](https://github.com/DiamondDolby)
#### [LinkedIn](https://www.linkedin.com/in/sharkycanada)

---
