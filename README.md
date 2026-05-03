# SCARA-Robot-simulator-vibecoding

# 🤖 Gemini x Python: 3-Axis SCARA Robot Simulator

This project is an advanced 3-axis SCARA robot kinematics simulator developed through a collaboration between a mechanical engineer and Google Gemini. It expands upon previous models to demonstrate complex multi-axis movements, inverse kinematics, trajectory planning, and obstacle avoidance.

## 📺 Project Walkthrough

[![Robot Simulator](https://img.youtube.com/vi/ZpYVr37zNo8/0.jpg)](https://www.youtube.com/watch?v=ZpYVr37zNo8)
> **Click the image above to watch the "Vibe Coding" process for this 3-axis system.**

## 🛠 Features

*   **AI-Assisted UI & Logic**: GUI scaffolding and inverse kinematics solver drafted via Gemini.
*   **Three Operation Modes**:
    *   **Manual Mode**: Direct, real-time control of the robot's individual axes.
    *   **Mission Mode**: 
        *   Precise Point-to-Point (PTP) control.
        *   Selectable trajectory profiles: **Step Control** or **S-Curve Control**.
        *   Configurable repetition counts for cyclic tasks.
        *   **Obstacle Avoidance**: Define obstacle coordinates in the workspace to automatically calculate safe maneuver paths.
    *   **Hardware Setting Mode**: Calibration and configuration environment for physical hardware integration.
*   **Advanced Data Visualization**: Real-time graph plotting for rigorous kinematic analysis, displaying:
    *   Joint Angles
    *   Angular Velocity
    *   Singularity monitoring
*   **Expert Refinement**: 20 years of mechanical design expertise applied to ensure strict mathematical accuracy and realistic kinematic behaviors.

## 🧠 Engineering Note: Vibe Coding

This repository demonstrates the advanced application of "Vibe Coding." Handling a 3-axis SCARA system introduces significant complexity regarding trajectory logic and singularity avoidance. The engineer focuses on defining the mathematical principles, workspace constraints, and control algorithms (like S-Curve and obstacle avoidance matrices), while Gemini rapidly iterates the GUI event loops, state management, and `matplotlib` data visualizations.

## 🚀 How to Run

1. Ensure you have Python installed.
2. Install the required external library for the graphing features:
   ```bash
   pip install matplotlib
   Run the script:
3. python robot_simulator_2axis.py
