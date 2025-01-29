# Autonomous Drone Training using NEAT and PPO

## Requirements

To run this project, you will need the following software:

- **Unity** (version 2022.3.10f1)
- **Python** (version 3.10.12)
- **ML-Agents Python** (version 1.0.0)
- **NEAT-Python** (custom version available at: [GitHub](https://github.com/xHalaso/neat-python-plus/tree/develop))
- **Jupyter Notebook**
- **Visual Studio 2022**

## Repository Structure

The repository contains the following directories and files:

- **NEAT/**
  - **configs/** - Configuration files for NEAT-Python
  - **results/** - Results of PPO and NEAT algorithms
  - **testers/** - Scripts for testing trained models
  - **trainers/** - Scripts for training models
  - **utilities/** - Utility scripts
- **trainer\_config.yaml** - Configuration file for the PPO algorithm
- Other files contain the Unity simulation environment

## Installation

Follow these steps to install the necessary dependencies:

1. Download **Unity**: [Unity Download](https://unity.com/releases/editor/whats-new/2022.3.10) *(valid as of 10.05.2024)*
2. Download **Python**: [Python Download](https://www.python.org/downloads/release/python-31012/) *(valid as of 10.05.2024)*
3. Install **ML-Agents Python**:
   ```bash
   pip install mlagents
   ```
4. Install **Jupyter Notebook**:
   ```bash
   pip install notebook
   ```
5. Download **Visual Studio 2022**: [Visual Studio Download](https://visualstudio.microsoft.com/vs/) *(valid as of 10.05.2024)*

## Running the Project

### PPO Algorithm Training

1. Open the entire project folder in Unity.
2. Navigate to the `Scenes` folder and select the desired experiment scene.
3. Open a terminal and run the following command to start training:
   ```bash
   mlagents-learn <your_flags_here>
   ```
4. Start the Unity simulation.

### PPO Algorithm Testing

1. Open the `Scenes` folder in Unity and select the desired experiment scene.
2. Assign the `.onnx` model to one of the drones in the hierarchy by dragging the model into the `BehaviorParameters` component.
3. Start the simulation.

### NEAT Algorithm Training

1. Open the `Scenes` folder in Unity and select the desired experiment scene.
2. Open the required training script in **Jupyter Notebook**.
3. If necessary, modify the corresponding configuration file.
4. Ensure that the only active drone in Unity is named **DummyDrone**.
5. Run the Python script and start the Unity simulation.

### NEAT Algorithm Testing

1. Open the `Scenes` folder in Unity and select the desired experiment scene.
2. Open the required testing script in **Jupyter Notebook**.
3. Run the Python script and start the Unity simulation.

## Common Issues

- Make sure that all Python scripts have the correct paths set for configuration files.

---

For more details, please refer to the technical documentation included in this repository.

---

Author: **Ing. Aleš Melichar**

