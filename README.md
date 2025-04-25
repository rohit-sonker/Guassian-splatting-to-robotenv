
# Compositional Gaussian Splatting for Simulated Mobile Robot Environments  
**16761 Course Project**

This project involves two primary components for reproducing our results: scene manipulation using Gaussian Splatting and mesh-based simulation via SuGaR.

---

## 🔧 Setup & Execution Guide

### 1. Manipulating Captured 3DGS Scene

#### a. Train and Manipulate 3DGS with Segment Anything Gaussians  
Follow the [official SegAnyGS instructions](https://github.com/yzslab/gaussian-splatting-lightning/tree/main?tab=readme-ov-file#210-segment-any-3d-gaussians), but **use our modified version** found in:

`./gaussian-splatting-lightning`

We modified the viewer to allow object manipulation within the 3DGS scene. The key changes are in:

`./gaussian-splatting-lightning/internal/renderers/seganygs_renderer.py`


This will allow you to train a SAGA Gaussian Splatting scene, and then you can load the scene using the interactively manipulate objects with 

`python viewer.py <path-to-seganygs-output>`

---

#### b. Extract Mesh with SuGaR  
Once training is complete, you’ll obtain a `.ply` file.  
Next, use the SuGaR method (unmodified) found in:

`./SuGaR`


Please follow the installation and usage instructions provided in the `SuGaR/README.md`.

---

#### c. Load Mesh in PyBullet  
SuGaR will output a reconstructed .obj mesh. This mesh can then be imported into PyBullet for simulation-based tasks (e.g., obstacle avoidance, path planning). The code to run this can be found under ...

---
#### Loading mesh env in Pybullet for Robot testing

Main file - load_mesh_test2.py

This loads up a given mesh environment and adds robot which plans its path from starting point to goal point. Both locations can be varied.
Robot functionalities are defined within utils.py

