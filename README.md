# BCS Framework & BCS Simulator Docker Setup

This repository contains the Docker setup for **BCS Framework** and **BCS Simulator**. It includes Dockerfiles and instructions to build, run, and interact with these environments.

## Contents

- **bcs_framework**: 
  - Dockerfile and instructions for setting up and running the BCS Framework.
  - Includes ROS2-based functionality with packages like `bcs_framework` and `active_configuration_files`.

- **bcs_simulator**: 
  - Dockerfile and instructions for setting up and running the BCS Simulator.
  - Integrates Qt-based simulations with ROS2.

---

## Usage

1. **BCS Framework**:
   - Use the `Dockerfile` in the `bcs_framework` folder to build and run the Docker image `docker pull amratianeil/bcs_framework:latest`.
   - Instructions: `docker run -it --rm --network bcs_network amratianeil/bcs_framework:latest`

2. **BCS Simulator**:
   - Use the `Dockerfile` in the `bcs_simulator` folder to build and run the Docker image `docker pull amratianeil/bcs_simulator:latest`.
   - Instructions: `docker run -it --rm amratianeil/bcs_simulator:latest`

---

