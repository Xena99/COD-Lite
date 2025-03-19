# COD-Lite

**COD-Lite** is a lightweight C++ project utilizing the [raylib](https://www.raylib.com/) library, designed to simulate fundamental aspects of game AI, including pathfinding, navigation meshes, and spatial partitioning.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)

## Features

- **AI Engine**: Manages AI behavior and decision-making processes.
- **Pathfinding**: Implements algorithms to determine optimal paths for AI agents.
- **Navigation Mesh Utilities**: Handles the creation and management of navigation meshes for efficient movement.
- **QuadTree Implementation**: Provides spatial partitioning to optimize collision detection and environment queries.

## Installation

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/Xena99/COD-Lite.git
   cd COD-Lite
   ```

2. **Install Dependencies**:
   - Ensure [raylib](https://www.raylib.com/) is installed on your system.
   - If not, follow the [raylib installation guide](https://github.com/raysan5/raylib#installation) suitable for your platform.

3. **Build the Project**:
   - Use your preferred C++ build system (e.g., `make`, `CMake`) to compile the project.
   - Ensure that the compiler can locate the raylib headers and libraries.

## Usage

1. **Run the Executable**:
   - After building, execute the generated binary.

2. **Controls**:
   - The AI agent will autonomously navigate the environment using the implemented pathfinding algorithms.
   - Observe the console for debug logs detailing AI decisions and pathfinding steps.

## Project Structure

The project comprises several key components:

- **AIEngine.cpp**: Manages the AI agent's behavior, including threading for asynchronous decision-making.
  - Initializes the AI engine with a reference to the `QuadTree` and the agent's starting position.
  - Handles the AI thread loop, processing movement towards targets and interacting with the environment.

- **PathFinding.cpp**: Contains the pathfinding logic for the AI agent.
  - Implements the A* algorithm to compute optimal paths.
  - Utilizes a priority queue to explore nodes based on cost, ensuring efficient path computation.

- **NavMeshUtils.cpp**: Provides utilities for navigation mesh management.
  - Detects walkable triangles within a specified radius around the AI agent.
  - Integrates with game models to assess ground and obstacle data for navigation purposes.

- **QuadTree.cpp**: Implements a quadtree for spatial partitioning.
  - Divides the game world into manageable sections to optimize collision detection and environment queries.
  - Supports insertion, deletion, and querying of objects within the spatial hierarchy.
