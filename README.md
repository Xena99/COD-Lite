# COD-Lite

COD-Lite is a lightweight C++ project built using the **raylib** library, designed to implement advanced **AI pathfinding, navigation meshes, and spatial partitioning** techniques. This project focuses on **optimized AI movement and decision-making** using **QuadTrees for spatial partitioning** and **A* pathfinding with funnel algorithms** for smooth navigation. 

## Table of Contents
- [Features](#features)
- [Output](#output)
- [Project Structure](#project-structure)

## Features

### AI Engine
- Manages **AI behavior, decision-making, and movement** through an **asynchronous processing system**.
- Uses a **threaded AI loop** to handle **real-time path recalculations** based on dynamic targets.

### Pathfinding
- Implements **A* search** for **optimal path computation** across a **NavMesh-based environment**.
- Utilizes **priority queues and heuristics** for efficient pathfinding.
- Incorporates the **Funnel Algorithm** for **smooth movement** along paths.

### Navigation Mesh (NavMesh)
- Detects **walkable surfaces** in the scene by **raycasting** over game models.
- Stores navigable regions using **triangulated mesh representations**.
- Supports **dynamic obstacle detection** by filtering **unreachable areas**.

### QuadTree Spatial Partitioning
- Implements a **QuadTree-based spatial hierarchy** for **optimized environment queries**.
- Divides the world into **quadrants**, enabling **fast triangle lookup** for AI movement.
- Supports **adaptive partitioning**, ensuring minimal computation overhead.

---

## Output
<img src="https://raw.githubusercontent.com/Xena99/COD-Lite/master/Output.png" width="600" height="400" />

- The AI agent autonomously navigates the environment using pathfinding algorithms to move efficiently across the navigation mesh (displayed in green).
- The navigation mesh is composed of triangular walkable areas, allowing the AI to traverse the environment while avoiding obstacles.
- The blue line represents the path computed using the Funnel Algorithm, which ensures natural and smooth movement across the terrain.
- The yellow triangles indicate the neighboring triangles of the triangle where the AI is currently positioned. These neighboring connections are crucial for    
  pathfinding and traversal decisions.
- The white sphere is the AI’s target position. The AI continuously updates its path to reach this target while navigating around obstacles.
- The obstacles (gray rocky blocks) represent non-walkable areas where triangles are filtered out to prevent AI from attempting movement through solid objects.

---

## Project Structure

The project comprises several core modules:

### **`AIEngine.cpp`** - AI Agent Management
- Initializes the **AI Engine** with references to the **QuadTree and NavMesh**.
- Implements **threaded execution** to continuously update **AI movement**.
- Handles **target updates** dynamically when AI **senses obstacles or objectives**.
- Executes **real-time path recalculations** to ensure smooth navigation.

### **`PathFinding.cpp`** - Path Computation
- Implements the **A* Algorithm** to find the most efficient route between **start and target points**.
- Uses a **min-heap priority queue** to explore **low-cost nodes first**.
- Extracts **portals** between triangles for **smooth transitions**.
- Runs the **Funnel Algorithm** to optimize AI movement along a natural path.

### **`NavMeshUtils.cpp`** - Navigation Mesh Management
- Uses **raycasting** to **detect walkable triangles** on the game terrain.
- Filters **obstructed areas** using **collision checks** against **obstacles**.
- **Visualizes** the navigation mesh using **triangle rendering**.
- Stores **triangle adjacency data** to speed up **pathfinding calculations**.

### **`QuadTree.cpp`** - Spatial Partitioning
- Implements a **QuadTree structure** for **fast spatial queries**.
- **Subdivides** large areas into **smaller nodes** containing walkable triangles.
- Dynamically **redistributes** triangles to prevent **overlapping storage**.
- Provides **efficient lookups** for **AI movement and collision detection**.

---

## Summary
COD-Lite demonstrates an **optimized AI movement system** by leveraging **spatial partitioning with QuadTrees**, **A* pathfinding with funnel optimization**, and **real-time NavMesh updates**. The project is designed to be lightweight while showcasing **high-performance AI behaviors** in **dynamic environments**.

---

This README now **clearly defines the technical aspects** of the project, explaining the **role of each module** while emphasizing the **efficiency and optimization techniques** used. Let me know if you’d like any modifications! 🚀
