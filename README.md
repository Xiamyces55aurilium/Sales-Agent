# Autonomous Delivery Agent for 2D Grid City Navigation

## Overview:-
This project designs and implements an autonomous delivery agent that navigates a 2D grid-based city environment.The agent will deliver packages efficiently by safely navigating through varying terrain costs,static obstacles,and dynamic moving obstacles such as vehicles with known schedules.
The system models the environment rigorously,and will integrate multiple search algorithms (uninformed and informed),and also implements a local search-based replanning strategy to handle dynamic changes.

## Features

**Environment Modeling:** Incorporates static obstacles, weighted terrain costs for movement, and time-dependent dynamic obstacles.
**Agent Design:** Rational agent using grid-based path planning and dynamic replanning to maximize delivery efficiency under constraints.

**Implemented Algorithms:**
Uniform-Cost Search (Dijkstra’s algorithm) for uninformed pathfinding.
A* Search with Manhattan distance heuristic for informed pathfinding.
Hill-Climbing with random restarts as a local search replanning method on dynamic obstacle encounters.
**Experimental Framework:**Supports testing on multiple map instances (small,medium,large,and dynamic obstacle maps) with logging and benchmarking of path cost,nodes expanded,and computation time.

## Setup Instructions
### Prerequisites
Python 3.8 or higher installed: [Download Python](https://www.python.org/downloads/)
Compatible on Windows,Linux,or macOS.
The folders should be arranged this way:-
autonomous_delivery_agent/
│
├── main.py                      #Main Python program 
├── maps/                       #Folder containing all map files
│   ├── small_map.txt
│   ├── medium_map.txt
│   ├── large_map.txt
│   └── dynamic_map.txt
├── logs/                       #Folder for simulation and replanning logs 
│   └── replanning_log.txt
├── README.md                   #Detailed project overview and instructions
├── requirements.md             #Project dependencies and setup notes



### Installation
1. Clone or download this repository.

2. Ensure folder structure:

