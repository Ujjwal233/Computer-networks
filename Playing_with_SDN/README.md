# Assignment 3: Introduction to SDN (Software Defined Networking)

**Course:** COL 334/672, Diwali'24  

## Overview

This project explores key concepts in Software Defined Networking (SDN), specifically focusing on OpenFlow-like APIs. The assignment is divided into three main parts, with an optional bonus part. Each part involves implementing different network control policies using the Ryu controller and simulating network behavior with Mininet.



## Part 1: Hub and Learning Switch

This part compares the performance of a **Hub Controller** and a **Learning Switch** in terms of:
- Flow rule installation.
- Throughput performance (tested using `iperf` between hosts h1 and h5).

### Files:
- `p1_hub.py`: Hub Controller implementation.
- `p1_learning.py`: Learning Switch implementation.

## Part 2: Spanning Tree Implementation

This part modifies the Learning Switch app to prevent packet loops in cyclic network topologies by constructing a **Spanning Tree**. The controller forwards broadcast packets to open ports in the tree and to attached hosts.

### Files:
- `p2_spanning_tree.py`: Spanning Tree implementation.

## Part 3: Shortest Path Routing

Implements **Shortest Path Routing** based on link capacities, while maintaining L2 routing. The controller builds shortest path trees from each switch to the hosts.

### Files:
- `p3_spr.py`: Shortest Path Routing implementation.


## Setup and Installation

1. **Install Mininet:**
   Mininet is required for network topology simulation. You can install it on Linux or use a virtual machine.  
   [Mininet Installation Instructions](https://mininet.org/download/)

2. **Install Ryu Controller:**
   Ryu will be used to implement the controllers.  
   [Ryu Installation Guide](https://ryu.readthedocs.io/en/latest/getting_started.html)

## Running the Controllers

To run the controllers with the Mininet topologies provided in the assignment, use the following commands:
- **Running Topo File:**
  ```bash
  sudo python3 <topo.py>
-**Now Open new terminal**
- **Running a Ryu App:**  
  ```bash
  ryu-manager --observe-links <app.py>
