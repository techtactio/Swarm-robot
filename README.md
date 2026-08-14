🤖 Swarm Robotics Waste Collection System

A decentralized multi-robot system designed to improve waste collection efficiency through area partitioning, autonomous navigation, task allocation, and robot-to-robot communication.

📌 Overview

This project focuses on developing a swarm of autonomous mobile robots that cooperate to collect waste from a predefined area.

Instead of relying on a centralized controller, each robot is designed to make local decisions while sharing relevant information with other robots. The overall collection area is initially divided into regions, and each robot is assigned a specific region to cover.

The project is being developed and tested in simulation using ROS 2 and Gazebo before moving toward physical implementation using ESP32-based robots.

🎯 Objectives
Divide a waste-collection area among multiple robots.
Assign individual regions/tasks to robots.
Enable autonomous navigation and area coverage.
Facilitate communication between robots.
Reduce unnecessary overlap between robots.
Develop a decentralized decision-making approach.
Improve the efficiency and scalability of waste collection.
🧠 Core Approach

The project uses a Distributed Intelligence (DI) approach.

Area Partitioning

The complete environment is divided into approximately equal regions:

+-------------------+-------------------+-------------------+
|                   |                   |                   |
|     Robot 1       |      Robot 2      |      Robot 3      |
|     Region A      |      Region B     |      Region C     |
|                   |                   |                   |
+-------------------+-------------------+-------------------+



Each robot is initially assigned a region and is responsible for waste collection and coverage within that region.

As the system develops, communication between robots can be used to exchange information such as:

Robot position
Region status
Waste locations
Collection progress
Robot availability
Task completion

This allows the swarm to make distributed decisions instead of depending on a single central controller.
