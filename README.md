# IROBOT

*Empowering Robots for Seamless Inventory Management in Small to Medium Warehouses*

![Last Commit](https://img.shields.io/github/last-commit/Faheemibrahim/IRobot)
![Languages](https://img.shields.io/github/languages/count/Faheemibrahim/IRobot)
![Top Language](https://img.shields.io/github/languages/top/Faheemibrahim/IRobot)

Built with the tools and technologies:

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![TurtleBot3](https://img.shields.io/badge/TurtleBot3-WafflePi-lightgreen)
![Taipy](https://img.shields.io/badge/-Taipy-purple)
![MongoDB](https://img.shields.io/badge/-MongoDB-green)
![Python](https://img.shields.io/badge/-Python-blue)
![C++](https://img.shields.io/badge/-C++-00599C)
![Lua](https://img.shields.io/badge/-Lua-000080)
![XML](https://img.shields.io/badge/-XML-orange)
![YAML](https://img.shields.io/badge/-YAML-red)
![OpenCV](https://img.shields.io/badge/-OpenCV-black)
![Git](https://img.shields.io/badge/-Git-orange)

---

## 📑 Table of Contents

- [Overview](#overview)
- [📽️ Demo / How It Works](#demo--how-it-works)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
  - [Usage](#usage)
  - [Testing](#testing)
- [Technologies](#technologies)
- [Folder Structure](#folder-structure)
- [Future Work](#future-work)
- [Acknowledgments](#acknowledgments)

---

## 📌 Overview

**IROBOT** is an autonomous warehouse inventory management system developed as part of a final year project at Heriot-Watt University. Designed for small to medium-sized warehouse environments, the system uses a ROS 2-powered TurtleBot3 Waffle Pi to autonomously navigate, scan, and monitor inventory shelves.

The robot identifies items using AprilTags, detects missing or unknown items, and updates this data in real-time to a user-friendly web interface built with Taipy. The system integrates key robotics and data technologies such as SLAM for mapping, Nav2 for navigation, MongoDB for inventory storage, and OpenCV for perception tasks.

🎓 University: Heriot-Watt University

👨‍🏫 Supervisor: Dr. Nidhal Abdulaziz

👨‍💻 Student: Mohammed Ibrahim

---

## 📽️ Demo / How It Works

Watch the robot in action across two warehouse scenarios:

### 🔹 Test Case 1 – Identifying 9 Known Items
[![Test Case 1](https://img.youtube.com/vi/vr4dMc1UE1U/0.jpg)](https://youtu.be/vr4dMc1UE1U?si=JOIPj0qWZc1x-HAi)

- Demonstrates scanning a fully stocked shelf with 9 known items.
- Verifies AprilTag detection and inventory logging.

### 🔹 Test Case 2 – Handling 2 Missing Items, 4 Unknowns, and 3 Known Items
[![Test Case 2](https://img.youtube.com/vi/_uMRsMMcNog/0.jpg)](https://youtu.be/_uMRsMMcNog?si=caOtP-Oy-F5uPxns)

- Detects shelf with mixed inventory (known, unknown, and missing items).
- Robot receives command via the Taipy Web UI and moves to the target shelf.
- Demonstrates full integration of navigation, detection, and web control.

---
