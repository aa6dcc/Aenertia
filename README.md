# Ænertia

EEE2 Balancing Robot End of Year Project

## Introduction

The aim of this robot is to provide an indoor assisantce to users, by guiding them through a perviously mapped area. 

**The bot should satisfy the following requirements:**

  - Balance and allow manual control of the acceleration and direction,

  - Map the area it exists in and remember key locations,

  - Detect humans and follow them when required,
    
  - Autonomously go to a key location
    
<ins> Advanced optional features </ins>

  - Allow vocal commands to control the bot,

  - Determine the fastest path to a given location when required,
    
  - Have a speaker for user friendly interactions



[Instructions](https://github.com/edstott/EE2Project/blob/main/balance-robot/README.md)







### Notes for  "Communications" team:

1) start by building website which can communicate with the server and control the robot by providing an interface and sending control commands: use JS, CSS, HTML
2) build TCP server (client = website, server = robot) : use Python, send commands with HTTP (POST/GET)
3) build and integrate AWS DynamoDB NoSQL database
4) implement autonomous navigation and mapping of unseen mazes with integrated real-time location tracking and obstacle management: use A* algorithm for optimal maze pathfinding and craft a Pygame visual demonstration
5) implement computer vision for obstacle detection/avoidance, gesture recognition

To do list:

- website to tune PID values
- implement camera snapshot
- upgrade website capabilities
- flash LED from phone
- list instructions for how to connect website to Raspberry Pi

TCP is used to communicate berween website and TCP server, UART is used to communicate between the TCP server and the microcontroller

Full Architecture:

![image](https://github.com/user-attachments/assets/aeba1356-e19b-4993-99e6-69086c825837)

