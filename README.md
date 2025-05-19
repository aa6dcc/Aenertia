# AenerShark
EEE2 Balancing Robot End of Year Project

Notes for  "Communications" team:

1) start by building website which can communicate with the server and control the robot by providing an interface and sending control commands: use JS, CSS, HTML
2) build TCP server (client = website, server = robot) : use Python, send commands with HTTP (POST/GET)
3) build and integrate AWS DynamoDB NoSQL database
4) implement autonomous navigation and mapping of unseen mazes with integrated real-time location tracking and obstacle management: use A* algorithm for optimal maze pathfinding and craft a Pygame visual demonstration

TCP is used to communicate berween website and TCP server, UART is used to communicate between the TCP server and the microcontroller

Full Architecture:

Website (client)
(React, JS, HTML)
       ↓ (HTTP)
Backend Web Server 
(Node.js, Flask, Express) 
hosted on Raspberry Pi
       |
  TCP socket
       ↓ 
  TCP server
       |
     UART
       ↓ 
     Robot 
 microcontroller
    (ESP32)
      ↓ 
  Data Capture
(Logs, images, sensor dumps)
      ↓ 
 Upload to S3
(store and retrieve)
      ↓
Trigger Lambda
      ↓
store metadata in
   dynamoDB
(CRUD architecture)
