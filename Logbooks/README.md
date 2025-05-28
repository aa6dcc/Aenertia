# Ænertia

EEE2 Balancing Robot End of Year Project

## Introduction

The aim of this robot is to provide an indoor assistance to users, by guiding them through a perviously mapped area. 

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

Database structure:

AenerShark/
├── server/
│   ├── database/               # DynamoDB logic & local helpers
│   │   ├── __init__.py
│   │   ├── dynamodb.py         # DynamoDB CRUD functions
│   │   └── utils.py            # Timestamps, file naming, ID gen
│   │
│   ├── lambda/                 # All Lambda code
│   │   └── store_metadata.py   # Triggered after S3 upload
│   │
│   ├── mqtt/                   # MQTT config, client, topics
│   │   └── mqtt_helper.py
│   │
│   ├── s3bucket/               # S3 upload logic
│   │   └── s3_upload.py        # Upload files to S3
│   │
│   ├── tuning/                 # PID tuning logic
│   │   └── pid_handler.py
│   │
│   ├── website/                # Static + JS frontend
│   │   ├── static/
│   │   │   ├── style.css
│   │   │   └── control.js
│   │   └── templates/
│   │       └── dashboard.html  # (if using Jinja2)
│   │
│   ├── main.py                 # FastAPI server.py logic
│   └── aws_config.py           # AWS session, clients, env
│
├── .env                        # (optional) store AWS keys
├── requirements.txt
├── README.md
└── create_db.py                # (optional for initial table setup)

