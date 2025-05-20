# Logbook - Antone

## 19/05/2025

First day of project. Built a high-level diagram covering the different software aspects of the project and how they link to one another, such as the UI, the backend website server, TCP server, microcontroller, AWS...
Also looked at past reports, thoroughly read through the instructions and documents and looked at different pathfinding algorithms for how to deal with the autonomous navigation

## 20/05/2025

Second day. Started researching how to build a website using JavaScript, HTML & CSS. At first I want to build a simple website with nothing much on it which just has the capacity to connect to the Raspberry Pi, then I want to add functionalities like seeing battery usage, time, being able to control the robot...
Thinking about what pathfinding algorithm would be optimal - A*, Dijsktra or something else perhaps?

Created a test website to see if it can communicate with the Raspberry Pi:

![image](https://github.com/user-attachments/assets/c4276dd6-73ed-4b51-abad-90e6941cfa66)

only option is click me:

![image](https://github.com/user-attachments/assets/1fc2818f-bbc8-41ce-8aca-879aa63aaf54)

We then created a basic server.js, git cloned the repo into the raspberry pi, downloaded Node.js and Express onto it, checked the IP via hostname -I and was able to successfully run the website onto the raspberry pi (both PC and raspberry pi connected to hotspot)

![image](https://github.com/user-attachments/assets/6f8fdb4e-5278-4caa-aeb9-7a2c6cda4053)

Added a flash LED button to the website: we can click it as many times as we wish and every time we do, we flash the LED

![image](https://github.com/user-attachments/assets/b0d10b7d-19a0-4b2b-8ba8-139483545fa4)


