# Logbook - Antoine

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

## 21/05/2025

In order to connect the Raspberry pi and make it run the website:
  - Download Python, Node, Express onto the Raspberry Pi (already done in our case)
  - No need to git clone this repo as it is already done, instead
    "cd AenerShark" => "git pull" => "cd AenerShark_Website"
  - type "hostname -I" and you will get an IP of the form: 172.20.10.2
  - type "node server.js" and you will get a message of the form: Server running at http://localhost:5000
  - on your PC, type "http://172.20.10.2:5000" (or replace 172.20.10.2 by the IP you were given and 5000 by the port number you were given after typing node server.js"
  - make sure the Raspberry Pi / laptop are connected to the same WiFi/mobile hotspot

Updated the website to have two tabs: PID values & Testing

![image](https://github.com/user-attachments/assets/3a9b5ea3-0a4b-4858-a5f0-938df428e5cf)

When connected to the Raspberry Pi, the website will give the user the ability to type in integer values for the different PID parameters and will have a table which stores these past inputs. 

## 22/05/2025

The draft website currently implemented is mostly comprised of index.html (the website is opened by simply clicking on index.html), as well as server.js, script.js and styles.css. 

Researched MQTT, why use it over HTTP, and how to implement it. In our case we will use the Paho library in Python to implement MQTT. 
We needed a broker to establish MQTT connections, such as a free-tier EC2 t2.micro running Mosquitto or HiveMQ. 
We measure the average RTT for a large number of messages in order to measure the performance of MQTT. 

MQTT (Message Queuing Telemetry Transport) is a publish-subscribe application layer protocol designed for constrained devices and low-bandwidth, high-latency, or unreliable networks. It is optimized for low power, low memory, and low network usage environments. It uses TCP/IP as its transport protocol. Nonetheless, MQTT can sometimes suffer from memory leaks, connection failures or message delivery problems. We can try and deal with these issues by using well-established and maintained libraries such as Paho, monitor memory usage, keep queues short, enable session persistence, use keep-alive pings, and enable automatic reconnect on the client-side (reconnect_delay_set() in Paho). HiveMQ allows us to do load balancing and we should try and minimise message size.

| Feature                | MQTT                                | HTTP                                 | WebSocket                           |
|------------------------|-------------------------------------|--------------------------------------|-------------------------------------|
| **Communication Type** | Publish/Subscribe                   | Request/Response                     | Full-duplex, Bidirectional          |
| **Connection**         | Persistent (TCP)                    | Stateless (new TCP for each request) | Persistent (over TCP)               |
| **Latency**            | Low                                 | Moderate                             | Low                                 |
| **Efficiency**         | High (small packets, low overhead)  | Low (larger headers, repeated setup) | Moderate                            |
| **Security**           | SSL/TLS                             | SSL/TLS                              | SSL/TLS                             |
| **Resource Usage**     | Very Low                            | High                                 | Moderate                            |
| **Requirement**        | Broker (e.g., Mosquitto)            | Web Server                           | WebSocket Server                    |
| **QoS Support**        | Yes                                 | No                                   | No                                  |
| **Built-in Ack**       | Yes                                 | Yes (via status codes)               | Limited (application-defined)       |
| **Mobile Support**     | Good                                | Good                                 | Good                                |


We then needed to update our web backend so we made a comparison of the relevant framework and APIs:

| **Feature**            | **FastAPI**                           | **Python REST API (Generic)**       | **Node.js**                         | **Flask**                            |
|------------------------|---------------------------------------|-------------------------------------|-------------------------------------|--------------------------------------|
| **Type**               | Web framework (Python)                | API toolkit/framework               | Runtime + frameworks (e.g. Express)| Web framework (Python)               |
| **Language**           | Python                                | Python                              | JavaScript / TypeScript             | Python                               |
| **Performance**        | Very high                             | Moderate                            | High                                | Moderate                             |
| **Ease of Use**        | High                                  | Moderate                            | Moderate                            | Very high                            |
| **Community Support**  | Growing rapidly                       | Mature (Django/DRF very popular)    | Massive                             | Mature                               |
| **Built-in Routing**   | Yes                                   | Yes (DRF / Flask-RESTful, etc.)     | Yes (Express, Fastify)              | Yes                                  |
| **Scalability**        | Good                                  | Moderate                            | Excellent                           | Limited 


I also got to do some research on different pathfinding algorithms:

| Algorithm         | Handles Weights? | Uses Heuristic? | Optimal?                        | Notes                               |
| ----------------- | ---------------- | --------------- | ------------------------------- | ----------------------------------- |
| BFS               | No               | No              | Yes (unweighted)                | Good for unweighted grids           |
| DFS               | No               | No              | No                              | Can go deep but take bad paths      |
| Dijkstra          | Yes              | No              | Yes                             | Slower than A\* with heuristics     |
| Greedy Best-First | Yes              | Yes             | No                              | Fast, not optimal                   |
| A\*               | Yes              | Yes             | Yes (with admissible heuristic) | Best general-purpose algorithm      |
| Bidirectional     | Depends          | Optional        | Depends                         | Faster in certain structured graphs |
| Floyd-Warshall    | Yes              | No              | Yes                             | All-pairs, slow                     |

This led me to the conclusion that A* is the best pathfiding algorithm, especially in the context of a robot which has the ability to find paths and follow people.

## 23/05/2025

Finally managed to create the UI for an arrow pad with a stop button in the middle. 
I implemented a click and hold logic. 

![image](https://github.com/user-attachments/assets/b5d8ff19-0729-44ed-8c55-d3886cb05dff)

I then added the Aenertia logo to the website so it looks like this:

![image](https://github.com/user-attachments/assets/2a834e82-afbf-470f-b8cd-7558645d683e)

The server is implemented using FastAPI, and the link to access the website I am using is set as http://localhost:8000/pad

The next step will be to control the robot not only by clicking buttons on the website but also by using the arrows on my keyboard, which would lead to a slight animation of the arrows on the website.
I will also try and make the website look better by enhancing some of the effects and adding some color.

Made some changes to the website:

![image](https://github.com/user-attachments/assets/237f3a62-89c0-432c-9bf3-cf2ca41f08ed)

I tried to make the control pad neater, and now the robot can be controlled not only using the arrow keys on the website but also using the keyboard arrows.
In the top right corner I also added a battery feature (the battery of my computer - this is a draft, in the future this will be the battery of the robot) as well as the time.

I would like for this website to have 3 separate tabs (PID control, flash LED and control robot pad) in the future to make the website more neat and organized.

