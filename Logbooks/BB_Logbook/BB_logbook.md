# Benoit Logbook


## Setting up the SSH connection

## Testing Camera

## Testing basic communication

## Intro to MQTT

MQTT: MQ Telemetry transport. It allows a client to "publish" or "subscribe" allowing communication between devices

<img src="https://github.com/aa6dcc/Aenertia/blob/main/assets/Capture%20d%E2%80%99e%CC%81cran%202025-05-24%20a%CC%80%2014.51.34.png?raw=true" width="850"/>


### Publish/Subscribe


Devices publish messages to named topics, and other devices subscribe to those topics to receive them.

Devices can:

- Publish to a topic = send a message on a channel

- Subscribe to a topic = listen for messages on that channel

Topics take the form of strings such as "cv/command", "cv/person_detected", "robot/move", "battery/level"

<br>

<ins>Example: </ins> UI -> PI

- UI Publishes to topic ```cv/command``` -> ```"enable_cv```

- Python code on the PI subscribes to ```cv/command```:

```
def on_message(client, userdata, msg):
  if msg.topic == "cv/command":
    command = msg.payload.decode()

    if command == "enable_cv":
      start_cv_loop()
```

<ins>Example: </ins> PI -> UI

- PI Publishes to topic ```cv/person_detected``` 

- UI subscribes to that topic using MQTT.js:

```
client.on('message', function (topic, message) {
  if (topic === 'cv/person_detected') {
    const detected = message.toString() === 'true';
    updateUIDetectionState(detected);
  }
});
```

### Broker

The broker is the central server that receives, filters, and delivers all MQTT messages.

A broker is basically:

- A registry of who is connected

- A map of who is subscribed to what

- A loop that receives messages and dispatches them to the right clients

### Programming Proccedure

1. Set up the broker (just install and run Mosquitto)

2. Write your clients (UI, Pi, ESP32) to connect to it






