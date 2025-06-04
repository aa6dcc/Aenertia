In this section I test the communication process by connecting a UI to a PI and using a python script to turn on either LED 0 1

Run the server: ```uvicorn blink:app --host 0.0.0.0 --port 8000```

Open the UI in a browser

Run the python code for LED: ```python3 gpio_mqtt_PI5```

