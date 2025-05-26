#  PID CONTROL LOG
## Jay Dong
<br>

# Tried Approaches for Inner Loop (Balancing)
<br>

### 1: Speed control

$$v = k_pe_\theta + k_d \frac{d e_\theta}{dt}$$

Result: Acceleration is constant, robot not responding in time when Kp is small and shaking when acceleration is too large. Unable to balance. 

### 2: Acceleration Control

$$a = k_pe_\theta + k_d \frac{d e_\theta}{dt}$$

$$
\text{speed} = 
\begin{cases} 
20 & \text{if } \text{acceleration} < 0 \\
-20 & \text{if } \text{acceleration} > 0
\end{cases}
$$

Result: Acceleration can be controled but not speed, may accelerate the maximum speed and stay there.

### 3: Parallel Two Loop Control

$$v_{target} = ke_\theta$$
$$a = k_pe_{\theta} + k_d \frac{d e_\theta}{dt}$$

Result: Very Slow Response at small angles which leads to falling

### 4: Acceleration Based on Error in Speed

$$v_{target} = k_pe_\theta + k_d \frac{d e_\theta}{dt}$$
$$a = k_{pa}(V_{target} - V_{real}) $$

Result: Similar with above, slow response at small angles

### 5: Cascade Loop Control:  Speed Error &rarr; Desired Gyro &rarr; Accleration

$$\omega_{target} = k_{po}e_\theta + k_{do} \frac{d e_\theta}{dt}$$
$$a = k_{pi}e_\omega + k_{di} \frac{d e_\omega}{dt} $$

In this code, we do the same thing as we did in the control lab. Desired angular velocity is PID controlled by the error in tilt angle. Then, the instantaneous gyro is recorded to find the the error between desired and real angular velocity of the robot and the acceleration is proportional to that. <br>
Result: It hard to tune because it has more parameters that the single loop model. However, the robot is able to stand on itself and able to adjust quickly to changes now using this code. However, it is still not stable when an external force is applied on it, this would also be a challenge when doing the velocity outer loop. 

### Extra tools/functions used in autobalance control:
&nbsp;&nbsp;&nbsp;&nbsp;Moving Average Filter (Weighted)  
&nbsp;&nbsp;&nbsp;&nbsp;Complementary Filter  
&nbsp;&nbsp;&nbsp;&nbsp;Kalman Filter  
&nbsp;&nbsp;&nbsp;&nbsp;Simple Low Pass Averaging filter  
<br>


# Tried Approaches for Outer Loop (Velocity)

### 1: Direct PID Control of Theta Angle

$$\theta_{target} = k_{pv}e_v + k_{dv} \frac{d e_v}{dt}$$

Since horizontal acceleration of the bot is proportional to the the tilt angle of the bot, we can change the tilt angle to obtain acceleration in a specified direction.  
However, during the process, we should strictly limit the theta angle to make it stay in the range where motors are still able to respond. 
<br><br>

# Server for Tuning PID
### To make the tuning process of PID quicker, a server is build on raspberry to send serial data to ESP32. 
1. I set up a python flask server that can receive protocols from the webpage interface. On the webpage interface, there are input boxes for the kp, ki, kd and sepoint variables. 
2. When the python flask server receives information from the webpage interface running on phone/computer, it encodes the information with a space between each other and send them to the serial port of the esp32. 
3. The esp32 listens to its serial port with 9600 baud rate on port 25 and 26 continuously. When it receives information, it will parse it and save it to the kp, ki, kd variables and update the PID loop.
4. Finally, print the values to serial monitor so we can see if the values are successfully sent and received. 

A problem of power raised during the process. Raspberry pi would automatically restart due to low voltage when motors are running. This is caused by motors drawing to much current from the circuit board and voltage regulaion is not done well enough to conpenstate for the power drawn. This should be fixed by further power monitoring. 

### Server should also be used in making a remote controller.
<br>

# Power Monitoring
## Basic Task:
The task for power monitoring is to measure the current and voltage for two things: The 15V motors and the rest 5V components.
Both nodes has a shunt resistor connected in series on the PCB used solely for current measurement. 
When current goes through the shunt resistor, there would be a slight voltage drop that is proportional to its resistance. Thus, by measureing this value using an ADC, we are able to find the current and power supplied in this branch of the system. 

## ADC Choice

The ADC choice would be MCP3208 since it is a most common 8 channel ADC that is compatable with the code provided. It has 12bits precision which is much better than the performance of the built-in analog input ports of the ESP32.

## Measurement and Amplification

In between the battery and the motors, there is a $100m\Omega$ resistor. The current to the motors is measured to be around 1A-1.5A when the motor is running at full speed, and the celing for current should be 2A since the fuse would burn if it goes above. 
Thus, the range of voltage across the shunt resistance should be 0 to 0.2V. 
The full range of ADC conversion is 0 to 3.3V, so an amplification of 15.5 times is required. 

In between the buck converter and all 5V devices (esp32 and raspberry pi), there is a $10m\Omega$ resistor. The current should also be lower than 2A so a maximum of 0.02V is calculated. 
This value need to be amplified by 155 times to reach 3.3V.

## Measuring Instructions
Amplifier1 should be connected to 0V (GND), 3.3V (Vs), I5+ (V+), 5V (V-). 

Amplifier2 should connected to 0V (GND), 3.3V (Vs), IM+ (V+), IMOT (V-). 

ADC should be connected to 0V (GND), 3.3V (Vcc), Amplifier1 Output (Channel1), Amplifier2 Output (Channel3).

# Circuit Diagram
![Circuit](JD_Files/Circuit%20Diagram%20PCB.png)













