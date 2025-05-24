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








