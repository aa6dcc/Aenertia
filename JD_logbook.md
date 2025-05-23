#  PID CONTROL LOG
## Jay Dong
<br>

## Tried Approaches
<br>

### 1: Speed control

$$v = k_pe_\theta + k_d \frac{d e_\theta}{dt}$$

### Result: Acceleration is constant, robot not responding in time when Kp is small and shaking when acceleration is too large. Unable to balance. 

### 2: Acceleration Control

$$a = k_pe_\theta + k_d \frac{d e_\theta}{dt}$$

$$
\text{speed} = 
\begin{cases} 
20 & \text{if } \text{acceleration} < 0 \\
-20 & \text{if } \text{acceleration} > 0
\end{cases}
$$

### Result: Acceleration can be controled but not speed, may accelerate the maximum speed and stay there.

### 3: Parallel Two Loop Control

$$v_{target} = ke_\theta$$
$$a = k_pe_{\theta} + k_d \frac{d e_\theta}{dt}$$

### Result: Very Slow Response at small angles which leads to falling

### 4: Acceleration Based on Error in Speed

$$v_{target} = k_pe_\theta + k_d \frac{d e_\theta}{dt}$$
$$a = k_{pa}(V_{target} - V_{real}) $$

### Result: Similar with above, slow response at small angles


