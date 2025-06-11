E_max = 92289
Ev_last = E_max
while(True):
    voltage = float(input("Current Voltage: "))
    if(voltage > 14.8):
        Ev = 13009*voltage - 115094
    elif(voltage > 14.2):
        Ev = 96166*voltage - 1346076
    elif(voltage > 12.77):
        Ev = 12606*voltage - 159533
    else:
        Ev = 314*voltage - 2460
    
    if Ev > Ev_last:
        Ev = Ev_last
    Ev_last = Ev
    
    percentage = Ev/E_max
    print("Energy = ",Ev,"\nPercentage = ", int(percentage*100), "%")
    
    
