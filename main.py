from machine import ADC, Pin
import time

pot = ADC(Pin(26))

while True:
    pot_value = pot.read_u16() 
    
    percentage = (pot_value / 65535) * 100
    
    print("Potentiometer Value:", pot_value, "Percentage:", percentage)
    
    time.sleep(0.1) 
