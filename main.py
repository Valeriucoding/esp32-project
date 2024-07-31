from machine import ADC, Pin, PWM
import time

# Initialize potentiometer pin
pot = ADC(Pin(26))
# Initialize pir motion sensor pin
pir_sensor = Pin(0, Pin.IN)
# Initialize buzzer pin
buzzer = PWM(Pin(1))
buzzer.freq(1047)
buzzer.duty_u16(0)
# Initialize pushbutton pin
button = Pin(11, Pin.IN, Pin.PULL_UP)

while True:
    # Potentiometer
    pot_value = pot.read_u16()

    percentage = (pot_value / 65535) * 100

    print("Potentiometer Value:", pot_value, "Percentage:", percentage)

    # Motion Sensor
    motion_detected = pir_sensor.value()

    if motion_detected:
        print("Motion detected!")
    else:
        print("No motion")

    # Buzzer
    if motion_detected:
        buzzer.freq(100)
        buzzer.duty_u16(1)
    else:
        buzzer.duty_u16(0)

    # Button
    if button.value() == 0:
        print("Button pressed!")
    else:
        print("Button not pressed")

    time.sleep(0.1)