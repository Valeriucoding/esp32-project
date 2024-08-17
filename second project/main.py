from machine import Pin, time_pulse_us, ADC
import time
# ultrasonic sensor
SOUND_SPEED=340
TRIG_PULSE_DURATION_US=10

trig_pin = Pin(21, Pin.OUT)
echo_pin = Pin(20, Pin.IN)
# joystick
xAxis = ADC(Pin(27))
yAxis = ADC(Pin(26))
button = Pin(16, Pin.IN, Pin.PULL_UP)

while True:
    trig_pin.value(0)
    time.sleep_us(5)
    trig_pin.value(1)
    time.sleep_us(TRIG_PULSE_DURATION_US)
    trig_pin.value(0)

    ultrason_duration = time_pulse_us(echo_pin, 1, 30000)
    distance_cm = SOUND_SPEED * ultrason_duration / 20000

    print(f"Distance : {distance_cm} cm")
    xValue = xAxis.read_u16()
    yValue = yAxis.read_u16()
    buttonValue = button.value()
    print("X Value:", xValue, "Y Value:", yValue, "Button Value:", buttonValue)
    time.sleep_ms(500)
