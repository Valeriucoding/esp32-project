from machine import ADC, Pin, PWM, I2C
import time
from pico_i2c_lcd import I2cLcd
import tm1637
from utils import run_buzzer_for_duration

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
# Initialize i2c lcd
i2c = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000)
I2C_ADDR = i2c.scan()[0]
lcd = I2cLcd(i2c, I2C_ADDR, 2, 16)
# Initialize 4 digit display
tm = tm1637.TM1637(clk=Pin(5), dio=Pin(4))
# Initialize servo
pwm = PWM(Pin(15))
pwm.freq(50)

start_buzzer, update_buzzer = run_buzzer_for_duration(buzzer, 1)
car_in_queue = False
num_of_cars_approved = 0
button_last_state = button.value()
while True:
    motion_detected = pir_sensor.value()
    if motion_detected and not buzzer.duty_u16() and not car_in_queue:
        car_in_queue = True
        print("Motion detected")
        # buzzer.freq(100)
        # buzzer.duty_u16(1)
        start_buzzer()
        lcd.clear()
        lcd.putstr("Car is at the barrier!")
    button_current_state = button.value()
    if button_current_state == 0 and button_last_state == 1 and car_in_queue:
        car_in_queue = False
        lcd.clear()
        lcd.putstr("Approved!")
        num_of_cars_approved += 1
        tm.number(num_of_cars_approved)
        # Raise the servo motor by 90 degrees
        # pwm.duty_u16(9000)

    update_buzzer()
    for position in range(3000, 9000, 50):
        pwm.duty_u16(position)
        time.sleep(0.01)
    time.sleep(5)
    for position in range(9000, 3000, -50):
        pwm.duty_u16(position)
        time.sleep(0.01)
    time.sleep(5)
    # for position in range(1000,9000,50):
    #     print(position)
    #     pwm.duty_u16(position)
    #     time.sleep(0.01)
    # for position in range(9000,1000,-50):
    #     pwm.duty_u16(position)
    #     time.sleep(0.01)

    if motion_detected:
        print("Motion detected!")
    else:
        print("No motion")
    time.sleep(0.1)

    # # Potentiometer
    # pot_value = pot.read_u16()

    # percentage = (pot_value / 65535) * 100

    # print("Potentiometer Value:", pot_value, "Percentage:", percentage)

    # # Motion Sensor
    # motion_detected = pir_sensor.value()

    # if motion_detected:
    #     print("Motion detected!")
    # else:
    #     print("No motion")

    # # Buzzer
    # if motion_detected:
    #     buzzer.freq(100)
    #     buzzer.duty_u16(1)
    # else:
    #     buzzer.duty_u16(0)

    # # Button
    # if  button.value() == 0:
    #     print("Button pressed!")
    # else:
    #     print("Button not pressed")

    # # Lcd
    # lcd.putstr("Hello world!")
    #  # servo
    # for position in range(1000,9000,50):
    #     pwm.duty_u16(position)
    #     time.sleep(0.01)
    # for position in range(9000,1000,-50):
    #     pwm.duty_u16(position)
    #     # time.sleep(0.01)

    # time.sleep(0.1)
    # # 4 digit display
    # tm.show('help')
    # lcd.clear()
