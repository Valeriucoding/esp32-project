from machine import ADC, Pin, PWM, I2C
import time
from pico_i2c_lcd import I2cLcd
import tm1637
from utils import run_buzzer_for_duration, set_servo_angle

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
# Initialize relay
relay = Pin(18, Pin.OUT)

start_buzzer, update_buzzer = run_buzzer_for_duration(buzzer, 1)
car_in_queue = False
num_of_cars_approved = 0
button_last_state = button.value()
servo_position = 4750
while True:
    motion_detected = pir_sensor.value()
    if motion_detected and not buzzer.duty_u16() and not car_in_queue:
        car_in_queue = True
        start_buzzer()
        lcd.clear()
        relay.on()
        lcd.putstr("Car is at the barrier!")
    button_current_state = button.value()
    if button_current_state == 0 and button_last_state == 1 and car_in_queue:
        car_in_queue = False
        lcd.clear()
        lcd.putstr("Approved!")
        num_of_cars_approved += 1
        tm.number(num_of_cars_approved)

        for position in range(4750, 9000, 50):
            pwm.duty_u16(position)
            time.sleep(0.01)
        time.sleep(5)
        for position in range(9000, 4750, -50):
            pwm.duty_u16(position)
            time.sleep(0.01)
        relay.off()
    update_buzzer()

    pot_value = pot.read_u16()
    new_servo_position = int(4750 + (pot_value / 65535) * (9000 - 4750))
    if new_servo_position != servo_position:
        servo_position = new_servo_position
        set_servo_angle(pwm, servo_position)

    if motion_detected:
        print("Motion detected!")
    else:
        print("No motion")
    time.sleep(0.1)