from machine import ADC, Pin, PWM, I2C
import time
from pico_i2c_lcd import I2cLcd
import tm1637

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

while True:
    motion_detected = pir_sensor.value()
    if motion_detected:
        buzzer.freq(10)
        buzzer.duty_u16(1)

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
