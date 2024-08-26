from machine import Pin, time_pulse_us, ADC, I2C, SPI
import time
from imu import MPU6050
from rotary_irq_rp2 import RotaryIRQ
from max7219 import Matrix8x8

# Initialize ultrasonic sensor
SOUND_SPEED = 340
TRIG_PULSE_DURATION_US = 10

trig_pin = Pin(21, Pin.OUT)
echo_pin = Pin(20, Pin.IN)

# Initialize joystick
xAxis = ADC(Pin(27))
yAxis = ADC(Pin(26))
button = Pin(16, Pin.IN, Pin.PULL_UP)

# Initialize MPU 6050
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
imu = MPU6050(i2c)

# Initialize 4x4 keypad
matrix_keys = [['1', '2', '3', 'A'],
               ['4', '5', '6', 'B'],
               ['7', '8', '9', 'C'],
               ['*', '0', '#', 'D']]

keypad_rows = [15, 14, 13, 12]
keypad_columns = [11, 10, 9, 8]

col_pins = []
row_pins = []

for x in range(0, 4):
    row_pins.append(Pin(keypad_rows[x], Pin.OUT))
    row_pins[x].value(1)
    col_pins.append(Pin(keypad_columns[x], Pin.IN, Pin.PULL_DOWN))
    col_pins[x].value(0)


def scankeys():
    for row in range(4):
        for col in range(4):
            row_pins[row].high()
            key = None

            if col_pins[col].value() == 1:
                print("You have pressed:", matrix_keys[row][col])
                key_press = matrix_keys[row][col]
                time.sleep(0.3)

        row_pins[row].low()


# Initialize rotary encoder
r = RotaryIRQ(
    pin_num_clk=17,
    pin_num_dt=18,
    reverse=False,
    incr=1,
    min_val=0,
    max_val=20,
    range_mode=RotaryIRQ.RANGE_BOUNDED,
    pull_up=True,
    half_step=False,
)

val_old = r.value()
print(val_old)

# led pins

led_pin = 6

# Initialize the LED pin as an output
led = Pin(led_pin, Pin.OUT)

# Turn on the LED
led.value(1)

# Wait for a short period
time.sleep(1)

# Turn off the LED
led.value(0)

while True:
    # ultrasonic sensor
    trig_pin.value(0)
    time.sleep_us(5)
    trig_pin.value(1)
    time.sleep_us(TRIG_PULSE_DURATION_US)
    trig_pin.value(0)

    ultrason_duration = time_pulse_us(echo_pin, 1, 30000)
    distance_cm = SOUND_SPEED * ultrason_duration / 20000
    # print(f"Distance : {distance_cm} cm")

    # joystick
    xValue = xAxis.read_u16()
    yValue = yAxis.read_u16()
    buttonValue = button.value()
    # print("X Value:", xValue, "Y Value:", yValue, "Button Value:", buttonValue)
    time.sleep_ms(500)
    #
    ax = round(imu.accel.x, 2)
    ay = round(imu.accel.y, 2)
    az = round(imu.accel.z, 2)
    gx = round(imu.gyro.x)
    gy = round(imu.gyro.y)
    gz = round(imu.gyro.z)
    tem = round(imu.temperature, 2)
    # print("ax",ax,"\t","ay",ay,"\t","az",az,"\t","gx",gx,"\t","gy",gy,"\t","gz",gz,"\t","Temperature",tem,"        ",end="\r")
    # keypad
    scankeys()

    # rotary encoder
    val_new = r.value()

    if val_old != val_new:
        val_old = val_new
        print("step =", val_new)

