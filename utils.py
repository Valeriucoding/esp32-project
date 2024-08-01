import time

def run_buzzer_for_duration(buzzer, duration):
    """Runs the buzzer for a specified duration (in seconds) without blocking the loop."""
    buzzer_running = False
    buzzer_start_time = 0

    def start_buzzer():
        nonlocal buzzer_running, buzzer_start_time
        buzzer.freq(100)
        buzzer.duty_u16(1)
        buzzer_running = True
        buzzer_start_time = time.time()

    def stop_buzzer():
        nonlocal buzzer_running
        buzzer.duty_u16(0)
        buzzer_running = False

    def update_buzzer():
        nonlocal buzzer_running, buzzer_start_time
        if buzzer_running:
            current_time = time.time()
            if current_time - buzzer_start_time >= duration:
                stop_buzzer()

    return start_buzzer, update_buzzer


def set_servo_angle(pwm, duty):
    """Sets the servo to the specified duty cycle."""
    pwm.duty_u16(duty)