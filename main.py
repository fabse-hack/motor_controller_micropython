from machine import I2C, Pin, PWM, SPI
import time
import sys
import neopixel
import math

# AS5600 Register Konstanten
AS5600_I2C_ADDR = 0x36
RAW_ANGLE_REGISTER_MSB = 0x0C
RAW_ANGLE_REGISTER_LSB = 0x0D
ANGLE_REGISTER_MSB = 0x0E
ANGLE_REGISTER_LSB = 0x0F
STATUS_REGISTER = 0x0B
AGC_REGISTER = 0x1A
MAGNITUDE_REGISTER_MSB = 0x1B
MAGNITUDE_REGISTER_LSB = 0x1C


class DCMotor:
    def __init__(self, pin1, pin2, frequency=15000, min_duty=0, max_duty=1023):
        self.pin1 = PWM(Pin(pin1), freq=frequency)
        self.pin2 = PWM(Pin(pin2), freq=frequency)
        self.min_duty = min_duty
        self.max_duty = max_duty
        self.speed = 0

    def duty_cycle(self, speed):
        speed = max(self.min_duty, min(self.max_duty, speed))
        return speed

    def set_speed(self, speed: int):
        if speed > 0:
            # right turn
            self.speed = self.duty_cycle(speed)
            self.pin1.duty(0)
            self.pin2.duty(self.speed)
            print(f"Motor Rechtslauf: Pin1=0, Pin2={self.speed}")
        elif speed < 0:
            # left turn
            self.speed = self.duty_cycle(-speed)
            self.pin1.duty(self.speed)
            self.pin2.duty(0)
            print(f"Motor Linkslauf: Pin1={self.speed}, Pin2=0")
        else:
            self.stop()

    def stop(self):
        self.pin1.duty(0)
        self.pin2.duty(0)
        self.speed = 0
        print("Motor gestoppt.")


class AS5600:
    def __init__(self, i2c):
        self.i2c = i2c
        self.last_position = self.read_registers()
        self.positions = [0, 0, 0]
        self.last_position = 0
        self.rounds = 0
        self.previous_quadrants = [0, 0]
        self.rounds_per_second = 0
        self.last_rounds_check_time = time.ticks_ms()
        self.distance_in_mm = 0

    def write_register(self, register, value):
        data = bytearray([register, value])
        self.i2c.writeto(AS5600_I2C_ADDR, data)

    def read_registers(self):
        try:
            msb = self.i2c.readfrom_mem(AS5600_I2C_ADDR, RAW_ANGLE_REGISTER_MSB, 1)[0]
            lsb = self.i2c.readfrom_mem(AS5600_I2C_ADDR, RAW_ANGLE_REGISTER_LSB, 1)[0]
            return (msb << 8) | lsb
        except OSError as e:
            print(f"OSError: {e}")
            dc_motor1.stop()
            dc_motor2.stop()
            return 0

    def get_status(self):
        status = self.read_registers()
        status_dict = {
            'magnet_too_strong': bool(status & 0x08),
            'magnet_too_weak': bool(status & 0x10),
            'magnet_detected': bool(status & 0x20)
        }

        print(f"Magnet zu stark: {status_dict['magnet_too_strong']}")
        print(f"Magnet zu schwach: {status_dict['magnet_too_weak']}")
        print(f"Magnet erkannt: {status_dict['magnet_detected']}")

        return status_dict

    def determine_direction(self):

        delta1 = self.positions[1] - self.positions[0]
        delta2 = self.positions[2] - self.positions[1]

        if delta1 > 0 and delta2 > 0:
            return 1  # Rechtslauf (clockwise)
        elif delta1 < 0 and delta2 < 0:
            return -1  # Linkslauf (counterclockwise)
        return 0  # Keine klare Richtung

    def get_quadrant(self):
        position = self.read_registers()
        if 0 <= position <= 1365:
            return 1
        elif 1365 < position <= 2730:
            return 2
        elif 2730 < position <= 4095:
            return 3
        else:
            return -1  # Invalid angle

    def count_rounds(self):
        current_position = self.read_registers()

        self.positions.pop(0)
        self.positions.append(current_position)

        direction = self.determine_direction()

        current_quadrant = self.get_quadrant()

        if current_quadrant == 1 and self.previous_quadrants[0] == 3 and self.previous_quadrants[1] == 2:
            self.rounds += 1
        elif current_quadrant == 3 and self.previous_quadrants[0] == 1 and self.previous_quadrants[1] == 2:
            self.rounds -= 1

        if current_quadrant != self.previous_quadrants[1]:
            self.previous_quadrants[0] = self.previous_quadrants[1]
            self.previous_quadrants[1] = current_quadrant

        r = 50
        scope = 2 * math.pi * r
        self.distance_in_mm = self.rounds * scope

        print(f"Direction: {direction}, Rounds: {self.rounds}, current_q: {current_quadrant}, previous_q: {self.previous_quadrants[0]}, pre-previous_q: {self.previous_quadrants[1]}")
        return self.distance_in_mm, self.rounds, current_quadrant, self.previous_quadrants

    def calculate_rounds_per_second(self):
        current_time = time.ticks_ms()
        elapsed_time = time.ticks_diff(current_time, self.last_rounds_check_time) / 1000.0  # Konvertiere zu Sekunden
        if elapsed_time > 0:
            self.rounds_per_second = self.rounds / elapsed_time
        self.last_rounds_check_time = current_time
        self.rounds = 0
        return self.rounds_per_second


class MotorController:
    def __init__(self, motor, sensor1):
        self.motor = motor
        self.sensor = sensor1
        self.kp = 1.0
        self.ki = 0.1
        self.kd = 0.01
        self.target_rpm = 0

        # PID-Parameter
        self.previous_error = 0
        self.integral = 0

    def set_target_rpm(self, rpm):
        self.target_rpm = rpm
        print(f"Target-RPM set on: {self.target_rpm}")

    def update(self):
        current_rpm = self.sensor.get_rpm()
        error = self.target_rpm - current_rpm
        self.integral += error
        derivative = error - self.previous_error

        adjustment = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        print(f"Error: {error}, Anpassung: {adjustment}")

        adjustment = max(-100, min(100, adjustment))
        print(f"Anpassung nach Begrenzung: {adjustment}")

        new_speed = self.motor.speed + int(adjustment)
        new_speed = max(0, min(1023, new_speed))
        print(f"new PWM-Speed: {new_speed}")

        self.motor.set_speed(new_speed if new_speed > 0 else 0)

        # Debug-Ausgaben
        print(f"PID-Regler: Fehler={error:.2f}, Integral={self.integral:.2f}, Derivative={derivative:.2f}, Anpassung={adjustment:.2f}")
        print(f"Neuer PWM-Speed: {new_speed}")
        self.previous_error = error

    def control_motor_with_rpm(self, target_rpm, duration=10):
        self.set_target_rpm(target_rpm)
        start_time = time.time()

        while time.time() - start_time < duration:
            self.sensor.update_rpm()
            self.update()
            print(f"Ziel-RPM: {self.target_rpm}, Aktuelle RPM: {self.sensor.get_rpm()}, PWM-Geschwindigkeit: {self.motor.speed}")
            time.sleep(0.1)
        self.motor.stop()
        print("Motor gestoppt.")

    def tune_pid(self, target_rpm, duration=5):
        kp_values = [0.5, 1.0, 1.5, 2.0]
        ki_values = [0.0, 0.1, 0.2, 0.5]
        kd_values = [0.0, 0.1, 0.2, 0.5]

        best_params = None
        best_performance = float('inf')

        for kp, ki, kd in product(kp_values, ki_values, kd_values):
            self.kp = kp
            self.ki = ki
            self.kd = kd
            print(f"Testing with kp={kp}, ki={ki}, kd={kd}")

            self.control_motor_with_rpm(target_rpm, duration)

            performance = sum((self.target_rpm - self.sensor.get_rpm())**2 for _ in range(duration * 10))
            print(f"Performance: {performance}")
            dc_motor1.stop()
            dc_motor2.stop()
            time.sleep(3)

            if performance < best_performance:
                best_performance = performance
                best_params = (kp, ki, kd)
                print(f"New best params: kp={kp}, ki={ki}, kd={kd} with performance={performance}")

        self.kp, self.ki, self.kd = best_params
        print(f"Best PID parameters: kp={self.kp}, ki={self.ki}, kd={self.kd}")


def product(*args):
    if not args:
        yield ()
        return
    for item in args[0]:
        for rest in product(*args[1:]):
            yield (item,) + rest


def calibrate_min_duty(motor, sensor):
    print("calibration minimal Duty-Cycles...")
    for duty in range(100, 1024, 20):
        motor.pin1.duty(0)
        motor.pin2.duty(duty)
        time.sleep(0.5)
        sensor.update_rpm()
        rpm = sensor.get_rpm()
        print(f"Duty: {duty}, RPM: {rpm}")
        dc_motor1.stop()
        dc_motor2.stop()
        time.sleep(3)
        if sensor.get_rpm() > 0:
            motor.stop()
            print(f"Minimal Duty-Cycle found: {duty} ###########################")
            time.sleep(3)
            return duty
    print("calibration error. Set min_duty to 750.")
    return 750


if __name__ == "__main__":

    sda1 = Pin(14)
    scl1 = Pin(13)
    i2c1 = I2C(0, sda=sda1, scl=scl1, freq=400000)

    sda2 = Pin(10)
    scl2 = Pin(8)
    i2c2 = I2C(0, sda=sda2, scl=scl2, freq=400000)

    led = neopixel.NeoPixel(Pin(36), 1)
    led.fill((50, 50, 50))
    led.write()

    sensor1 = AS5600(i2c1)
    sensor1.get_status()
    print("blaaa")
    sensor1.read_registers()

    # sensor2 = AS5600(i2c2)

    # initalize dc motor
    frequency1 = 15000
    pin11 = 38
    pin21 = 36
    dc_motor1 = DCMotor(pin11, pin21, frequency1)

    frequency2 = 15000
    pin12 = 38
    pin22 = 36
    dc_motor2 = DCMotor(pin12, pin22, frequency2)

    try:
        while True:
            sensor1.count_rounds()
    except KeyboardInterrupt:
        print("Program interrupted by user")

    # calibration of minimal Duty-Cycles
    # min_duty1 = calibrate_min_duty(dc_motor1, sensor1)
    # min_duty2 = calibrate_min_duty(dc_motor2, sensor2)
    # dc_motor1.min_duty = min_duty1
    # dc_motor2.min_duty = min_duty2

    # controller = MotorController(dc_motor1, sensor1)

    # try:
    #     controller.tune_pid(target_rpm=90, duration=10)
    #     controller.control_motor_with_rpm(target_rpm=90, duration=10)
    # except KeyboardInterrupt:
    #     dc_motor1.stop()
    #     print("Program ending and motor stopped.")
