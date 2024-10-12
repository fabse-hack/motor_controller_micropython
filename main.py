from machine import I2C, Pin, PWM, SPI
import time
import sys
import neopixel
import _thread


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
        self.last_angle = self.read_position()
        self.last_time = time.ticks_us()  # microseconds since start
        self.rpm = 0
        self.angle_history = []
        self.time_history = []
        self.max_history = 5  # counting of measurements for Mittelwertbildung
        self.angle_sum = 0  # Summe der Winkeländerungen
        self.time_sum = 0  # Summe der Zeitänderungen

    def read_register(self, register):
        return self.i2c.readfrom_mem(AS5600_I2C_ADDR, register, 1)[0]

    def read_registers(self, reg_msb, reg_lsb):
        try:
            msb = self.i2c.readfrom_mem(AS5600_I2C_ADDR, reg_msb, 1)[0]
            lsb = self.i2c.readfrom_mem(AS5600_I2C_ADDR, reg_lsb, 1)[0]
            return (msb << 8) | lsb
        except OSError as e:
            print(f"OSError: {e}")
            dc_motor1.stop()
            dc_motor2.stop()
            return 0

    def write_register(self, register, value):
        self.i2c.writeto_mem(AS5600_I2C_ADDR, register, bytearray([value]))

    def read_position_raw(self):
        return self.read_registers(RAW_ANGLE_REGISTER_MSB, RAW_ANGLE_REGISTER_LSB)

    def initialize(self, settings=None):
        if settings:
            self.set_power_mode(settings.get('power_mode', 0))
            self.set_output_stage(settings.get('output_stage', 0))
        print("Sensor initialize with custom settings")

    def read_position(self):
        raw_angle = self.read_scaled_angle()
        degrees = (raw_angle / 4096) * 360
        return degrees

    def read_scaled_angle(self):
        return self.read_registers(ANGLE_REGISTER_MSB, ANGLE_REGISTER_LSB)

    def get_status(self):
        status = self.read_register(STATUS_REGISTER)
        return {
            'magnet_too_strong': bool(status & 0x08),
            'magnet_too_weak': bool(status & 0x10),
            'magnet_detected': bool(status & 0x20)
        }

    def get_magnitude(self):
        return self.read_registers(MAGNITUDE_REGISTER_MSB, MAGNITUDE_REGISTER_LSB)

    def update_rpm(self):
        current_angle = self.read_position()
        current_time = time.ticks_us()

        # calculate time differents in Seconds
        delta_time = time.ticks_diff(current_time, self.last_time) / 1_000_000

        # Berechne Winkeländerung
        delta_angle = current_angle - self.last_angle

        # Handle Wrap-around
        if delta_angle > 180:
            delta_angle -= 360
        elif delta_angle < -180:
            delta_angle += 360

        # Berechne RPM
        if delta_time > 0:
            angular_velocity = delta_angle / delta_time  # Winkelgeschwindigkeit in Grad pro Sekunde
            self.rpm = (angular_velocity / 360) * 60  # Umdrehungen pro Minute (RPM)

            self.angle_history.append(delta_angle)
            self.time_history.append(delta_time)
            self.angle_sum += delta_angle
            self.time_sum += delta_time

            if len(self.angle_history) > self.max_history:
                self.angle_sum -= self.angle_history.pop(0)
                self.time_sum -= self.time_history.pop(0)

            # Berechne den gleitenden Durchschnitt
            if self.time_sum > 0:
                self.rpm = (self.angle_sum / self.time_sum) / 360 * 60

        self.last_angle = current_angle
        self.last_time = current_time

        print(f"Delta Winkel: {delta_angle:.2f}°, Delta Zeit: {delta_time:.4f}s, RPM: {self.rpm:.2f}")

    def get_rpm(self):
        return self.rpm


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
    print("booting")
    time.sleep(3)
    print("now starting....")

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
    sensor1.initialize()

    sensor2 = AS5600(i2c2)
    sensor2.initialize()

    # initalize dc motor
    frequency1 = 15000
    pin11 = 38
    pin21 = 36
    dc_motor1 = DCMotor(pin11, pin21, frequency1)

    frequency2 = 15000
    pin12 = 38
    pin22 = 36
    dc_motor2 = DCMotor(pin12, pin22, frequency2)

    # calibration of minimal Duty-Cycles
    min_duty1 = calibrate_min_duty(dc_motor1, sensor1)
    min_duty2 = calibrate_min_duty(dc_motor2, sensor2)
    dc_motor1.min_duty = min_duty1
    dc_motor2.min_duty = min_duty2

    controller = MotorController(dc_motor1, sensor1)

    try:
        controller.tune_pid(target_rpm=90, duration=10)
        controller.control_motor_with_rpm(target_rpm=90, duration=10)
    except KeyboardInterrupt:
        dc_motor1.stop()
        print("Program ending and motor stopped.")
