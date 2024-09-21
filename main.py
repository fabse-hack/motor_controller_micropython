from machine import I2C, Pin, PWM
import time

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
    def __init__(self, pin1, pin2, enablea, frequency=15000, min_duty=750, max_duty=1023):
        self.pin1 = Pin(pin1, Pin.OUT)
        self.pin2 = Pin(pin2, Pin.OUT)
        self.enablea = PWM(Pin(enablea), freq=frequency)
        self.min_duty = min_duty
        self.max_duty = max_duty
        self.speed = 0

    def duty_cycle(self, speed):
        speed = max(self.min_duty, min(self.max_duty, speed))
        return speed

    def set_speed(self, speed: int):
        if speed > 0:
            # Rechtslauf
            self.speed = self.duty_cycle(speed)
            self.enablea.duty(self.speed)
            self.pin1.value(0)
            self.pin2.value(1)
        elif speed < 0:
            # Linkslauf
            self.speed = self.duty_cycle(-speed)
            self.enablea.duty(self.speed)
            self.pin1.value(1)
            self.pin2.value(0)
        else:
            self.stop()

    def stop(self):
        self.enablea.duty(0)
        self.pin1.value(0)
        self.pin2.value(0)
        self.speed = 0


class AS5600:
    def __init__(self, i2c):
        self.i2c = i2c
        self.last_angle = self.read_position()
        self.rotation_count = 0
        self.start_time = time.time()
        self.rpm = 0

    def read_register(self, register):
        return self.i2c.readfrom_mem(AS5600_I2C_ADDR, register, 1)[0]

    def read_registers(self, reg_msb, reg_lsb):
        msb = self.i2c.readfrom_mem(AS5600_I2C_ADDR, reg_msb, 1)[0]
        lsb = self.i2c.readfrom_mem(AS5600_I2C_ADDR, reg_lsb, 1)[0]
        return (msb << 8) | lsb

    def write_register(self, register, value):
        self.i2c.writeto_mem(AS5600_I2C_ADDR, register, bytearray([value]))

    def read_position_raw(self):
        return self.read_registers(RAW_ANGLE_REGISTER_MSB, RAW_ANGLE_REGISTER_LSB)

    def initialize(self, settings=None):
        if settings:
            self.set_power_mode(settings.get('power_mode', 0))
            self.set_output_stage(settings.get('output_stage', 0))
        print("Sensor initialisiert mit benutzerdefinierten Einstellungen")

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

        # Überprüfe, ob eine vollständige Umdrehung erfolgt ist
        if self.last_angle > 300 and current_angle < 60:
            self.rotation_count += 1
        elif self.last_angle < 60 and current_angle > 300:
            self.rotation_count -= 1

        self.last_angle = current_angle

        elapsed_time = time.time() - self.start_time
        if elapsed_time >= 1:
            self.rpm = (self.rotation_count / elapsed_time) * 60
            self.rotation_count = 0
            self.start_time = time.time()

    def get_rpm(self):
        return self.rpm


class MotorController:
    def __init__(self, motor, sensor, kp=1.0):
        self.motor = motor
        self.sensor = sensor
        self.kp = kp
        self.target_rpm = 0

    def set_target_rpm(self, rpm):
        self.target_rpm = rpm

    def update(self):
        current_rpm = self.sensor.get_rpm()
        error = self.target_rpm - current_rpm
        adjustment = self.kp * error

        adjustment = max(-100, min(100, adjustment))

        new_speed = max(0, min(1023, self.motor.speed + adjustment))

        self.motor.set_speed(new_speed if new_speed > 0 else 0)


def control_motor_with_rpm(controller, target_rpm, duration=10):
    controller.set_target_rpm(target_rpm)
    start_time = time.time()

    while time.time() - start_time < duration:
        controller.sensor.update_rpm()
        controller.update()
        print(f"Ziel-RPM: {controller.target_rpm}, Aktuelle RPM: {controller.sensor.get_rpm()}, PWM-Speed: {controller.motor.speed}")
        time.sleep(0.1)
    controller.motor.stop()
    print("Motor gestoppt.")


def calibrate_min_duty(motor, sensor):
    print("Kalibrierung des minimalen Duty-Cycles...")
    for duty in range(500, 1024, 10):
        motor.enablea.duty(duty)
        motor.pin1.value(0)
        motor.pin2.value(1)
        time.sleep(0.5)
        sensor.update_rpm()
        if sensor.get_rpm() > 0:
            motor.stop()
            print(f"Minimaler Duty-Cycle gefunden: {duty}")
            return duty
    print("Kalibrierung fehlgeschlagen.")
    return 750  # Standardwert, falls Kalibrierung fehlschlägt


if __name__ == "__main__":
    sda = Pin(5)
    scl = Pin(3)
    i2c = I2C(0, sda=sda, scl=scl, freq=400000)

    sensor = AS5600(i2c)
    sensor.initialize()

    # Initialisierung des Motors
    frequency = 15000
    pin1 = 39
    pin2 = 38
    enablea_pin = 47
    dc_motor = DCMotor(pin1, pin2, enablea_pin, frequency)

    # Kalibrierung des minimalen Duty-Cycles
    min_duty = calibrate_min_duty(dc_motor, sensor)
    dc_motor.min_duty = min_duty

    controller = MotorController(dc_motor, sensor, kp=2.0)  # kp kann angepasst werden

    try:
        # Setze den Motor auf 50 RPM für 10 Sekunden
        control_motor_with_rpm(controller, target_rpm=50, duration=10)
    except KeyboardInterrupt:
        dc_motor.stop()
        print("Programm beendet und Motor gestoppt.")
