from robot import Robot
from component import Motor, ServoMotor, Ultrasonic
from typing import List
import time

class Forward(Robot):
    def __init__(
        self,
        left_motors_ids: List[str],
        right_motors_ids: List[str],
        servo_id: str,
        ultrasonic_id: str
    ):
        """Initialize the Forward movement with left and right motors."""
        super().__init__(left_motors_ids + right_motors_ids, servo_id, ultrasonic_id)
        self.__left_motors = self._motors[:len(left_motors_ids)]
        self.__right_motors = self._motors[len(left_motors_ids):]

    @property
    def left_motors(self) -> List[Motor]:
        """Get the left motors."""
        return self.__left_motors

    @left_motors.setter
    def left_motors(self, motors: List[Motor]):
        """Set the left motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Left motors must be a non-empty list of Motor instances")
        self.__left_motors = motors

    @property
    def right_motors(self) -> List[Motor]:
        """Get the right motors."""
        return self.__right_motors

    @right_motors.setter
    def right_motors(self, motors: List[Motor]):
        """Set the right motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Right motors must be a non-empty list of Motor instances")
        self.__right_motors = motors

    @property
    def servo(self) -> ServoMotor:
        """Get the servo motor."""
        return self._servo

    @property
    def ultrasonic(self) -> Ultrasonic:
        """Get the ultrasonic sensor."""
        return self._ultrasonic

    def move(self, speed: int = 512):
        """Move forward in a straight line."""
        if not 0 <= speed <= 1023:
            raise ValueError("Speed must be between 0 and 1023")
        for motor in self.__left_motors + self.__right_motors:
            motor.move_forward(speed)

class Backward(Robot):
    def __init__(
        self,
        left_motors_ids: List[str],
        right_motors_ids: List[str],
        servo_id: str,
        ultrasonic_id: str
    ):
        """Initialize the Backward movement with left and right motors."""
        super().__init__(left_motors_ids + right_motors_ids, servo_id, ultrasonic_id)
        self.__left_motors = self._motors[:len(left_motors_ids)]
        self.__right_motors = self._motors[len(left_motors_ids):]

    @property
    def left_motors(self) -> List[Motor]:
        """Get the left motors."""
        return self.__left_motors

    @left_motors.setter
    def left_motors(self, motors: List[Motor]):
        """Set the left motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Left motors must be a non-empty list of Motor instances")
        self.__left_motors = motors

    @property
    def right_motors(self) -> List[Motor]:
        """Get the right motors."""
        return self.__right_motors

    @right_motors.setter
    def right_motors(self, motors: List[Motor]):
        """Set the right motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Right motors must be a non-empty list of Motor instances")
        self.__right_motors = motors

    @property
    def servo(self) -> ServoMotor:
        """Get the servo motor."""
        return self._servo

    @property
    def ultrasonic(self) -> Ultrasonic:
        """Get the ultrasonic sensor."""
        return self._ultrasonic

    def move(self, speed: int = 512):
        """Move backward in a straight line."""
        if not 0 <= speed <= 1023:
            raise ValueError("Speed must be between 0 and 1023")
        for motor in self.__left_motors + self.__right_motors:
            motor.move_backward(speed)

class TurnLeft(Robot):
    def __init__(
        self,
        left_motors_ids: List[str],
        right_motors_ids: List[str],
        servo_id: str,
        ultrasonic_id: str
    ):
        """Initialize the TurnLeft movement with left and right motors."""
        super().__init__(left_motors_ids + right_motors_ids, servo_id, ultrasonic_id)
        self.__left_motors = self._motors[:len(left_motors_ids)]
        self.__right_motors = self._motors[len(left_motors_ids):]

    @property
    def left_motors(self) -> List[Motor]:
        """Get the left motors."""
        return self.__left_motors

    @left_motors.setter
    def left_motors(self, motors: List[Motor]):
        """Set the left motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Left motors must be a non-empty list of Motor instances")
        self.__left_motors = motors

    @property
    def right_motors(self) -> List[Motor]:
        """Get the right motors."""
        return self.__right_motors

    @right_motors.setter
    def right_motors(self, motors: List[Motor]):
        """Set the right motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Right motors must be a non-empty list of Motor instances")
        self.__right_motors = motors

    @property
    def servo(self) -> ServoMotor:
        """Get the servo motor."""
        return self._servo

    @property
    def ultrasonic(self) -> Ultrasonic:
        """Get the ultrasonic sensor."""
        return self._ultrasonic

    def move(self, speed: int = 512):
        """Turn left."""
        if not 0 <= speed <= 1023:
            raise ValueError("Speed must be between 0 and 1023")
        for motor in self.__left_motors:
            motor.move_forward(speed // 3)
        for motor in self.__right_motors:
            motor.move_forward(speed)

class TurnRight(Robot):
    def __init__(
        self,
        left_motors_ids: List[str],
        right_motors_ids: List[str],
        servo_id: str,
        ultrasonic_id: str
    ):
        """Initialize the TurnRight movement with left and right motors."""
        super().__init__(left_motors_ids + right_motors_ids, servo_id, ultrasonic_id)
        self.__left_motors = self._motors[:len(left_motors_ids)]
        self.__right_motors = self._motors[len(left_motors_ids):]

    @property
    def left_motors(self) -> List[Motor]:
        """Get the left motors."""
        return self.__left_motors

    @left_motors.setter
    def left_motors(self, motors: List[Motor]):
        """Set the left motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Left motors must be a non-empty list of Motor instances")
        self.__left_motors = motors

    @property
    def right_motors(self) -> List[Motor]:
        """Get the right motors."""
        return self.__right_motors

    @right_motors.setter
    def right_motors(self, motors: List[Motor]):
        """Set the right motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Right motors must be a non-empty list of Motor instances")
        self.__right_motors = motors

    @property
    def servo(self) -> ServoMotor:
        """Get the servo motor."""
        return self._servo

    @property
    def ultrasonic(self) -> Ultrasonic:
        """Get the ultrasonic sensor."""
        return self._ultrasonic

    def move(self, speed: int = 512):
        """Turn right."""
        if not 0 <= speed <= 1023:
            raise ValueError("Speed must be between 0 and 1023")
        for motor in self.__left_motors:
            motor.move_forward(speed)
        for motor in self.__right_motors:
            motor.move_forward(speed // 3)

class Stop(Robot):
    def __init__(
        self,
        left_motors_ids: List[str],
        right_motors_ids: List[str],
        servo_id: str,
        ultrasonic_id: str
    ):
        """Initialize the Stop movement with left and right motors."""
        super().__init__(left_motors_ids + right_motors_ids, servo_id, ultrasonic_id)
        self.__left_motors = self._motors[:len(left_motors_ids)]
        self.__right_motors = self._motors[len(left_motors_ids):]

    @property
    def left_motors(self) -> List[Motor]:
        """Get the left motors."""
        return self.__left_motors

    @left_motors.setter
    def left_motors(self, motors: List[Motor]):
        """Set the left motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Left motors must be a non-empty list of Motor instances")
        self.__left_motors = motors

    @property
    def right_motors(self) -> List[Motor]:
        """Get the right motors."""
        return self.__right_motors

    @right_motors.setter
    def right_motors(self, motors: List[Motor]):
        """Set the right motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Right motors must be a non-empty list of Motor instances")
        self.__right_motors = motors

    @property
    def servo(self) -> ServoMotor:
        """Get the servo motor."""
        return self._servo

    @property
    def ultrasonic(self) -> Ultrasonic:
        """Get the ultrasonic sensor."""
        return self._ultrasonic

    def move(self, speed: int = 0):
        """Stop the robot."""
        for motor in self._motors:
            motor.stop()

class ObstacleAvoidance(Robot):
    def __init__(
        self,
        left_motors_ids: List[str],
        right_motors_ids: List[str],
        servo_id: str,
        ultrasonic_id: str
    ):
        """Initialize the ObstacleAvoidance movement with left and right motors."""
        super().__init__(left_motors_ids + right_motors_ids, servo_id, ultrasonic_id)
        self.__left_motors = self._motors[:len(left_motors_ids)]
        self.__right_motors = self._motors[len(left_motors_ids):]

    @property
    def left_motors(self) -> List[Motor]:
        """Get the left motors."""
        return self.__left_motors

    @left_motors.setter
    def left_motors(self, motors: List[Motor]):
        """Set the left motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Left motors must be a non-empty list of Motor instances")
        self.__left_motors = motors

    @property
    def right_motors(self) -> List[Motor]:
        """Get the right motors."""
        return self.__right_motors

    @right_motors.setter
    def right_motors(self, motors: List[Motor]):
        """Set the right motors."""
        if not motors or not all(isinstance(m, Motor) for m in motors):
            raise ValueError("Right motors must be a non-empty list of Motor instances")
        self.__right_motors = motors

    @property
    def servo(self) -> ServoMotor:
        """Get the servo motor."""
        return self._servo

    @property
    def ultrasonic(self) -> Ultrasonic:
        """Get the ultrasonic sensor."""
        return self._ultrasonic

    def avoid_obstacles(self, speed: int = 512, threshold_distance: float = 20.0):
        """Implement obstacle avoidance logic."""
        while True:
            try:
                distance = self.ultrasonic.distance()
                print(f"Measured distance: {distance:.2f} cm")
                if distance < threshold_distance:
                    print("Obstacle detected!")
                    for motor in self._motors:
                        motor.stop()
                    self.servo.write_angle(90)
                    time.sleep(0.5)
                    left_distance = self.ultrasonic.distance()
                    print(f"Left distance: {left_distance:.2f} cm")
                    self.servo.write_angle(-90)
                    time.sleep(0.5)
                    right_distance = self.ultrasonic.distance()
                    print(f"Right distance: {right_distance:.2f} cm")
                    self.servo.write_angle(0)
                    time.sleep(0.5)
                    if left_distance > right_distance and left_distance > threshold_distance:
                        print("Turning left")
                        for motor in self.__left_motors:
                            motor.move_forward(speed // 3)
                        for motor in self.__right_motors:
                            motor.move_forward(speed)
                        time.sleep(1)
                    elif right_distance > threshold_distance:
                        print("Turning right")
                        for motor in self.__left_motors:
                            motor.move_forward(speed)
                        for motor in self.__right_motors:
                            motor.move_forward(speed // 3)
                        time.sleep(1)
                    else:
                        print("No clear path, stopping")
                        for motor in self._motors:
                            motor.stop()
                        break
                else:
                    print("Path clear, moving forward")
                    for motor in self.__left_motors + self.__right_motors:
                        motor.move_forward(speed)
                time.sleep(0.1)
            except Exception as e:
                print(f"Error: {e}")
                for motor in self._motors:
                    motor.stop()
                break

    def move(self, speed: int = 512):
        """Start obstacle avoidance."""
        self.avoid_obstacles(speed, threshold_distance=20.0)