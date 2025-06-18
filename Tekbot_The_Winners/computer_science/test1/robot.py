from abc import ABC, abstractmethod
from typing import List
from component import Motor, ServoMotor, Ultrasonic

class Robot(ABC):
    def __init__(
        self,
        motors_ids: List[str],
        servo_id: str,
        ultrasonic_id: str
    ):
        """Initialize the robot with motors, servo, and ultrasonic sensor."""
        self._motors = [Motor(id) for id in motors_ids]
        self._servo = ServoMotor(servo_id)
        self._ultrasonic = Ultrasonic(ultrasonic_id)

    @property
    def servo(self) -> ServoMotor:
        """Get the servo motor."""
        return self._servo

    @servo.setter
    def servo(self, servo: ServoMotor):
        """Set the servo motor."""
        if not isinstance(servo, ServoMotor):
            raise ValueError("Servo must be an instance of ServoMotor")
        self._servo = servo

    @property
    def ultrasonic(self) -> Ultrasonic:
        """Get the ultrasonic sensor."""
        return self._ultrasonic

    @ultrasonic.setter
    def ultrasonic(self, ultrasonic: Ultrasonic):
        """Set the ultrasonic sensor."""
        if not isinstance(ultrasonic, Ultrasonic):
            raise ValueError("Ultrasonic must be an instance of Ultrasonic")
        self._ultrasonic = ultrasonic

    @abstractmethod
    def move(self, speed: int = 0):
        """Abstract method to move the robot."""
        pass