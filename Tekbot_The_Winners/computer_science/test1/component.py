from typing import Optional
import random

class Motor:
    def __init__(self, identifier: str):
        self._identifier = identifier
        self._speed = 0
        self._direction = "stop"

    def move_forward(self, speed: int):
        """Move the motor forward with the specified speed."""
        if not 0 <= speed <= 1023:
            raise ValueError("Speed must be between 0 and 1023")
        self._speed = speed
        self._direction = "forward"
        print(f"[SIMULATION MOTOR {self._identifier}] Moving forward with speed {speed}")

    def move_backward(self, speed: int):
        """Move the motor backward with the specified speed."""
        if not 0 <= speed <= 1023:
            raise ValueError("Speed must be between 0 and 1023")
        self._speed = speed
        self._direction = "backward"
        print(f"[SIMULATION MOTOR {self._identifier}] Moving backward with speed {speed}")

    def stop(self):
        """Stop the motor."""
        self._speed = 0
        self._direction = "stop"
        print(f"[SIMULATION MOTOR {self._identifier}] Stopped")

    @property
    def speed(self) -> int:
        """Get the current speed of the motor."""
        return self._speed

    @speed.setter
    def speed(self, speed: int):
        """Set the speed of the motor."""
        if not 0 <= speed <= 1023:
            raise ValueError("Speed must be between 0 and 1023")
        self._speed = speed

    @property
    def direction(self) -> str:
        """Get the current direction of the motor."""
        return self._direction

    @direction.setter
    def direction(self, value: str) -> None:
        """Set the direction of the motor."""
        valid_directions = ["forward", "backward", "stop"]
        if value not in valid_directions:
            raise ValueError(f"Direction must be one of {valid_directions}")
        self._direction = value

class ServoMotor:
    def __init__(self, identifier: str):
        self._identifier = identifier
        self._angle = 0

    def write_angle(self, degrees: Optional[float] = None, radians: Optional[float] = None):
        """Set the servo motor angle."""
        if degrees is None and radians is None:
            raise ValueError("An angle in degrees must be provided")
        if degrees is None:
            import math
            degrees = math.degrees(radians)
        self._angle = degrees % 360
        print(f"[SIMULATION SERVO {self._identifier}] Angle set to {self._angle} degrees")

    def stop(self):
        """Stop the servo motor."""
        print(f"[SIMULATION SERVO {self._identifier}] Stopped")

    @property
    def angle(self) -> float:
        """Get the current angle of the servo motor."""
        return self._angle

    @angle.setter
    def angle(self, degrees: float):
        """Set the angle of the motor."""
        self._angle = degrees % 360

class Ultrasonic:
    def __init__(self, identifier: str):
        self._identifier = identifier

    def distance(self) -> float:
        """Measure the simulated distance."""
        distance = random.uniform(5, 100)
        print(f"[SIMULATION ULTRASONIC {self._identifier}] Measured distance: {distance:.2f} cm")
        return distance