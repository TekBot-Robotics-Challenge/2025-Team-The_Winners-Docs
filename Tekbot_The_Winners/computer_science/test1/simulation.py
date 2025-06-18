import time
from action import Forward, Backward, TurnLeft, TurnRight, Stop, ObstacleAvoidance
from typing import List

def main():
    """Run simulation for all robot movements."""
    left_motors_ids = ["motor_g1", "motor_g2"]
    right_motors_ids = ["motor_d1", "motor_d2"]
    servo_id = "servo_1"
    ultrasonic_id = "ultrasonic_1"

    print("=== Testing Forward ===")
    robot_forward = Forward(left_motors_ids, right_motors_ids, servo_id, ultrasonic_id)
    robot_forward.move(speed=512)
    time.sleep(1)

    print("\n=== Testing Backward ===")
    robot_backward = Backward(left_motors_ids, right_motors_ids, servo_id, ultrasonic_id)
    robot_backward.move(speed=512)
    time.sleep(1)

    print("\n=== Testing TurnLeft ===")
    robot_left = TurnLeft(left_motors_ids, right_motors_ids, servo_id, ultrasonic_id)
    robot_left.move(speed=512)
    time.sleep(1)

    print("\n=== Testing TurnRight ===")
    robot_right = TurnRight(left_motors_ids, right_motors_ids, servo_id, ultrasonic_id)
    robot_right.move(speed=512)
    time.sleep(1)

    print("\n=== Testing Stop ===")
    robot_stop = Stop(left_motors_ids, right_motors_ids, servo_id, ultrasonic_id)
    robot_stop.move()

    print("\n=== Testing ObstacleAvoidance ===")
    robot_obstacle = ObstacleAvoidance(left_motors_ids, right_motors_ids, servo_id, ultrasonic_id)
    robot_obstacle.move(speed=512)
    time.sleep(5)

if __name__ == "__main__":
    main()