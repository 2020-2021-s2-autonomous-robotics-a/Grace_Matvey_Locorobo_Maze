from locorobo import LocoRobo
from locorobo import MotorDirection
from locorobo import Data
from locorobo import WaitType
from locorobo import Song
from locorobo import Note

def get_robot(robots, name):
    robot = None

    # Search through robots found during the scan for
    # the one we want
    for r in robots.values():
        if r.name == name:
            robot = r

            # We found the robot, so stop the for loop
            break

    # If we did not find the robot during the scan, stop the program
    if not robot:
        raise Exception('Could not find robot with specified name')

    return robot


def main():
    # Tell LocoRobo what serial port to use
    LocoRobo.setup("/dev/tty.usbmodem1")
    
    # Scan for robots
    robots = LocoRobo.scan(2000)

    # Use get_robots to find robot with name lr 00:07 in the scan result
    robot = get_robot(robots, "Nimbus")

    robot.connect()
    robot.activate_motors()
    robot.enable_sensor(Data.ULTRASONIC, True)
    LocoRobo.wait(.2)
    
    maze_half_width = 30
    
    left, front, right = read(robot)
    print(left, front, right)
    

    while left < 100 or right < 100 or front < 100:
        robot.setup_wait(WaitType.DISTANCE, maze_half_width * 1000)
        robot.move(MotorDirection.FORWARD, MotorDirection.FORWARD, 1, 1, True)
        left, front, right = read(robot)
        print(left, front, right)
        if not (left < 100 or right < 100 or front < 100):
            break
        
        if left < 100 and left > maze_half_width or left > 100:
            turn_left(robot)
        elif front < 100 and front < maze_half_width:
            if right < 100 and right < maze_half_width:
                turn_right(robot, degrees = 180)
            else:
                turn_right(robot)
    

    robot.deactivate_motors()
    robot.disconnect()

def turn_right(robot, speed=1, degrees=91):
    """Turn robot right by degrees at speed."""
    robot.setup_wait(WaitType.ROTATION, degrees * 1000)
    robot.move(MotorDirection.FORWARD, MotorDirection.BACKWARD, speed, speed, True)

def turn_left(robot, speed=1, degrees=90):
    """Turn robot right by degrees at speed."""
    robot.setup_wait(WaitType.ROTATION, degrees * 1000)
    robot.move(MotorDirection.BACKWARD, MotorDirection.FORWARD, speed, speed, True)

def read(robot):
    """Return left, front, right distances."""
    speed = 3 / 4

    robot.enable_sensor(Data.ULTRASONIC, True)
    front = robot.get_sensor_value(Data.ULTRASONIC)

    turn_right(robot, speed)
    right = robot.get_sensor_value(Data.ULTRASONIC)

    turn_left(robot, speed, 180)
    left = robot.get_sensor_value(Data.ULTRASONIC)

    turn_right(robot, speed)

    return left, front, right

# If we are on the main thread, run the program
if __name__ == "__main__":

    try:
        main()
    except:
        LocoRobo.stop()
        raise

    LocoRobo.stop()

    # For compatibility with webapp's python, we can't use finally.
    # If you are using local python, you can do the following
    #
    # try:
    #     main()
    # finally:
    #     LocoRobo.stop()
