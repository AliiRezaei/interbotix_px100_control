from interbotix_xs_modules.arm import InterbotixManipulatorXS
from time import sleep

def main():
    bot = InterbotixManipulatorXS(robot_model = "px100",
                                 robot_name   = "px100", 
                                 group_name   = "arm",
                                 gripper_name = "gripper")
    
    bot.arm.go_to_home_pose()
    sleep(5)
    bot.arm.go_to_sleep_pose()

if __name__ == "__main__":
    main()