import numpy as np
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Twist, Vector3

# Global variables
red_center = Point()
red_flag = False
red_base = Point()
blue_base = Point()
red_twist = Twist()
game_over = False
accumulated_error = 0.
neutral_zone = False

# Helper functions
def set_center(sphere_center):
    global red_center
    red_center = sphere_center
    return

def set_flag(flag_status):
    global red_flag, neutral_zone
    # Logic for needing to go back through neutral zone
    if red_flag != flag_status.data:
        neutral_zone = False
    red_flag = flag_status.data
    return

def set_game_over(game_state):
    global game_over
    game_over = game_state.data
    return

def set_red_base(base):
    global red_base
    red_base = base
    return

def set_blue_base(base):
    global blue_base
    blue_base = base
    return

def yaw_vel_to_twist(yaw, vel): 
    twist_msg = Twist() 
    twist_msg.linear = Vector3(0, 0, 0) 
    twist_msg.angular.x = np.cos(yaw) * vel 
    twist_msg.angular.y = np.sin(yaw) * vel 
    twist_msg.angular.z = 0 
    return twist_msg
stuckFile = open('stuckLog.txt','w')
positionHistory = []
def stuck(redBase):
    global positionHistory
    stuckFile.write("Stuck function running: positionHistory = %s\n" % str(positionHistory))
    if len(positionHistory) < 2:
        positionHistory.append((redBase.x,redBase.y))
        return False
    else:
        change = sum([(x-y)**2 for x,y in zip((redBase.x,redBase.y),positionHistory[0])])**0.5
        stuckFile.write("Change = %.2f\n" % change)
        if change < 5.:
            stuckFile.write("Stuck!\n")
            return True
        oldestTime = positionHistory[-1]
        nowTime = (redBase.x,redBase.y)
        positionHistory = [oldestTime,nowTime]
        #positionHistory = list(positionHistory[-1]).append((redBase.x,redBase.y))

def get_heading_and_distance():
    global red_center, red_flag, red_base, blue_base, neutral_zone
    if stuck(red_center):
        target_x = (0.35 * (max(red_base.x, blue_base.x) 
                            - min(red_base.x, blue_base.x)) 
                            + min(red_base.x, blue_base.x))
        target_y = (0.35 * (max(red_base.y, blue_base.y) 
                            - min(red_base.y, blue_base.y)) 
                            + min(red_base.y, blue_base.y))
    else:
        if neutral_zone and red_flag:
            # Have flag, go home
            target_x = red_base.x
            target_y = red_base.y
        elif not red_flag and (neutral_zone != False):
            # Don't have flag, go to opponent's base
            target_x = blue_base.x
            target_y = blue_base.y
        else:
            if red_flag:
                fy = 0.5
                fx = 0.25
            else:
                fy = 0.25
                fx = 0.5
            # Haven't passed through neutral zone, go there
            target_x = (fx * (max(red_base.x, blue_base.x) 
                            - min(red_base.x, blue_base.x)) 
                            + min(red_base.x, blue_base.x))
            target_y = (fy * (max(red_base.y, blue_base.y) 
                            - min(red_base.y, blue_base.y)) 
                            + min(red_base.y, blue_base.y))
    delta_x = target_x - red_center.x
    delta_y = target_y - red_center.y
    print("[{}, {}]".format(delta_x, delta_y))
    distance = np.sqrt(delta_x ** 2 + delta_y ** 2)
    if not neutral_zone and distance < 100:
        neutral_zone = True
    heading = np.arctan2(delta_y, delta_x)
    return heading, distance

# Agent function
def proportional_control():
    global red_twist, accumulated_error

    if red_center != Point():
        heading, distance = get_heading_and_distance()
        heading = -heading # Switch from camera to world coordinates
        if distance < 100:
            accumulated_error = 0
        else:
            accumulated_error += distance
        speed = max(distance*0.04 + accumulated_error / 10000.,5.)
    else:
        speed = 0
        heading = 0
    red_twist = yaw_vel_to_twist(heading, speed)
    return

# Init function
def simple_agent():
    global game_over
    # Setup ROS message handling
    rospy.init_node('red_agent', anonymous=True)

    pub_red_cmd = rospy.Publisher('/red_sphero/twist_cmd', Twist, queue_size=1)
    sub_red_center = rospy.Subscriber('/red_sphero/center', Point, set_center, queue_size=1)
    sub_red_flag = rospy.Subscriber('/red_sphero/flag', Bool, set_flag, queue_size=1)
    sub_red_base = rospy.Subscriber('/red_sphero/base', Point, set_red_base, queue_size=1)
    sub_blue_base = rospy.Subscriber('/blue_sphero/base', Point, set_blue_base, queue_size=1)
    sub_game_over = rospy.Subscriber('/game_over', Bool, set_game_over, queue_size=1)

    # Agent control loop
    rate = rospy.Rate(2) # Hz
    while not rospy.is_shutdown():
        proportional_control()
        pub_red_cmd.publish(red_twist)
        if game_over != False:
            break
        rate.sleep()
    print("Game ended. No agent to save.")
    return

if __name__ == '__main__':
    try:
        simple_agent()
    except rospy.ROSInterruptException:
        pass

