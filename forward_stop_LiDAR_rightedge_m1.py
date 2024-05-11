import rclpy
import time
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

near, med, far = [0, 0, 0.2, 0.4], [0.2, 0.4, 0.4, 0.6], [0.4, 0.6, 0.8, 0.8]
dis = [near, med, far]
shape_types = ['Left Shoulder', 'Triangle', 'Right Shoulder']

#from fuzzy_right import fuzzymain
mynode_ = None
pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
    'bright': 0,
}
twstmsg_ = None

# main function attached to timer callback
def timer_callback():
    global pub_, twstmsg_
    if ( twstmsg_ != None ):
        pub_.publish(twstmsg_)

def clbk_laser(msg):
    global regions_, twstmsg_   
    regions_ = {         
        #LIDAR readings are anti-clockwise
         
        'fleft':  find_nearest (msg.ranges[130:150]),
        
        'bleft':   find_nearest (msg.ranges[150:190]),
       
        'front':  find_nearest (msg.ranges[90:150]),
        
        'bright':  find_nearest(msg.ranges[240:300]),
        
        'fright': find_nearest (msg.ranges[285:305]),
    }  
      
    twstmsg_= movement()

    
# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)

# Function to check the shape of the membership function

def checkShape(fshape):
    if fshape[0] == fshape[1]:
        return 'Left Shoulder'
    elif fshape[1] == fshape[2]:
        return 'Triangle'
    elif fshape[2] == fshape[3]:
        return 'Right Shoulder'

# Membership Value function
def memValue(inp, fzyset):
    rfb1 = []
    for shpe in fzyset:
        wshape = checkShape(shpe)
        
        if wshape == "Triangle":
            if shpe[0] <= inp <= shpe[2]:
                edge = (inp - shpe[0]) / (shpe[2] - shpe[0])
                rfb1.append('Medium')
                rfb1.append(edge)
            elif shpe[2] <= inp <= shpe[3]:
                edge = (shpe[3] - inp) / (shpe[3] - shpe[2])
                rfb1.append('Medium')
                rfb1.append(edge)
        elif wshape == "Left Shoulder":
            if shpe[0] <= inp <= shpe[2]:
                edge = 1
                rfb1.append('Near')
                rfb1.append(edge)
            elif shpe[2] <= inp <= shpe[3]:
                edge = (shpe[3] - inp) / (shpe[3] - shpe[2])
                rfb1.append('Near')
                rfb1.append(edge)
        elif wshape == "Right Shoulder":
            if shpe[0] <= inp <= shpe[1]:
                edge = (inp - shpe[0]) / (shpe[1] - shpe[0])
                rfb1.append('Far')
                rfb1.append(edge)
            elif shpe[1] <= inp <= shpe[3]:
                edge = 1
                rfb1.append('Far')
                rfb1.append(edge)
                
    return rfb1

def fuzzymain(x_rfs, x_rbs):
    


    rfs, rbs = memValue(x_rfs, dis), memValue(x_rbs, dis)
    f_values = [min(rfs[i], rbs[j]) for i in range(1, len(rfs), 2) for j in range(1, len(rbs), 2)]
    rules = [[rfs[i], rbs[j]] for i in range(0, len(rfs), 2) for j in range(0, len(rbs), 2)]

    DefRules = [ 
                ["Near",   "Near",     "left",   "Slow"   ],
                ["Near",   "Medium",   "left",   "Slow"   ],
                ["Near",    "Far",     "left",   "Average"],
                ["Medium",  "Near",    "right",  "Slow"   ],
                ["Medium",  "Medium",  "zero",   "Average"],
                ["Medium",  "Far",     "left",   "Average"],
                ["Far",     "Near",    "right",   "Slow"  ],
                ["Far",   "Medium",    "right",   "Slow"  ],
                ["Far",     "Far",     "right",   "Fast"  ]
            ]
    direction = [DefRules[j][2:] + [f_values[i]] for i, rule in enumerate(rules) for j, d_rule in enumerate(DefRules) if rule == d_rule[:2]]

    for d in direction:
        if d[0] == 'right':
            d[0] = -1
        elif d[0] == 'zero':
            d[0] = 0
        elif d[0] == 'left':
            d[0] = 1
        if d[1] == 'Slow':
            d[1] = 0.1
        elif d[1] == 'Average':
            d[1] = 0.3
        elif d[1] == 'Fast':
            d[1] = 0.5

    total_f_values = sum(d[2] for d in direction)
    if total_f_values == 0:
        print("Warning: Sum of f_values in direction is zero! Returning default values.")
        return 0, 0  # default values, you can adjust as necessary

    speed, turning = sum(d[1] * d[2] for d in direction) / total_f_values, sum(d[0] * d[1] for d in direction) / total_f_values

    # Clamping the turning value between -1 and 1
    turning = max(-1, min(turning, 1))

    return speed, turning

#Basic movement method
def movement():
    global regions_, mynode_
    
    # Get the region values from the global regions_
    regions = regions_
    x_rfs = regions['fright']  # Providing region for robot's RFS input
    x_rbs = regions['bright']  # Providing region for robot's RBS input
    
    print("Min distance in rightfront region: ", x_rfs)
    print("Min distance in right back region: ", x_rbs)

    # Create an object of Twist class, used to express the linear and angular velocity of the robot
    msg = Twist()

    spd, ster = fuzzymain(x_rfs, x_rbs)

    # If an obstacle is found to be within 0.25 of the LiDAR sensor's front region, the linear velocity is set to 0 (robot stops)
    if regions['front'] < 0.25:
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        return msg
    # If there is no obstacle in front of the robot, it continues to move forward
    else:
        msg.linear.x = spd  # Assigning speed to linear velocity
        msg.angular.z = ster  # Assigning steering to angular velocity
        return msg

#used to stop the rosbot
def stop():
    global pub_
    msg = Twist()
    msg.angular.z = 0.0
    msg.linear.x = 0.0
    pub_.publish(msg)

def main():
    global pub_, mynode_

    rclpy.init()
    mynode_ = rclpy.create_node('reading_laser')

    # Define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,)

    # Create publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # Subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds 
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # Stop the robot
    except:
        stop()  # Stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
