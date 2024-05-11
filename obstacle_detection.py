
import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformRegistration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
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

# FUNCTION TO IMPLEMENT PID CONTROLLER


def clbk_laser(msg):
    global regions_, twstmsg_
    
    regions_ = {
        #LIDAR readings are anti-clockwise
        #'right':  find_nearest(msg.ranges[85:95]),
       # 'fright': find_nearest (msg.ranges[130:140]),
        #'front':  find_nearest (msg.ranges[175:185]),
       # 'fleft':  find_nearest (msg.ranges[220:230]),
       # 'left':   find_nearest (msg.ranges[265:275]),
        
        
        'right':  find_nearest(msg.ranges[95:105]),
        'fright': find_nearest (msg.ranges[130:140]),
        'front':  find_nearest (msg.ranges[175:185]),
        'fleft':  find_nearest (msg.ranges[220:230]),
        'left':   find_nearest (msg.ranges[265:275]),
        'bright': find_nearest (msg.ranges[310:320]),
    }    
    twstmsg_= movement()

    
# Find nearest point
def find_nearest(list):
    f_list = filter(lambda item: item > 0.0, list)  # exclude zeros
    return min(min(f_list, default=10), 10)

# Function to check the shape of the membership function

def checkShape(fshape):
    if fshape[0] == fshape[1]:
        #print("shape is a left shoulder")
        return shape[0]
    elif fshape[1] == fshape[2]:
        #print("shape is a Triangle")
        return shape[1]
    elif fshape[2] == fshape[3]:
       #print("shape is a right shoulder")
        return shape[2]

# Membership Value function

def memValue(inp,fzyset):
    rfb1 = []
    
    for shpe in fzyset:  
        wshape = checkShape(shpe)
        
        if wshape == "Triangle":                 # Shape of "Medium" is Triangle                         
                
            if shpe[0] <= inp <= shpe[2]:           
                edge = ( inp - shpe[0] ) / (shpe[2] - shpe[0])     #Rising edge
                
                rfb1.append('Medium')
                rfb1.append(edge)
                    
            elif shpe[2] <= inp <= shpe[3]:
                edge = ( shpe[3] - inp ) / (shpe[3] - shpe[2])     #Falling edge
               
                rfb1.append('Medium')
                rfb1.append(edge)
        

        elif wshape == "Left Shoulder":           # Shape of "Near" is Left Shoulder
            
            
            if shpe[0] <= inp <= shpe[2]:         
                edge = 1                           # Edge is "Near"
                
                rfb1.append('Near')
                rfb1.append(edge)
                 
            elif shpe[2] <= inp <= shpe[3]:                      # Shape of "Far" is Right Shoulder
                edge = ( shpe[3] - inp ) / (shpe[3] - shpe[2])    #Falling edge
                
                rfb1.append('Near')
                rfb1.append(edge)
                 
                
        elif wshape == "Right Shoulder":                               #Far
            if shpe[0] <= inp <= shpe[1]:
                edge = ( inp - shpe[0] ) / (shpe[1] - shpe[0])          #Rising edge
                
                rfb1.append('Far')
                rfb1.append(edge)
                    
            elif shpe[1] <= inp <= shpe[3]:
                edge = 1
                
                rfb1.append('Far')           # Edge is "Far"
                rfb1.append(edge)
                    
    return rfb1 
 
# Defining fuzzy function

def fuzzymain(x_right,x_med,x_left):
    near = [0,0,0.2,0.4]       
    med = [0.2,0.4,0.4,0.6]
    far = [0.4,0.6,0.8,0.8]

    dis = [near, med, far]      # Left Front sensor
    distance = dist = dis       # Left Front sensor equals Right Front sensor, Medium Front sensor

    shape =['Left Shoulder','Triangle','Right Shoulder']

    def checkShape(fshape):
    if fshape[0] == fshape[1]:
        return 'Left Shoulder'
    elif fshape[1] == fshape[2]:
        return 'Triangle'
    elif fshape[2] == fshape[3]:
        return 'Right Shoulder'

    def memValue(inp,fzyset):
        rfb1 = []
        
        for shpe in fzyset:  
            wshape = checkShape(shpe)
            
            if wshape == "Triangle":                 # Shape of "Medium" is Triangle                         
                    
                if shpe[0] <= inp <= shpe[2]:           
                    edge = ( inp - shpe[0] ) / (shpe[2] - shpe[0])     #Rising edge
                    
                    rfb1.append('Medium')
                    rfb1.append(edge)
                        
                elif shpe[2] <= inp <= shpe[3]:
                    edge = ( shpe[3] - inp ) / (shpe[3] - shpe[2])     #Falling edge
                
                    rfb1.append('Medium')
                    rfb1.append(edge)
            

            elif wshape == "Left Shoulder":           # Shape of "Near" is Left Shoulder
                
                
                if shpe[0] <= inp <= shpe[2]:         
                    edge = 1                           # Edge is "Near"
                    
                    rfb1.append('Near')
                    rfb1.append(edge)
                    
                elif shpe[2] <= inp <= shpe[3]:                      # Shape of "Far" is Right Shoulder
                    edge = ( shpe[3] - inp ) / (shpe[3] - shpe[2])    #Falling edge
                    
                    rfb1.append('Near')
                    rfb1.append(edge)
                    
                    
            elif wshape == "Right Shoulder":                               #Far
                if shpe[0] <= inp <= shpe[1]:
                    edge = ( inp - shpe[0] ) / (shpe[1] - shpe[0])          #Rising edge
                    
                    rfb1.append('Far')
                    rfb1.append(edge)
                        
                elif shpe[1] <= inp <= shpe[3]:
                    edge = 1
                    
                    rfb1.append('Far')           # Edge is "Far"
                    rfb1.append(edge)
                        
        return rfb1
    
    rfs = memValue(x_right,dis)
    print(rfs)

    mfs = memValue(x_med,dist)
    print(mfs)

    lfs = memValue(x_left,distance)
    print(lfs)

    
# Calculation of firing strengths

    f1= min(rfs[1],mfs[1],lfs[1])  
    print("Firing strength of f1: ", f1)

    f2= min(rfs[1],mfs[1],lfs[3])   
    print("Firing strength of f2: ", f2)

    f3= min(rfs[1],mfs[3],lfs[1])   
    print("Firing strength of f3: ", f3)

    f4= min(rfs[1],mfs[3],lfs[3])   
    print("Firing strength of f4: ",f4)

    f5= min(rfs[3],mfs[1],lfs[1])   
    print("Firing strength of f5: ",f5)

    f6= min(rfs[3],mfs[1],lfs[3])   
    print("Firing strength of f6: ",f6)

    f7= min(rfs[3],mfs[3],lfs[1])   
    print("Firing strength of f7: ",f7)

    f8= min(rfs[3],mfs[3],lfs[3])   
    print("Firing strength of f8: ",f8)

    firing_str = [f1,f2,f3,f4,f5,f6,f7,f8]
    print("Overall firing strenghths ", firing_str)

# extracting (near, medium, far) from the rfs and rbs lists to compare them with the rules:

    rule1 = []   
    for i in range(len(rfs)):    #traversing from 0 to length of list
        if rfs[i] == "Near" or rfs[i] == "Medium" or rfs[i] == "Far":   
            rule1.append(rfs[i])                   #if matched, appending its value to the rfs list
        
    rule2 = [] 
    for j in range(len(mfs)):    #traversing from 0 to length of list
        if mfs[j] == "Near" or mfs[j] == "Medium" or mfs[j] == "Far":   
            rule2.append(mfs[j])                  #if matched, appending its value to the mfs list
                
    rule3 = [] 
    for k in range(len(lfs)):    #traversing from 0 to length of list
    	if lfs[k] == "Near" or lfs[k] == "Medium" or lfs[k] == "Far":   
            rule3.append(lfs[k])                 #if matched, appending its value to the lfs list
            
    print(rule1)    # Right Front sensor outputs
    print(rule2)    # Medium Front sensor outputs
    print(rule3)    # Left Front sensor outputs
	
    rulecompare = [[r,m,l] for r in rule1 for m in rule2 for l in rule3] 
    print(rulecompare)

#Defining Rules: Right Front sensor,Medium Front sensor outputs, Left Front sensor outputs , turning, speed

    DefRules = [ 
             ["Near",   "Near",    "Near",     "left",   "Slow"   ],
             ["Near",   "Near",    "Medium",   "left",   "Slow"   ],
             ["Near",   "Near",    "Far",      "left",   "Average"],
              
             ["Near",   "Medium",  "Near",      "left",   "Slow"],
             ["Near",   "Medium",  "Medium",    "left",   "Average"],
             ["Near",   "Medium",   "Far",      "left",   "Fast"],
    
             ["Near",    "Far",   "Near",       "left",   "Fast"],
             ["Near",    "Far",   "Medium",     "left",   "Fast"],
             ["Near",    "Far",    "Far",       "left",   "Fast"],
    
            
             ["Medium",  "Near",    "Near",     "right",    "Slow"  ],
             ["Medium",  "Near",    "Medium",    "zero",   "Slow"],
             ["Medium",  "Near",     "Far",     "left",   "Average"],
             
             ["Medium",   "Medium",  "Near",      "zero",   "Average"],
             ["Medium",   "Medium",  "Medium",    "zero",   "Average"],
             ["Medium",   "Medium",  "Far",       "left",   "Average"],
    
             ["Medium",    "Far",    "Near",      "right",   "Average"],
             ["Medium",    "Far",    "Medium",    "right",   "Average"],
             ["Medium",    "Far",    "Far",       "left",   "Average"],
    
    
             ["Far",     "Near",    "Near",      "left",   "Slow"  ],
             ["Far",     "Near",    "Medium",    "left",   "Slow"  ],
             ["Far",     "Near",    "Far",       "left",   "Fast"  ],
    
             ["Far",     "Medium",   "Near",      "left",   "Average"  ],
             ["Far",     "Medium",   "Medium",    "left",   "Average"  ],
             ["Far",     "Medium",   "Far",       "left",   "Average"  ],
    
             ["Far",     "Far",      "Near",      "left",   "Slow"  ],
             ["Far",     "Far",      "Medium",    "left",   "Slow"  ],
             ["Far",     "Far",      "Far",       "zero",   "Fast"  ],
           
    
           ]
	
    direction =[]

    for i in range(len(rulecompare)):
    	for j in range(len(DefRules)):
        
            if (rulecompare[i][0] == DefRules[j][0]) and (rulecompare[i][1] == DefRules[j][1]) and (rulecompare[i][2] == DefRules[j][2]):               # if X output(near, med) matches with the #defined rules, then appending steering and speed values to the list
                direction.append([ DefRules[j][3], DefRules[j][4] ])
                        
    direction[0].extend([f1])     # extending the list of column values of speed and steering with firing strengths
    direction[1].extend([f2])     # i.e. [steering,speed,firing strength]
    direction[2].extend([f3])
    direction[3].extend([f4])
    direction[4].extend([f5]) 
    direction[5].extend([f6]) 
    direction[6].extend([f7]) 
    direction[7].extend([f8]) 
	

    print(direction[0])
	
#center of gravity values for speed

    speed_turn = direction.copy()    #copying direction list to another list, [steering,speed,firing strength]

    print("Speed and turning of robot is :", speed_turn)

    for i in range(len(speed_turn)):
        if speed_turn[i][1] == 'Slow':   # index 1 is speed's center of gravity
            speed_turn[i][1] = 0.1
        elif speed_turn[i][1] == 'Average':
            speed_turn[i][1] = 0.3
        elif speed_turn[i][1] == 'Fast':
            speed_turn[i][1] = 0.5
            
    print(speed_turn)



    for j in range(len(speed_turn)):
        if speed_turn[j][0] == 'right':      # index 0 is steering's center of gravity
            speed_turn[j][0] = -1
        elif speed_turn[j][0] == 'zero':
            speed_turn[j][0] = 0
        elif speed_turn[j][0] == 'left':
            speed_turn[j][0] = 1
            
    print(speed_turn)
	
    # speed

    speed = (speed_turn[0][2]*speed_turn[0][1] + speed_turn[1][2]*speed_turn[1][1] + speed_turn[2][2]*speed_turn[2][1]) / (speed_turn[0][2] + speed_turn[1][2] + speed_turn[2][2])
    print("Speed of robot is:", speed)

    # turning/steering

    turning = (speed_turn[0][0]*speed_turn[0][1] + speed_turn[1][0]*speed_turn[1][1] + speed_turn[2][0]*speed_turn[2][1]) / (speed_turn[0][2] + speed_turn[1][2] + speed_turn[2][2])
    print("Turning of robot is:", turning)
	

    return speed, turning


#Basic movement method
def movement():
    global regions_, mynode_
    regions = regions_
    
    x_right = regions['fright']                # Providing region for robot's RFS input
    x_med = regions['front']                # Providing region for robot's front sensor input
    x_left = regions['fleft']                  # Providing region for robot's LFS sensor input
    
    print("Min distance in right region: ", x_right)
    print("Min distance in front region: ", x_med)
    print("Min distance in left region: ", x_left)
    

    #create an object of twist class, used to express the linear and angular velocity of the turtlebot 
    msg = Twist()

    spd, ster = fuzzymain(x_right, x_med, x_left)

    
    #If an obstacle is found to be within 0.25 of the LiDAR sensors front region the linear velocity is set to 0 (turtlebot stops)
    if (regions['front'])< 0.25:
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        return msg
    #if there is no obstacle in front of the robot, it continues to move forward
    else:
        msg.linear.x = spd
        msg.angular.z = ster
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

    # define qos profile (the subscriber default 'reliability' is not compatible with robot publisher 'best effort')
    qos = QoSProfile(
        depth=10,
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    )

    # publisher for twist velocity messages (default qos depth 10)
    pub_ = mynode_.create_publisher(Twist, '/cmd_vel', 10)

    # subscribe to laser topic (with our qos)
    sub = mynode_.create_subscription(LaserScan, '/scan', clbk_laser, qos)

    # Configure timer
    timer_period = 0.2  # seconds 
    timer = mynode_.create_timer(timer_period, timer_callback)

    # Run and handle keyboard interrupt (ctrl-c)
    try:
        rclpy.spin(mynode_)
    except KeyboardInterrupt:
        stop()  # stop the robot
    except:
        stop()  # stop the robot
    finally:
        # Clean up
        mynode_.destroy_timer(timer)
        mynode_.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
