#python

publisher_front_sonar = None
subscriber_cmd_vel = None

def subscriber_cmd_vel_callback(msg):
    print (msg.keys()) # for debug (to get the keys of the python dict associated to the topic)
    sim.addLog(sim.verbosity_scriptinfos,'subscriber received cmd_vel : linear='+str(msg['linear'])+', angular='+str(msg['angular']))
    speed = msg['linear']['x']*5.0
    steering = msg['angular']['z']
    # and apply these commands to the actuators (joints) in CoppeliaSim (/Steering and /FrontMotor)
    sim.setJointTargetPosition(sim.getObject("/Steering"),steering)
    sim.setJointTargetVelocity(sim.getObject("/FrontMotor"),speed)

def sysCall_init():
    sim = require('sim')
    simROS2 = require('simROS2')
    global publisher_front_sonar, subscriber_cmd_vel
    # Prepare the float32 publisher and subscriber
    if simROS2:
        publisher_front_sonar=simROS2.createPublisher('/front_sonar','std_msgs/msg/Float32')
        subscriber_cmd_vel=simROS2.createSubscription('/cmd_vel','geometry_msgs/msg/Twist','subscriber_cmd_vel_callback')

def sysCall_actuation():
    global publisher_front_sonar, subscriber_cmd_vel
    # Send an updated distance for the front obstacle
    front_sonar = sim.getObject("/FrontSonar")
    result, distance, detected_point, detected_object_handle, detected_object_normal = sim.handleProximitySensor (front_sonar)
    if simROS2:
       simROS2.publish(publisher_front_sonar,{'data':distance})
    pass

def sysCall_cleanup():
    global publisher_front_sonar, subscriber_cmd_vel
    # Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    if simROS2:
        simROS2.shutdownPublisher(publisher_front_sonar)
