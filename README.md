# Python control of a robot in CoppeliaSim

## CoppeliaSim
CoppeliaSim EDU is an interesting tool to teach how to model dynamically a robot in a virtual environnement. 
https://www.coppeliarobotics.com/

Once the robot has been created, it can be used as a real robot that we can control from an external application.
Here we will control the robot with a Python program using two different ways to access the robot without using the API :
1. using socket
2. using **ROS2** topics

*Note : here all control programms and scripts will be writtent in python (we will not use Lua for scripts as we used to with VRep)*

## The tricycle robot
The dynamic model of the tricycle robot has been defined in CoppeliaSim from the CAD File : https://grabcad.com/library/tricycle-23.
The actuators are the handlebar and the front wheel (this simplified model does not use the force on the two pedals).
The sensors are a sonar (ultrasonic range detector), a GPS and a video camera.

## Using a socket
The server side of the socket will be executed in a script inside CoppeliaSim and the external control program will be executed on the client side of the socket.
### Server side on CoppeliaSim
After loading the tricycle scene, the first thing to do is to add a non threaded script to an object in the scene. Select for example the **tricycle** object and, in the top menu, click Add->Associate child script->Non threaded->Python. Note that as basic sockets are blocking on the server side, using threaded scripts may freeze the simulation. After creating the script, a window will popup with some code in it. Clear all this code and replace it with :
```python
#python
include script_socket_server_blocking_tricycle
```
This allows to develop the server code in our favorite IDE. The script **script_socket_server_blocking_tricycle.py** defines the server side of the socket.

This script is now briefly explained. When the simulation is started, an infinite call to the **server()** function is done :
```python
#python
import socket

def sysCall_thread():
    sim.setThreadAutomaticSwitch(True)
    while True: # infinite call to server
        server()
```
The **server()** function starts with the creation of the socket. Here the control program and the simulator are on the same computer, we use the localhost IP (127.0.0.1). The port number should be greater than 1024 and not used by another executable. Here, we set it arbitrarily to 5000. Then we create the socket, bind the address and port and start listening. We wait for the client (the control program) to connect:
```python
def server():
    host = "127.0.0.1" # IP of server (here localhost as we stay on the same computer)
    port = 5000  # initiate port number (above 1024)
    print ("host: %s:%d"%(host,port)) # debug (print in CoppeliaSim bottom window)

    server_socket = socket.socket()  # get instance
    server_socket.bind((host, port))  # bind host address and port together

    # configure how many client the server can listen simultaneously
    server_socket.listen(1)

    # warning : this is blocking !!!
    conn, address = server_socket.accept()  # accept new connection
    print("Connection from: " + str(address)) # debug
```
Once the client (control program) is connected we enter an infinite loop that will terminate (break) when the client stops. In the first part of the loop, we get the commands (steering and speed) from the client and we apply these values to the actuators (/FrontMotor and /Steering joints) in CoppeliaSim. In the last part, we acquire the simulation time and the sensor data (/FrontSonar) and send them to the client (control program):
```python 
    # communication loop
    while True:
        # receive data stream. it won't accept data packet greater than 1024 bytes
        data = conn.recv(1024).decode()
        if not data:
            print ("close ...") # debug
            # if no data are received, break (client connection lost)
            break
        else:
            # get robot commands (steering and speed)
            print("from connected user: " + str(data))
            sdata = data.split(";")
            steering = float(sdata[0])
            speed = float(sdata[1])
            # and apply these commands to the actuators
            sim.setJointTargetPosition(sim.getObject("/Steering"),steering)
            sim.setJointTargetVelocity(sim.getObject("/FrontMotor"),speed)

            # get sensor data
            simu_time = sim.getSimulationTime() # sim time
            front_sonar = sim.getObject("/FrontSonar") # get sonar handle
            result, distance, detected_point, detected_object_handle, detected_object_normal = sim.handleProximitySensor (front_sonar)
            # and send these sensor data to the client 
            sensor_data = "%.3f;%.2f"%(simu_time, distance)
            conn.send(sensor_data.encode())  # send data to the client (encode string to bytes)

            sim.switchThread() # resume in next simulation step
    conn.close()  # close the connection 
```

### Client side - python control program
The control program on the client side is a standard Python code and does not require any API to work.
This program send the commands (steering and speed) to the robot  and get the simulation time and the distance to the front obstacle.
A very simple control algorithm is implemented to change the direction of the tricycle when a front obstacle is too close.
The **client()** function start with opening the communication socket to the server
```python 
import socket
import math
import time

def client():
    host = "127.0.0.1" # IP of server
    port = 5000  # socket server port number
    client_socket = socket.socket()  # instantiate
    client_socket.connect((host, port))  # connect to the server
```
Then the commands (steering and speed) are initialized
```python 
    # init commands
    speed = 1.0
    steering = 0.0 * math.pi / 180.0
```
Finally, we enter an infinite loop where we send the command to the robot and receive the sensor data from it. from the sensor data, we define the new commands :
```python
    # comunication loop*
    while True:
        message = "%.2f;%.2f"%(steering,speed)  # send commands 
        client_socket.send(message.encode())  # send message
        data = client_socket.recv(1024).decode()  # receive response
        print('Received from server: ' + data)  # debug: show in terminal
        if len(data) > 0:
            sdata = data.split(";")
            sim_time = float(sdata[0])
            distance = float(sdata[1])
            if distance == 0.0 or distance > 1.0:
                steering = 0.95 * steering
                speed *= 1.05
                if speed > 1.0:
                    speed = 1.0
            else:
                steering = 70.0 * math.pi / 180.0
                speed  = 0.5
        time.sleep(0.1)
    client_socket.close()  # close the connection (not used, stop with Ctrl-C)

if __name__ == '__main__':
    client() 
```
The loop terminates when Ctrl-C is typed.

## Using ROS2 topics (Humble)

### Using ROS2 plugin
We will use version 4.6.0 of CoppeliaSim and the Humble version of ROS2. 
ROS2 is initialized with (may already be done in .bashrc file) :
```bash
source /opt/ros/humble/setup.bash
```
The use of the ROS2 plugin is described here : 
https://manual.coppeliarobotics.com/en/ros2Tutorial.htm
We define the path to CoppeliaSim main folder (ex. CoppeliaSim_Edu_V4_6_0_rev18_Ubuntu22_04) :
```bash
export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
```
In the **COPPELIASIM_ROOT_DIR** folder, the ROS2 plugin that interfaces ROS2 to CoppeliSim 
is a shared library called **libsimROS2.so**. With this plugin, CoppeliaSim will be able to publish and to subscribe to topics. 
**libsimROS2.so** can be used as it is if we use standard ROS2 message formats in our topics.
We will see below how to rebuild **libsimROS2.so** with additional ROS2 message formats.

#### Running the example as a test
The example is quite simple, it publishes a topic with the simulation time and subscribe to it. We open an empty scene, add a cuboid and attached the following non threaded python script to the cuboid :
```python
#python
def subscriber_callback(msg):
    # This is the subscriber callback function
    log_message = f'subscriber receiver following Float32: {msg["data"]}'
    sim.addLog(sim.verbosity_scriptinfos, log_message)

def getTransformStamped(objHandle, name, relTo, relToName):
    # This function retrieves the stamped transform for a specific object
    t = simROS2.getSystemTime()
    p = sim.getObjectPosition(objHandle, relTo)
    o = sim.getObjectQuaternion(objHandle, relTo)
    return {
        'header': {
            'stamp': t,
            'frame_id': relToName
        },
        'child_frame_id': name,
        'transform': {
            'translation': {'x': p[0], 'y': p[1], 'z': p[2]},
            'rotation': {'x': o[0], 'y': o[1], 'z': o[2], 'w': o[3]}
        }
    }

def sysCall_init():
    sim = require('sim')
    simROS2 = require('simROS2')

    # The child script initialization
    self.objectHandle = sim.getObject('.')
    self.objectAlias = sim.getObjectAlias(self.objectHandle, 3)
    
    # Prepare the float32 publisher and subscriber (we subscribe to the topic we publish):
    self.publisher = simROS2.createPublisher('/simulationTime', 'std_msgs/msg/Float32')
    self.subscriber = simROS2.createSubscription('/simulationTime', 'std_msgs/msg/Float32', subscriber_callback)

def sysCall_actuation():
    # Send an updated simulation time message, and send the transform of the object attached to this script:
    simROS2.publish(self.publisher, {'data': sim.getSimulationTime()})
    simROS2.sendTransform(getTransformStamped(self.objectHandle, self.objectAlias, -1, 'world'))
    # To send several transforms at once, use simROS2.sendTransforms instead

def sysCall_cleanup():
    # Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    simROS2.shutdownPublisher(self.publisher)
    simROS2.shutdownSubscription(self.subscriber)
```
When executing the simulation, an error may appear on subscriber's callback function. A workaround can be found here :
https://forum.coppeliarobotics.com/viewtopic.php?p=40358&hilit=ros2#p40358

- Change the callback function name **subscriber_callback**to a string **"subscriber_callback"** in the python script
```python 
    self.subscriber = simROS2.createSubscription('/simulationTime', 'std_msgs/msg/Float32', subscriber_callback)
```
with:
```python
    self.subscriber = simROS2.createSubscription('/simulationTime', 'std_msgs/msg/Float32', "subscriber_callback")
```
- Comment line 160 in lua/functional.lua file in **COPPELIASIM_ROOT_DIR**:
```lua
 -- assert(type(getvar(func)) == 'function')
```
After restarting CoppelisSim, it should work with the simulation time displayed in the text area below the 3D view.
In another terminal, we can check the ROS2 topics :
```bash
ros2 topic list
```
Note that a tf has been published using the pose of the object with function **getTransformStamped()** in the python script : 
```bash
ros2 topic echo /simulationTime
ctrl+C
ros2 topic echo /tf
ctrl+C
```

#### Adding a new ROS2 message format

We will add **sensor_msgs/msg/NavSatFix.msg** a ROS2 format for GPS (GNSS) localization of the robot.
First we need to clone the git repo with the ROS2 plugin and if not already done define **COPPELIASIM_ROOT_DIR** env variable.
Then we build and install the plugin
```bash
git clone https://github.com/CoppeliaRobotics/simROS2.git
export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
cd simROS2
```
Before compiling the plugin we need to install some dependencies :
```bash
sudo apt install -y xsltproc
python3 -m pip install --user xmlschema
```
Now we can build and install the plugin :
```bash 
colcon build
source install/setup.bash
```
At the end if the file **meta/interfaces.txt** we add a line with **sensor_msgs/msg/NavSatFix**, we can also add **sensor_msgs/msg/NavSatStatus** and rebuild :
```bash
colcon build
source install/setup.bash
```

#### Using two simple topics to control the robot
Basically similar to Foxy, just a few changes in the code , see below in the Foxy section for details


#### Using standard /cmd_vel topic to control the robot
Basically similar to Foxy, just a few changes in the code , see below in the Foxy section for details.
The only change is that we do not need to recompile the **ROS2** plugin  **libsimExtROS2.so** as the **geometry_msgs/msg/Twist** message type is now in the default list.

For controlling directly **/cmd_vel** withe the keyboard or the mouse, there are several ROS2 packages that can be used, for example **teleop_tools**.
We assume that a terminal has been open :
```bash
cd /to/your/working/folder  # change with the right path
mkdir -p ws/src
cd ws/src
git clone https://github.com/ros-teleop/teleop_tools.git
cd teleop_tools
rm -Rf .git
cd ../..
colcon build
source install/setup.bash
```
We can now use for example the mouse to move the tricycle
```bash
ros2 run mouse_teleop mouse_teleop  --remap /mouse_vel:=/cmd_vel &
```
Note that the output of **mouse_teleop** is a topic (with twist format) called **/mouse_vel** and we have to remap it to **/cmd_vel** twist topic that moves the tricyle.

#### Creating a translation node
Basically similar to Foxy, just a few changes in the code , see below in the Foxy section for details.

#### Codes and scenes
All codes and scenes are in the **ros2-humble** folder :

 - the control of the robot with  two simple topics **/steering** and **/speed** is in **tricycle-ros2-speed-steering.ttt** with python code of the non threaded script in **script_ros2_speed_steering_tricycle.py**
 - the control of the robot with  a single **/cmd_vel** topic is in **tricycle-ros2-cmd-vel.ttt** with python code of the non threaded script in **script_ros2_cmd_vel_tricycle.py**
 - the ROS2 workspace is **ws**. It contains in **src** subfolder two packages : **coppeliasim_ros2_tools** for the translation node and **teleop_tools** for the teleopration of the robot via the mouse or the keyboard

## Using ROS2 topics (Foxy)

### Using ROS2 plugin
Before starting CoppeliaSim, **ROS2** must have been initialized, for example with the bash command :
```bash
source /opt/ros/foxy/setup.bash
```
The documentation using **ROS2** in **CoppeliaSim** is here : 
https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm

#### Using two simple topics to control the robot
After loading the tricycle scene, the first thing to do is to add to an object in the scene a non threaded script. Select for example the **tricycle** object and, in the top menu, click Add->Associate child script->Non threaded->Python. After creating the script, a window will popup with some code in it. Clear all this code and replace it with :
```python
#python
include script_ros2_speed_steering_tricycle
```
We will use 2 separate topics to control the robot **/speed** and **/steering**. The messages on these topics are scalar 32 bits floating numbers (std_msgs/msg/Float32) and we will use a single topic for the front sonar sensor **/front_sonar**.
The **script_ros2_speed_steering_tricycle.py** starts by defining global variables for the publisher and the subscribers in case we need them in different functions.
```python
#python

publisher_front_sonar = None
subscriber_steering = None
subscriber_speed = None
```
At initialization, we create the publisher and the 2 subscribers :
```python
def sysCall_init():
    global publisher_front_sonar, subscriber_steering, subscriber_speed
    # Prepare the float32 publisher and subscribers 
    if simROS2:
        publisher_front_sonar=simROS2.createPublisher('/front_sonar','std_msgs/msg/Float32')
        subscriber_steering=simROS2.createSubscription('/steering','std_msgs/msg/Float32','subscriber_steering_callback')
        subscriber_speed=simROS2.createSubscription('/speed','std_msgs/msg/Float32','subscriber_speed_callback')
```
The subscribers need each a callback function to process the arriving topics. We can put these callbacks before **sysCall_init()** function in the python code :
```python
def subscriber_speed_callback(msg):
    print (msg.keys()) # for debug (to get the keys of the python dict associated to the topic)
    sim.addLog(sim.verbosity_scriptinfos,'subscriber received speed = '+str(msg['data']))
    speed = msg['data']
    # and apply speed to front wheel motor joint in CoppeliaSim (/FrontMotor)
    sim.setJointTargetVelocity(sim.getObject("/FrontMotor"),speed)

def subscriber_steering_callback(msg):
    print (msg.keys()) # for debug (to get the keys of the python dict associated to the topic)
    sim.addLog(sim.verbosity_scriptinfos,'subscriber received steering = '+str(msg['data']))
    steering = msg['data']
    # and apply steering to handlebar joint in CoppeliaSim (/Steering)
    sim.setJointTargetPosition(sim.getObject("/Steering"),steering)   
```
Now we publish the distance to the front obstacle : 
```python
def sysCall_actuation():
    global publisher_front_sonar, subscriber_steering, subscriber_speed
    # Send an updated distance for the front obstacle
    front_sonar = sim.getObject("/FrontSonar")
    result, distance, detected_point, detected_object_handle, detected_object_normal = sim.handleProximitySensor (front_sonar)
    if simROS2:
       simROS2.publish(publisher_front_sonar,{'data':distance})
    pass
```
And finally, we can do some cleaning :
```python
def sysCall_cleanup():
    global publisher_front_sonar, subscriber_steering, subscriber_speed
    # Following not really needed in a simulation script (i.e. automatically shut down at simulation end):
    if simROS2:
        simROS2.shutdownPublisher(publisher_front_sonar)
```

To test it we just need to publish the **/speed** and **/steering** topics and to echo the **/front_sonar** topic :
```bash
ros2 topic pub --once /steering std_msgs/msg/Float32 "{'data' : -1.0}"
ros2 topic pub --once /steering std_msgs/msg/Float32 "{'data' : 0.0}"
ros2 topic pub --once /speed std_msgs/msg/Float32 "{'data' : 0.5}"
ros2 topic pub --once /speed std_msgs/msg/Float32 "{'data' : 0.0}"
ros2 topic echo /front_sonar
```

#### Using standard /cmd_vel topic to control the robot

The python script **script_ros2_cmd_vel_tricycle.py** replace the two topics **/speed** and **/steering** by a single topic **/cmd_vel**.
To get it working we need to recompile the **ROS2** plugin  **libsimExtROS2.so** as the **geometry_msgs/msg/Twist** message type is not in the default list.
Info on how to recompile the plugin can be found in the README.md here https://github.com/CoppeliaRobotics/simExtROS2.git.
Here we will use a [procedure](https://gitlab.com/bnzr/python_control_coppeliasim/-/tree/master#Compiling-CoppeliaSim-ROS2-Plugin)
that use directly the code of the plugin that comes with the installation.

After adding **geometry_msgs/msg/Twist** in **meta/interfaces.txt**, we can use **/cmd_vel** to control the robot, 
with for example **teleop_twist_keyboard** to control the robot with the keyboard.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Compiling CoppeliaSim ROS2 Plugin
First we need to locate where **CoppeliaSim** is installed, for ex :
```
/home/user/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04
```
and we have to define the environnement variable **COPPELIASIM_ROOT_DIR** with this path , e.g.
```bash
export COPPELIASIM_ROOT_DIR=/home/user/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04
```
We may need to add some packages :
```bash
sudo apt install -y xsltproc 
sudo pip install catkin_pkg empy lark  pyparsing pyyaml setuptools argcomplete xmlschema
```
Instead of cloning the repo, we will use the plugin code already installed.
```bash
cd $COPPELIASIM_ROOT_DIR/programming/ros2_packages/sim_ros2_interface/
```
To add new message types, the file to modify is **meta/interfaces.txt**. After making the changes (e.g. adding **geometry_msgs/msg/Twist**), we setup **ROS2** (if not already done), then we compile the plugin and we copy the modified plugin **libsimExtROS2.so** in the  **CoppeliaSim** root folder :
```bash
source  /opt/ros/foxy/setup.bash
colcon build
cp -v ./build/sim_ros2_interface/libsimExtROS2.so $COPPELIASIM_ROOT_DIR/
```
Note : if something goes wrong you can use debug mode or permissive mode
```bash
colcon build --cmake-args ' -DCMAKE_BUILD_TYPE=Debug'
colcon build --cmake-args ' -DCMAKE_CXX_FLAGS=-fpermissive'
```
If a compilation error occurs on example_interfaces/action/Fibonacci you may have to comment it in **meta/interfaces.txt**.

#### Creating a translation node
Instead of adding new messages types in **CoppeliaSim** we can just keep using simple messages and design a **ROS2** node to make the translation between the simple topics known by **CoppeliaSim** and the **ROS2** topics. Here we will first define the location of the **ROS2** workspace and create a Python package (you can use a C++ package instead). As we know we will use sensors and geometry message types, we can add them using **--dependencies** at the package creation. The node is named **coppeliasim_ros2_interface** 
in the package **coppeliasim_ros2_tools** :
```bash
source  /opt/ros/foxy/setup.bash  # setup ROS2 if not already done ...
export MY_WORK_DIR=/home/user/some_working_dir  # replace with your working folder path
cd $MY_WORK_DIR
mkdir -p ws/src
cd ws/src
ros2 pkg create --build-type ament_python coppeliasim_ros2_tools --node-name coppeliasim_ros2_interface --dependencies rclpy geometry_msgs sensors_msgs
```
The files in the **coppeliasim_ros2_tools** package are :
```bash
tree coppeliasim_ros2_tools
```
```
coppeliasim_ros2_tools
|-- coppeliasim_ros2_tools
|   |-- __init__.py
|   `-- coppeliasim_ros2_interface.py
|-- package.xml
|-- resource
|   `-- coppeliasim_ros2_tools
|-- setup.cfg
|-- setup.py
`-- test
    |-- test_copyright.py
    |-- test_flake8.py
    `-- test_pep257.py
```
Once the package is created, we need to update **<description>** **<maintainer>** and **<license>** tags in **package.xml** and to make the same update in **maintainer**, **maintainer_email**, **description** and **license** variables in **setup.py**. The **setup.py** should have been correctly created to execute the **main()** function of  **coppeliasim_ros2_interface.py** that just print
'Hi from coppeliasim_ros2_tools.' We can test it :
```bash
cd $MY_WORK_DIR/ws
colcon build
source install/setup.bash
ros2 run coppeliasim_ros2_tools coppeliasim_ros2_interface
```
If the node name **--node-name coppeliasim_ros2_interface** was not defined at the package creation, the file **setup.py** should have been modified and the file **coppeliasim_ros2_interface.py** should have been created. If we need to add new nodes in this package we will have to do it manually.
Now, we will subscribe to **/cmd_vel**, decompose it and publish its content in two simple float messages on topics **/speed** and **/steering**.

#### Codes and scenes
All codes and scenes are in the **ros2-foxy** folder
