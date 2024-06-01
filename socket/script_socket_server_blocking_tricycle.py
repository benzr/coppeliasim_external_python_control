#python
import socket


def sysCall_thread():
    sim.setThreadAutomaticSwitch(True)
    while True: # infinite call to server
        server()

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
