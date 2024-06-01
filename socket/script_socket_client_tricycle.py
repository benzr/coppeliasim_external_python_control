import socket
import math
import time

def client():
    host = "127.0.0.1" # IP of server
    port = 5000  # socket server port number
    client_socket = socket.socket()  # instantiate
    client_socket.connect((host, port))  # connect to the server

    # init commands
    speed = 1.0
    steering = 0.0 * math.pi / 180.0
    
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

