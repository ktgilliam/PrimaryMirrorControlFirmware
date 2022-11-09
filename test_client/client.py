import socket
import json
import sys

#MOVEABSOLUTE = 5       Velocity in rad/sec
#MOVERELATIVE = 6       Velocity in rad/sec  
#MOVERAWABSOLUTE = 7    Velocity in steps/sec
#MOVERAWRELATIVE = 8    Velocity in steps/sec
#MOVEFOCUS = 9,         Velocity in um/sec
#MOVEFOCUSRAW = 10      Velocity in steps/sec

target_host = '169.254.178.142'
target_port = 1883
# create a socket connection
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# let the client connect
client.connect((target_host, target_port))



# send some data
command0 = '{"PMCMessage":{"Handshake": 57005}}'                 # 0 = rad / sec                     0 = absolute
command1 = '{"PMCMessage":{"FindHome": 100}}'                    # 1 = steps / sec                   1 = relative   
command2 = '{"PMCMessage":{"SetFocus": 10000, "SetVelocity": 1000, "VelUnits": 1 ,"MoveType": 1}}'
command3 = '{"PMCMessage":{"SetTip": 10, "SetTilt": 10, "SetVelocity": 10, "VelUnits": 1, "MoveType": 1}}'
command4 = '{"PMCMessage":{"SetTip": 0.7854, "SetTilt": 0 , "SetVelocity": 50, "VelUnits": 1, "MoveType": 1}}'
command5 = '{"PMCMessage":{"SetTip": 0, "SetTilt": 0.7854 , "SetVelocity": 50, "VelUnits": 1, "MoveType": 1}}'

command10 = '{"PMCMessage":{"Stop": 0}}'
command20 = '{"PMCMessage":{"GetStatus": 0}}'
command30 = '{"PMCMessage":{"GetPositions": 0}}'


try:
    client.send(bytes(command30, encoding='utf-8'))

    #get some data
    while True:
        response = client.recv(4096).decode('utf-8')
        print(response)

        if not response:
            break
        if response == "\0":
            break

        #parsed_response = json.loads(response)

finally: 
    client.close

    sys.exit(0)