import socket
import json
import sys

#tip/tilt angles in radians

target_host = '169.254.232.24'
target_port = 1883
# create a socket connection
client = socket.socket()
# let the client connect
client.connect((target_host, target_port))



# send some data
command0 = '{"PMCMessage":{"Handshake": 2766}}' # 2766 = 0x0ACE  # 0 = rad / sec                     0 = absolute
command1 = '{"PMCMessage":{"FindHome": 100}}'                    # 1 = steps / sec                   1 = relative   
command2 = '{"PMCMessage":{"SetFocus": 10000, "SetVelocity": 1000, "VelUnits": 1 ,"MoveType": 1}}'
command3 = '{"PMCMessage":{"SetTip": 0, "SetTilt": 0, "SetVelocity": 100, "VelUnits": 1, "MoveType": 0}}'
command4 = '{"PMCMessage":{"SetTip": 1.5708, "SetTilt": 0 , "SetVelocity": 200, "VelUnits": 1, "MoveType": 1}}' 
command5 = '{"PMCMessage":{"SetTip": 0, "SetTilt": 0.7854 , "SetVelocity": 50, "VelUnits": 1, "MoveType": 1}}'


# Focus Absolute, rad/sec
command21 = '{"PMCMessage":{"SetFocus": 6000, "SetVelocity": 0.0175, "VelUnits": 0, "MoveType": 0}}'#good

# Focus Absolute Raw, steps/sec
command22 = '{"PMCMessage":{"SetFocus": 0, "SetVelocity": 1000, "VelUnits": 1, "MoveType": 0}}' #good

# Focus Relative, rad/sec 
command31 = '{"PMCMessage":{"SetFocus": 10000, "SetVelocity": 1000, "VelUnits": 0,"MoveType": 1}}' # good

# Focus Relative Raw, steps/sec
command32 = '{"PMCMessage":{"SetFocus": 10000, "SetVelocity": 1000, "VelUnits": 1,"MoveType": 1}}' #good

# Move absolute, rad/sec
command41 = '{"PMCMessage":{"SetTip": 0, "SetTilt": 0 , "SetVelocity": 0.1745, "VelUnits": 0, "MoveType": 0}}' 

# Move absolute Raw, steps/sec
command42 = '{"PMCMessage":{"SetTip": 0, "SetTilt": 0 , "SetVelocity": 400, "VelUnits": 1, "MoveType": 0}}'

# Move relative, rad/sec
command51 = '{"PMCMessage":{"SetTip": 0.087, "SetTilt": 0.087 , "SetVelocity": 200, "VelUnits": 1, "MoveType": 1}}'

# Move relative Raw, steps/sec
command52 = '{"PMCMessage":{"SetTip": 0.87, "SetTilt": 0.7854 , "SetVelocity": 800, "VelUnits": 1, "MoveType": 1}}'


command = '{"PMCMessage":{"Stop": 0}}'
command20 = '{"PMCMessage":{"GetStatus": 0}}'
command30 = '{"PMCMessage":{"GetPositions": 0}}'


try:
    client.send(bytes(command0, encoding='utf-8'))

    #get some data

    while True:

        response = client.recv(4096).decode('utf-8')
        print(response)

        if not response:
            break
        if response == '\0':
            break

        #parsed_response = json.loads(response)

finally: 
    client.close

    sys.exit(0)