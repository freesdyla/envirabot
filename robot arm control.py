import socket
import time
HOST = "192.168.0.2"
PORT = 30001

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

MESSAGE = bytearray(b'movej([-3.1415926,-1.308997,-1.797689,-0.453786,1.570796,-3.1415926],a=0.5,v=0.9)\n') # inside chamber
#MESSAGE = bytearray(b'movej([-3.512475,0.10472,-2.46266,-0.811578,1.935745,-3.13688],a=0.5,v=0.5)\n') # topview home
#MESSAGE = bytearray(b'movej([0.290597,-1.53589,-2.48569,0.258658,1.316502,-2.98608],a=0.5,v=0.3)\n') # sideview home
#MESSAGE = bytearray(b'movej([-3.141592653,0.0,-1.972222,-2.740167,1.464331,-3.141592653],a=0.5,v=1.5)\n') # topview calib

#MESSAGE = bytearray(b'powerdown()\n')

s.send(MESSAGE)       
s.close()
