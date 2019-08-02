import socket

UDP_IP = '10.25.215.211'
UDP_PORT = 10000
MESSAGE = bytearray(b'get config 1 rover')

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

try:
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    sock.settimeout(10)
    
    data, addr = sock.recvfrom(1024)
    print("received bytes:", len(data),": ",data)
    
except socket.timeout:
    print("timeout")
    sock.close()


sock.close()
