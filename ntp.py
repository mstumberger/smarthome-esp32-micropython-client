import network
import time
def gettime_ntp():
    # http://code.activestate.com/recipes/117211-simple-very-sntp-client/
    import socket
    import struct
    TIME1970 = 2208988800      # Thanks to F.Lundh
    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client.sendto(b'\x1b' + 47 * b'\0', socket.getaddrinfo('time.nist.gov', 123)[0][-1])
    data, address = client.recvfrom(1024)
    if data:
        t = struct.unpack('!12I', data)[10]
        t -= TIME1970
        return t

# ENABLE WIFI
station = network.WLAN(network.STA_IF)
station.active(True)
# CONNECT
station.connect("TEMP_SERV", "supertemp")

# WAIT UNTIL CONNECTED
while not station.isconnected():
    print("sleep")
    time.sleep(1)

print(gettime_ntp())