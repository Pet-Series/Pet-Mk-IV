# Python Program to Get IP Address
import socket
import commands
print ( commands.getoutput("hostname -I"))

hostname = socket.gethostname()
#IPAddr = socket.gethostbyname(hostname)
IPAddr = socket.gethostbyname(socket.getfqdn())
print("Your Computer Name is:" + hostname)
print("Your Computer IP Address is:" + IPAddr)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
print(s.getsockname()[0])

print((([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0])

