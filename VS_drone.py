from dronekit import connect, VehicleMode, LocationGlobalRelative
import socket
import time
import math

def haversine_distance(lat1, lon1, lat2, lon2):
    r = 6371000 
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = math.sin(dLat / 2) * math.sin(dLat / 2) + \
        math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * \
        math.sin(dLon / 2) * math.sin(dLon / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = r * c
    return distance

def current_location(veh):
    return [veh.location.global_relative_frame.lat,veh.location.global_relative_frame.lon,veh.location.global_relative_frame.alt]

server_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_host='localhost'
server_port=12345
server_socket.bind((server_host,server_port))
server_socket.listen(1)

print("waiting for connection")
client_socket,client_address=server_socket.accept()
print(f"Connection established with {client_address}")

vehicle=connect('udp:127.0.0.1:14562',wait_ready=False)
vehicle.mode=VehicleMode("GUIDED")
vehicle.armed=True

while not vehicle.armed:
    print("Waiting to Arm...")
    time.sleep(1)

while True:
    data=client_socket.recv(1024)
    dat=data.decode()
    print(dat)
    if dat.lower()=="start":
        break

altitude=20
vehicle.simple_takeoff(altitude)
while vehicle.location.global_relative_frame.alt<0.95*altitude:
    print(f"Gaining Altitude. Currently at {vehicle.location.global_relative_frame.alt} metres")
    time.sleep(1)

while True:
    data=client_socket.recv(34)
    if data.decode()=='0000000000000000000000000000000000':
        break
    loc=data.decode()
    location=loc.split(',')
    print(location)
    global lat,lon,alt
    lat=float(location[0])
    lon=float(location[1])
    alt=float(location[2])
    point=LocationGlobalRelative(lat,lon,alt)
    vehicle.simple_goto(point)
    vehicle.airspeed=float(location[3])
    print(current_location(vehicle))
    time.sleep(1)

final_msg=client_socket.recv(28)
final_location=final_msg.decode().split(',')
print(final_location)
f_lat=float(final_location[0])
f_lon=float(final_location[1])
f_alt=float(final_location[2])
final_point=LocationGlobalRelative(f_lat,f_lon,f_alt)
vehicle.simple_goto(final_point)

while haversine_distance(current_location(vehicle)[0],current_location(vehicle)[1],f_lat,f_lon)>2.5:
    print(current_location(vehicle))
    time.sleep(1)

# final_msg=client_socket.recv(28)
# final_location=final_msg.decode().split(',')
# f_lat=float(final_location[0])
# f_lon=float(final_location[1])
# f_alt=float(final_location[2])
# final_point=LocationGlobalRelative(f_lat,f_lon,f_alt)
# vehicle.simple_goto(final_point)
# while haversine_distance(current_location(vehicle)[0],current_location(vehicle)[1],f_lat,f_lon)>1:
#     print(current_location(vehicle))
#     time.sleep(1)

vehicle.mode=VehicleMode("LAND")
while current_location(vehicle)[2]>0.3:
    print(f"Landing. Current Altitude is {current_location(vehicle)[2]} metres")
    time.sleep(1)

vehicle.armed=False
vehicle.close()
client_socket.close()
server_socket.close()