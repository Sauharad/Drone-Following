from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
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

def reverse_haversine(point,distance,bearing):
    r=6371000
    la1=math.radians(point.lat)
    lo1=math.radians(point.lon)  
    ad=distance/r
    theta=math.radians(bearing)
    lat2=math.degrees(math.asin(math.sin(la1)*math.cos(ad)+math.cos(la1)*math.cos(theta)*math.sin(ad)))
    lon2=math.degrees(lo1+math.atan2(math.sin(theta)*math.sin(ad)*math.cos(la1),math.cos(ad)-math.sin(la1)*math.sin(lat2)))
    return LocationGlobalRelative(lat2,lon2,point.alt)

def current_location(veh):
    return [veh.location.global_relative_frame.lat,veh.location.global_relative_frame.lon,veh.location.global_relative_frame.alt]

client_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server_host='localhost'
server_port=12345
client_socket.connect((server_host,server_port))

vehicle=connect('udp:127.0.0.1:14552',wait_ready=False)
vehicle.mode=VehicleMode("GUIDED")
vehicle.armed=True

while not vehicle.armed:
    print("Waiting to Arm...")
    time.sleep(1)

initial_location=vehicle.location.global_relative_frame

altitude=5
vehicle.simple_takeoff(altitude)
while current_location(vehicle)[2]<0.95*altitude:
    print(f"Gaining Altitude. Currently at {current_location(vehicle)[2]} metres")
    time.sleep(1)

start_msg="start"
client_socket.sendall(start_msg.encode())

altitude=20
p=LocationGlobalRelative(initial_location.lat,initial_location.lon,altitude)
vehicle.simple_goto(p)
while current_location(vehicle)[2]<0.95*altitude:
    print(f"Gaining Altitude. Currently at {current_location(vehicle)[2]} metres")
    time.sleep(1)

point=LocationGlobalRelative(28.75409055,77.11635881,20)
vehicle.simple_goto(point)

while haversine_distance(current_location(vehicle)[0],current_location(vehicle)[1],point.lat,point.lon)>2:
    la=current_location(vehicle)[0]
    lo=current_location(vehicle)[1]
    al=current_location(vehicle)[2]
    lat="{:.7f}".format(la)
    lon="{:.7f}".format(lo)
    alt="{:.3f}".format(al)
    vel="{:.3f}".format(vehicle.airspeed)
    send_str=f"{lat},{lon},{alt},{vel}"
    client_socket.sendall(send_str.encode())
    print(current_location(vehicle))
    time.sleep(1)

end_str='0000000000000000000000000000000000'
client_socket.sendall(end_str.encode())
time.sleep(1)

la=current_location(vehicle)[0]
lo=current_location(vehicle)[1]
al=current_location(vehicle)[2]
final_loc=LocationGlobalRelative(la,lo,al)
point_sent=reverse_haversine(final_loc,5,180)
lat="{:.7f}".format(point_sent.lat)
lon="{:.7f}".format(point_sent.lon)
alt="{:.3f}".format(point_sent.alt)
send_str=f"{lat},{lon},{alt}"
client_socket.sendall(send_str.encode())

client_socket.close()
print("Waypoint Reached")
vehicle.mode=VehicleMode("LAND")
while current_location(vehicle)[2]>0.3:
    print(f"Landing. Current Altitude is {current_location(vehicle)[2]} metres")
    time.sleep(1)

vehicle.armed=False
vehicle.close()