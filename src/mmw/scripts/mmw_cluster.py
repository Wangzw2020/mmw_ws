#! /usr/bin/env python3
import sys

import rospy
import socket
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
HOST = '192.168.0.227'
PORT = 8886

class points:
	def __init__(self):
		self.id=0
		self.property='None'
		self.x=0
		self.y=0
		self.vx=0
		self.vy=0
		self.RCS=0

def analyse(data,pub_marker):
	print("length:",len(data))
	if data[0]==202 and data[1]==203 and data[2]==204 and data[3]==205:
		print("start")				#包头
	data_length=data[4]*256+data[5]
	print("data_length:",data_length)		#数据长度
	num=int((data_length-33)/12)
	print("total_num=",num)
	if data[6]==1:
		print("Object")				#数据类型
	elif data[6]==2:
		print("Cluster")
	print("id=?")					#设备编号
	print("time:%s.%s.%s %s:%s:%s" %(data[27],data[28],data[29],data[30],data[31],data[32]))			#数据时间
	all_points=[]
	if len(data)>41:
		for k in range(num):
			p = points()
			print("point_id:")			#点数据
			
			p.id=data[35+12*k]
			print("target_id=%d" %p.id)		#点编号
			
			if data[36+12*k]==0:
				p.type="moving"
			elif data[36+12*k]==1:
				p.type="still"
			elif data[36+12*k]==2:
				p.type="coming"
			elif data[36+12*k]==3:
				p.type="maybe still"
			elif data[36+12*k]==4:
				p.type="unknown"
			elif data[36+12*k]==5:
				p.type="across still"
			elif data[36+12*k]==6:
				p.type="crossing"
			elif data[36+12*k]==7:
				p.type="stop"
			print("point_type:%s" %p.type)			#点属性
			
			p.x=(data[37+12*k]*256+data[38+12*k])/100
			if data[39+12*k]>=122.5:
				p.y=(data[39+12*k]*256+data[40+12*k]-65536)/100
			else:
				p.y=(data[39+12*k]*256+data[40+12*k])/100
			print("point_position:(%sm,%sm)" %(p.x,p.y))		#点位置
			
			if data[41+12*k]>=122.5:
				p.vx=(data[41+12*k]*256+data[42+12*k]-65536)/100
			else:
				p.vx=(data[41+12*k]*256+data[42+12*k])/100	
			if data[43+12*k]>=122.5:
				p.vy=(data[43+12*k]*256+data[44+12*k]-65536)/100
			else:
				p.vy=(data[43+12*k]*256+data[44+12*k])/100	
			print("point_speed:vx=%sm/s vy=%sm/s" %(p.vx,p.vy))	#点速度
			
			if data[45+12*k]>=122.5:
				p.RCS=(data[45+12*k]*256+data[46+12*k]-65536)/10
			else:
				p.RCS=(data[45+12*k]*256+data[46+12*k])/10
			print("point_RCS:%s" %p.RCS)
			
			all_points.append(p)
			print(data)	
	else:
		print("No points!")
	if data[-1]==237 and data[-2]==236 and data[-3]==235 and data[-4]==234:
		print("end")
		publish_marker_msg(pub_marker,all_points)
	return 0
	
def publish_marker_msg(pub,all_points):
	markerarray = MarkerArray()
	num = len(all_points)
	for i in range(num):
		marker = Marker()
		marker.header.frame_id = 'mmw'
		marker.header.stamp = rospy.Time.now()
		p = all_points[i]
		marker.ns = p.type
		marker.id = p.id
		
		marker.type = Marker.CUBE
		marker.action = Marker.ADD
		
		marker.pose.position.x = p.x
		marker.pose.position.y = p.y
		marker.pose.position.z = 0.0
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = 0.0
		marker.pose.orientation.w = 1.0
		
		marker.scale.x = 0.2
		marker.scale.y = 0.2
		marker.scale.z = 1.0
		
		marker.color.r = 0.5
		marker.color.g = 0.5
		marker.color.b = 0.5
		marker.color.a = 1
		marker.lifetime = rospy.Duration(1/15)
		marker.text = '(' + str(p.vx) + ',' + str(p.vy) + ')'
		markerarray.markers.append(marker)
	pub.publish(markerarray)
	
def talker():
	pub_marker = rospy.Publisher('mmw_marker',MarkerArray,queue_size=1)
	sk = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	sk.bind((HOST,PORT))
	sk.listen(1)
	conn,addr = sk.accept()
	print ('You got a connection from client, IP:',addr)
	while not rospy.is_shutdown():
		data = conn.recv(1024)
		analyse(data,pub_marker)
		conn.sendall(data)
	conn.close()

if __name__ == '__main__':
	rospy.init_node("mmw_node", anonymous=True)
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
	rospy.spin()
