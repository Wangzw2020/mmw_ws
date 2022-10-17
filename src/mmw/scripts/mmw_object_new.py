#! /usr/bin/env python3
import sys
import math
import rospy
import time
import numpy
import socket
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
HOST = '192.168.0.226'
PORT = 8887

theta = 0

class target:
	def __init__(self):
		self.id=0
		self.type='None'
		self.lane=0
		self.length=0
		self.width=0
		self.yaw=0
		self.x=0
		self.y=0
		self.vx=0
		self.vy=0
		self.v=0
		self.a=0
		self.ax=0
		self.ay=0
		self.longitude=0
		self.latitude=0
		self.elevation=0
		self.RCS=0
		self.coefficient=''
		
def analyse(data,pub_marker):
	print("length:",len(data))
	if data[0]==202 and data[1]==203 and data[2]==204 and data[3]==205:
		print("start")				#包头
	data_length=data[4]*256+data[5]
	print("data_length:",data_length)		#数据长度
	num=int((data_length-33)/37)
	print("total_num=",num)
	if data[6]==1:
		print("Object")				#数据类型
	print("id=?")					#设备编号
	print("time:%s.%s.%s %s:%s:%s" %(data[27],data[28],data[29],data[30],data[31],data[32]))			#数据时间
	all_targets=[]
	if len(data)>41:
		for k in range(num):
			t = target()
			print("target_data:")			#目标数据
			t.id=data[35+37*k]*256+data[36+37*k]
			print("target_id=%d" %t.id)		#目标编号
			t.lane=data[37+37*k]
			print("lane_num:%d" %t.lane)	#目标所属车道
			if data[38+37*k]==0:
				t.type="dota"
			elif data[38+37*k]==1:
				t.type="car"
			elif data[38+37*k]==2:
				t.type="truck"
			elif data[38+37*k]==3:
				t.type="pedestrain"
			elif data[38+37*k]==4:
				t.type="motorbike"
			elif data[38+37*k]==5:
				t.type="bike"
			elif data[38+37*k]==6:
				t.type="wide_vehicle"
			print("target_type:%s" %t.type)			#目标类型
			t.length=data[39+37*k]/10						#目标长度
			t.width=data[40+37*k]/10						#目标宽度
			print("target_length:%sm  target_width:%sm" %(t.length,t.width))
			t.yaw=(data[41+37*k]*256+data[42+37*k])/10		#目标偏航角
			t.yaw = -t.yaw
			t.yaw += theta
			print("target_yaw:%s°" %t.yaw)
			#目标位置
			p_x=(data[43+37*k]*256+data[44+37*k])/100
			if data[45+37*k]>=122.5:
				p_y=(data[45+37*k]*256+data[46+37*k]-65536)/100
			else:
				p_y=(data[45+37*k]*256+data[46+37*k])/100
			t.x = numpy.cos(theta * 3.14 / 180) * p_x - numpy.sin(theta * 3.14 / 180) * p_y
			t.y = numpy.sin(theta * 3.14 / 180) * p_x + numpy.cos(theta * 3.14 / 180) * p_y
			print("target_position:(%sm,%sm)" %(t.x,t.y))
			#目标速度
			if data[47+37*k]>=122.5:
				v_x=(data[47+37*k]*256+data[48+37*k]-65536)/100
			else:
				v_x=(data[47+37*k]*256+data[48+37*k])/100
			if data[49+37*k]>=122.5:
				v_y=(data[49+37*k]*256+data[50+37*k]-65536)/100
			else:
				v_y=(data[49+37*k]*256+data[50+37*k])/100
			t.vx = numpy.cos(theta * 3.14 / 180) * v_x - numpy.sin(theta * 3.14 / 180) * v_y
			t.vy = numpy.sin(theta * 3.14 / 180) * v_x + numpy.cos(theta * 3.14 / 180) * v_y
			t.v=(data[51+37*k]*256+data[52+37*k])/100
			print("target_speed:%skm/h vx=%sm/s vy=%sm/s" %(t.v,t.vx,t.vy))
			
			if data[53+37*k]>=122.5:						#加速度
				t.a=(data[53+37*k]*256+data[54+37*k]-65536)/100
			else:
				t.a=(data[53+37*k]*256+data[54+37*k])/100	
			if data[55+37*k]>=122.5:						#x加速度
				t.ax=(data[55+37*k]*256+data[56+37*k]-65536)/100
			else:
				t.ax=(data[55+37*k]*256+data[56+37*k])/100	
			if data[57+37*k]>=122.5:						#y加速度
				t.ay=(data[57+37*k]*256+data[58+37*k]-65536)/100
			else:
				t.ay=(data[57+37*k]*256+data[58+37*k])/100	
			print("target_accelaration:%sm/s² ax=%sm/s² ay=%sm/s²" %(t.a,t.ax,t.ay))
			
			
			t.longitude=data[59+37*k]+data[60+37*k]+data[61+37*k]+data[62+37*k]#?????
			t.latitude=data[63+37*k]+data[64+37*k]+data[65+37*k]+data[66+37*k]
			#print("target_position:(%s,%s)" %(t.longitude,t.latitude))#duowu
			
			if data[67+37*k]>=122.5:
				t.elevation=(data[67+37*k]*256+data[68+37*k]-65536)/10	#海拔
			else:
				t.elevation=(data[67+37*k]*256+data[68+37*k])/10
			print("target_elevation:%sm" %t.elevation)
			
			if data[69+37*k]>=122.5:
				t.RCS=(data[69+37*k]*256+data[70+37*k]-65536)/10
			else:
				t.RCS=(data[69+37*k]*256+data[70+37*k])/10
			print("target_RCS:%s" %t.RCS)
			
			if data[71+37*k]==1:
				t.coefficient='<25%'
			elif data[71+37*k]==2:
				t.coefficient='<50%'
			elif data[71+37*k]==3:
				t.coefficient='<75%'
			elif data[71+37*k]==4:
				t.coefficient='<90%'
			elif data[71+37*k]==5:
				t.coefficient='<95%'
			elif data[71+37*k]==6:
				t.coefficient='<99.9%'
			elif data[71+37*k]==7:
				t.coefficient='<100%'
			else:
				t.coefficient='无效'
			print("coefficient=%s" %t.coefficient)
			all_targets.append(t)
			print(data)	
	else:
		print("No target!")
	if data[-1]==237 and data[-2]==236 and data[-3]==235 and data[-4]==234:
		print("end")
		publish_marker_msg(pub_marker,all_targets)
	return 0
	
def publish_marker_msg(pub,all_targets):
	markerarray = MarkerArray()
	num = len(all_targets)
	for i in range(num):
		marker = Marker()
		#marker.header.frame_id = 'mmw'
		marker.header.frame_id = 'laser_link'
		marker.header.stamp = rospy.Time.now()
		t = all_targets[i]
		marker.ns = t.type
		marker.id = t.id
		
		marker.type = Marker.CUBE
		marker.action = Marker.ADD
		
		temp_x = t.x
		temp_y = t.y
		# 现场更改下毫米波雷达朝向
		t.yaw += 90
		marker.pose.position.x = temp_y
		marker.pose.position.y = -temp_x
		marker.pose.position.z = 0.0
		marker.pose.orientation.x = 0.0
		marker.pose.orientation.y = 0.0
		marker.pose.orientation.z = numpy.sin(0.5*t.yaw*3.14/180)
		marker.pose.orientation.w = numpy.cos(0.5*t.yaw*3.14/180)
		
		marker.scale.x = t.length
		marker.scale.y = t.width
		marker.scale.z = 1.0
		
		marker.color.r = 0.5
		marker.color.g = 0.5
		marker.color.b = 0.5
		marker.color.a = 1
		marker.lifetime = rospy.Duration(1/15)
		marker.text = '(' + str(t.vx) + ',' + str(t.vy) + ')' + str(t.v)
		markerarray.markers.append(marker)
	pub.publish(markerarray)
	
def talker():
	pub_marker = rospy.Publisher('mmw_marker',MarkerArray,queue_size=1)
	sk = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
	flag = 0
	while(flag == 0 and rospy.is_shutdown() == False):
		try:
			sk.bind((HOST,PORT))
			sk.listen(1)
			conn,addr = sk.accept()
			print ('You got a connection from client, IP:',addr)
			flag = 1
		except:
			time.sleep(1)
			print("wait for connection!")
	
	while not rospy.is_shutdown():
		try:
			data = conn.recv(60720)
			if data[0]==202 and data[1]==203 and data[2]==204 and data[3]==205 and data[-1]==237 and data[-2]==236 and data[-3]==235 and data[-4]==234:
				print("Data package is intact!")
				analyse(data,pub_marker)
			else:
				print("Data package is not intact!")
				while(1):
					if data[-1]==237 and data[-2]==236 and data[-3]==235 and data[-4]==234:
						break
					data_add = conn.recv(60720)
					if data_add[0]==202 and data_add[1]==203 and data_add[2]==204 and data_add[3]==205:
						data = data_add
					else:
						data += data_add
				analyse(data,pub_marker)
			#print(data)
			# conn.sendall(data)
		except:
			print("analyse error \n\n\n\n\n\n\n\n analyse error")
			pass
	conn.close()

if __name__ == '__main__':
	theta = rospy.get_param("theta", 10)
	print("now theta: " ,theta)
	rospy.init_node("mmw_node", anonymous=True)
	talker()
	rospy.spin()
