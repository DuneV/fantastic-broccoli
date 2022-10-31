#!/usr/bin/env python3
from math import pi
import rospy
import sys, os
import copy 
from geometry_msgs.msg import Twist 
from std_msgs.msg import Float32, Float32MultiArray
import matplotlib
import matplotlib.pyplot as plt
import numpy as np

# radio de la rueda
radii = 0.035 
# separacion entre ruedas
l = 0.23

simtime = 0
# posicion x del robot (marco global)
pos_x = 0
# posicion y del robot (marco global)
pos_y = 0
# orientacion del robot
pos_theta = None
theta = 0

# pose final deseada [x,y,theta] en marco global
endPos = [1.0, -2.0, -np.pi/4]

# callbacks -----------------------------------

def simTime_callback(msg):
	global simtime
	simtime = msg.data

def orientation_callback(msg):
	global theta
	theta = msg.data	

def position_callback(msg):
	# cada vez que llega un mensaje por el topico turtlebot_position, se extrae la pose
	# note que el tipo de mensaje corresponde a un geometry_msgs/Twist
	global pos_x, pos_y, pos_theta
	pos_x = msg.linear.x
	pos_y = msg.linear.y
	pos_theta = msg.angular.z	

# ---------------------------------------------

# creacion del nodo control_turtlebot2
rospy.init_node('control_turtlebot2', anonymous=True)

# recibe por argumentos pose final deseada
# ejemplo rosrun name_package control_turtlebot 1 -2 1.57
# si no recibe argumentos, la pose final es la definicion de endPos
myargv = rospy.myargv(argv=sys.argv)
if len(myargv)>2:
	endPos[0] = myargv[1] # pos x en metros
	endPos[1] = myargv[2] # por y en metros
	endPos[2] = myargv[3] # orientacion en rad

# creacion del publisher para enviar las velocidades de las ruedas
velocity_publisher = rospy.Publisher('/turtlebot_wheelsVel', Float32MultiArray, queue_size=10)
rate = rospy.Rate(10)

# suscripcion al topico simulationTime
rospy.Subscriber("simulationTime", Float32, simTime_callback)
# suscripcion al topico turtlebot_position - ver position_callback
rospy.Subscriber("turtlebot_position", Twist, position_callback)
# suscripcion al topico turtlebot_orientation - ver orientation_callback
rospy.Subscriber("turtlebot_orientation", Float32, orientation_callback)


# mesaje para enviar la velocidad de giro de la rueda [izquierda, derecha]
vel_msg = Float32MultiArray()
vel_msg.data = [0.0, 0.0]

# variables para guardar el historial de la pose del robot de vrep
vrep_move_x = []
vrep_move_y = []
vrep_theta = []

# variables para monitorear el error de posicion en coordenadas cartesianas
errorPos_x = []
errorPos_y = [] 
errorTheta = []

# variables para graficar 
t = []
plotTime = []

# se guarda la ultima pose del robot (adquira a traves del position_callback)
vrep_move_x.append(pos_x)
vrep_move_y.append(pos_y)
vrep_theta.append(theta)

# se calcula el error de posicion con respecto a los datos de vrep
# en la practiva, los datos de vrep deben corresponder a la estimacion por odometria
errorPos_x.append(endPos[0] - vrep_move_x[-1])
errorPos_y.append(endPos[1] - vrep_move_y[-1])
errorTheta.append(endPos[2] - theta)
plotTime.append(simtime)
t.append(0)

# actualizacion de las variables de control
rho = np.sqrt(endPos[0]**2 + endPos[1]**2)
alpha = -theta + np.arctan2(endPos[1], endPos[0])
beta = errorTheta[-1]

# constantes del PID
K_rho = 0.15 #0.25
K_alpha = 0.5 #0.5
K_beta = 0.0

while not rospy.is_shutdown():
	velX_frameR = 0
	velW_frameR = 0
	vel_msg.data = [0.0, 0.0]
	velocity_publisher.publish(vel_msg)

	#Primero corregimos la orientacion de forma que el robot este alineado con el punto al que se quiere ir
	while np.abs(alpha)>0.01:
		t.append(simtime)
		deltaT = simtime - t[-2]
		if deltaT!=0:
			# se guarda la ultima pose del robot (adquira a traves del position_callback)
			plotTime.append(simtime)
			vrep_move_x.append(pos_x)
			vrep_move_y.append(pos_y)
			vrep_theta.append(theta)
			
			# se calcula el error de posicion con respecto a los datos de vrep
			deltaX = endPos[0] - vrep_move_x[-1]
			deltaY = endPos[1] - vrep_move_y[-1]
			eTheta = endPos[2] - theta

			# actualizacion de las variables de control
			#Si el angulo necesitado esta en el segundo o tercer cuadrante aplicar la correccion
			neededAngle= np.arctan2(deltaY, deltaX)
			if deltaY>=0 and deltaX<0:
				neededAngle+=np.pi
			elif deltaY<0 and deltaX<0:
				neededAngle-=np.pi

			rho = np.sqrt(deltaX**2 + deltaY**2)
			alpha = neededAngle-theta
			beta = eTheta
			
			# se guarda el error de posicion
			errorPos_x.append(deltaX)
			errorPos_y.append(deltaY)
			errorTheta.append(eTheta)

			# control P para alpha -> vel angular en marco local
			velW_frameR = K_alpha*alpha
			if np.abs(velW_frameR) > np.pi:
				velW_frameR = np.pi*np.sign(velW_frameR)

			# calculo de las velocidades de las ruedas
			vr = (velW_frameR*l)/(2*radii)
			vl = (-velW_frameR*l)/(2*radii)
			
			# mensaje con las velocidades para las ruedas izquierda y derecha
			vel_msg.data = [vl, vr]
			velocity_publisher.publish(vel_msg)		

			# se imprime en terminal alpha, rho y el error en orientacion	
			print( str(alpha) + ' ' + str(rho) + ' ' + str(eTheta) )
			rate.sleep()

	# primero vamos a corregir el error [x,y] entre la posicion del robot
	# y la posicion deseada
	# para ello, rho debe aproximarse a 0.0, pero trabajaremos con una
	# tolerancia de 2cm
	while rho>0.02:
		t.append(simtime)
		deltaT = simtime - t[-2]
		if deltaT!=0:
			# se guarda la ultima pose del robot (adquira a traves del position_callback)
			plotTime.append(simtime)
			vrep_move_x.append(pos_x)
			vrep_move_y.append(pos_y)
			vrep_theta.append(theta)
			
			# se calcula el error de posicion con respecto a los datos de vrep
			deltaX = endPos[0] - vrep_move_x[-1]
			deltaY = endPos[1] - vrep_move_y[-1]
			eTheta = endPos[2] - theta

			# actualizacion de las variables de control
			rho = np.sqrt(deltaX**2 + deltaY**2)
			alpha = -theta + np.arctan2(deltaY, deltaX)
			beta = eTheta
			
			# se guarda el error de posicion
			errorPos_x.append(deltaX)
			errorPos_y.append(deltaY)
			errorTheta.append(eTheta)

			# control P para rho -> vel lineal en x en marco local
			velX_frameR = K_rho * rho
			if np.abs(velX_frameR) > 7.0:
				velX_frameR = 7.0*np.sign(velX_frameR)

			vr = velX_frameR/radii
			vl = velX_frameR/radii
			# mensaje con las velocidades para las ruedas izquierda y derecha
			vel_msg.data = [vl, vr]
			velocity_publisher.publish(vel_msg)		

			# se imprime en terminal alpha, rho y el error en orientacion	
			print( str(alpha) + ' ' + str(rho) + ' ' + str(eTheta) )
			rate.sleep()

	# una vez le robot en la posicíon destino, se corrije la orientacion
	while np.abs(beta)>0.01:
		t.append(simtime)
		deltaT = simtime - t[-2]
		K_beta = 0.5
		velX_frameR=0
		if deltaT!=0:
			plotTime.append(simtime)
			vrep_move_x.append(pos_x)
			vrep_move_y.append(pos_y)
			vrep_theta.append(theta)

			deltaX = endPos[0] - vrep_move_x[-1]
			deltaY = endPos[1] - vrep_move_y[-1]
			eTheta = endPos[2] - theta
			beta = eTheta

			errorPos_x.append(deltaX)
			errorPos_y.append(deltaY)
			errorTheta.append(eTheta)

			# control P para beta -> vel angular en marco local
			velW_frameR = K_beta*beta
			if np.abs(velW_frameR) > np.pi:
				velW_frameR = np.pi*np.sign(velW_frameR)

			vr = (velW_frameR*l)/(2*radii)
			vl = (-velW_frameR*l)/(2*radii)

			vel_msg.data = [vl, vr]
			velocity_publisher.publish(vel_msg)			
			print( str(alpha) + ' ' + str(rho) + ' ' + str(eTheta) )
			rate.sleep()

	vel_msg.data = [0.0, 0.0]
	velocity_publisher.publish(vel_msg)
	break


# se muestra una grafica con la trayectoria del robot
plt.figure(1)
plt.subplot(2, 1, 1)
plt.plot(np.array(vrep_move_x), np.array(vrep_move_y))
plt.xlabel('VREP x')
plt.ylabel('VREP y')
plt.subplot(2, 1, 2)
plt.plot(np.array(plotTime), vrep_theta)
plt.ylabel('vrep_theta')

# se muestra una grafica con el error en x, y y theta
plt.figure(2)
plt.subplot(3, 1, 1)
plt.plot(np.array(plotTime), np.array(errorPos_x))
plt.ylabel('Error position in x')
plt.subplot(3, 1, 2)
plt.plot(np.array(plotTime), np.array(errorPos_y))
plt.ylabel('Error position in y')
plt.subplot(3, 1, 3)
plt.plot(np.array(plotTime), np.array(errorTheta))
plt.ylabel('Error orientation')
plt.xlabel('simulation time')
plt.show()


	
