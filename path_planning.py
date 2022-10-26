#!/usr/bin/env python3

''' SETUP '''

import matplotlib.pyplot as plt
import numpy as np
import ros_numpy
import rospy
import tf
import math
import heapq
from gazebo_ros import gazebo_interface
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


global map_info
global base_pos

#--------Callback Functions-------------
def callback_obstacle_angles(msg):
    global obstacle_angles
    obstacle_angles = np.asarray(msg.data)

def callback_obstacle_distances(msg):
    global obstacle_distances
    obstacle_distances = np.asarray(msg.data)

def callback_base_pos(msg):
    global base_pos
    base_pos = np.asarray(msg.data)

def callback_time(msg):
    global time
    global clock
    time = int(str(msg.clock))//1000000000
    clock = True

def control(meta):
	global obstacle_angles
	global obstacle_distances
	global base_pos
	global time
	#global meta
	inicio = time
	## Control Parameters
	k_att = 1
	k_rep = 3
	kvx = 1.5
	kvy = 1.5
	kw = 1
	h = 0.2
	d_max = 0.8
	pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
	loop = rospy.Rate(10)
	# Initial state
	n_sections = 24
	obstacle_angles = np.zeros((n_sections))
	obstacle_distances = np.zeros((n_sections))
	base_pos = np.asarray([2,2,2])
	rospy.sleep(1)
	

	while not rospy.is_shutdown():
		## CONTROL ALGORITHM ##
		#Force vectors calculus
		F_att = k_att*(meta[0:2] - base_pos[0:2])
		F_rep_x = k_rep*(obstacle_distances - d_max)*np.cos(obstacle_angles + base_pos[2])
		F_rep_y = k_rep*(obstacle_distances - d_max)*np.sin(obstacle_angles + base_pos[2])

		F_x = F_att[0] + np.sum(F_rep_x)
		F_y = F_att[1] + np.sum(F_rep_y)
		#Force normalization
		distance_left = math.sqrt(math.pow(meta[0]-base_pos[0],2) + pow(meta[1]-base_pos[1],2))
		normF_x = F_x if distance_left <=1.0 else F_x/math.sqrt(math.pow(F_x,2) + math.pow(F_y,2))
		normF_y = F_y if distance_left <=1.0 else F_y/math.sqrt(math.pow(F_x,2) + math.pow(F_y,2))
		# Gradient step
		x_d = base_pos[0] + h*normF_x 
		y_d = base_pos[1] + h*normF_y 
		#theta_d = math.atan2(y_d - base_pos[1],x_d - base_pos[0])
		theta_d = math.atan2(meta[1] - base_pos[1],meta[0] - base_pos[0])
		# If the step is close enough to the goal, then the reference is the goal
		if math.sqrt(math.pow(meta[0] - x_d,2) + math.pow(meta[1] - y_d,2)) <=0.5:
			x_d = meta[0]
			y_d = meta[1]
			theta_d = meta[2]
		print("Estacion: ",meta)
		print("Pose: ",base_pos)
		#print("Reference: ",x_d,", ",y_d,", ",theta_d)
		# Errors calculations
		error_x = x_d - base_pos[0]
		error_y = y_d - base_pos[1]
		error_w = math.atan2(math.sin(theta_d-base_pos[2]),math.cos(theta_d-base_pos[2]))
		# Message sending
		msg_cmd_vel = Twist()
		# Distance between the robot base and the goal
		dis_error = math.sqrt(math.pow(meta[0] - base_pos[0],2) + math.pow(meta[1] - base_pos[1],2))
		angle_error = math.atan2(math.sin(meta[2]-base_pos[2]),math.cos(meta[2]-base_pos[2]))
		# Condition to continue or stop the control proccess
		if dis_error >= 0.05 or abs(angle_error) >= 0.2:
			if dis_error < .05:
				kw = .3
			msg_cmd_vel.linear.x = kvx*(error_x*math.cos(base_pos[2]) + error_y*math.sin(base_pos[2]))
			msg_cmd_vel.linear.y = kvy*(-error_x*math.sin(base_pos[2]) + error_y*math.cos(base_pos[2]))
			msg_cmd_vel.angular.z = kw*error_w
		else:
			msg_cmd_vel.linear.x = 0
			msg_cmd_vel.linear.y = 0
			msg_cmd_vel.angular.z = 0
			break
			#bandera = False
		pub_cmd_vel.publish(msg_cmd_vel)
		fin = time
		if (fin - inicio) > 3 and meta != [0,0,0]:
			break
		loop.sleep()

def callback_map(msg):
	global map_info
	map_info = msg
	
def callback_base_pos(msg):
	global base_pos
	base_pos = np.asarray(msg.data)
	
def get_minimap(msg):
	# Función callback_map regresa un mapa reducido de 80x80 de los datos del slam. Los valores en el minimapa corresponden a:
	# 0. Zona explorada
	# 1. Zona sin explorar.
	# 2. Obstáculos.
	# Si el 20% del nodo está sin explorar, se considera nodo inexplorado. Si hay al menos 1 pixel de obstáculo, todo el
	# nodo se considera como obstaculizado. 
	
	resolution = 16
	data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
	data[data==-1] = 1
	data[data==100] = 2
	
	m = math.ceil(msg.info.height/resolution)
	n = math.ceil(msg.info.width/resolution)
	
	minimap = np.zeros((m, n), dtype=np.int8)
	for i in range(m):
		for j in range(n):
			subset_data = data[i*resolution:(i+1)*resolution, j*resolution:(j+1)*resolution]
			if len(np.where(subset_data==1)[0]) >= 20:
				minimap[i,j] = 1
			if len(np.where(subset_data==2)[0]) >= 1:
				minimap[i,j] = 2
	
	return minimap
	
def find_closest_node(minimap, node):
	# Regresar los índices del nodo inexplorado que es accesible más cercano a "node".
	
	minimap_cost = 255*np.ones(minimap.shape, dtype=np.uint8)
	
	m = minimap.shape[0]
	n = minimap.shape[1]
	adjacent_idx   = [[1,0],[0,1],[-1,0],[0,-1]]
	
	# Indicar los nodos inexplorados que son accesibles (aquellos que cuentan con una casilla explorada al lado).
	for i in range(m):
		for j in range(n):
			if minimap[i, j] == 0:
				for [x, y] in adjacent_idx:
					if minimap[i+x,j+y] == 1:
						minimap_cost[i+x,j+y] = abs(node[1]-(i+x)) + abs(node[0]-(j+y))
	
	# closest_node es una tupla con los índices del nodo más cercano y la distancia para llegar a el (x, y, distancia).
	closest_node_list = np.where(minimap_cost == np.amin(minimap_cost))
	closest_node = (closest_node_list[1][0], closest_node_list[0][0], np.amin(minimap_cost))
	
	return closest_node
	
def node_to_coords(node):
	# Convierte los indices de un nodo en posiciones en el plano. 0.05 metros es la resolución del slam.
	resolution = 16
	x = (node[0]-24.5)*(resolution*0.05)
	y = (node[1]-24.5)*(resolution*0.05)
	
	return (x, y)
	
def a_star(start_r, start_c, goal_r, goal_c, grid_map):

	cost_map = np.ones(minimap.shape)
	if grid_map[goal_r, goal_c] == 1:
		grid_map[goal_r, goal_c] = 0
	grid_map[start_r, start_c] = 0
	in_open_list   = np.full(grid_map.shape, False)
	in_closed_list = np.full(grid_map.shape, False)
	g_values       = np.full(grid_map.shape, float("inf"))
	f_values       = np.full(grid_map.shape, float("inf"))
	parent_nodes   = np.full((grid_map.shape[0],grid_map.shape[1],2),-1)
	
	adjacent_idx   = [[1,0],[0,1],[-1,0],[0,-1]]
	
	open_list = [] 
	heapq.heappush(open_list, (0, [start_r, start_c]))
	in_open_list [start_r, start_c] = True
	g_values     [start_r, start_c] = 0
	[row, col] = [start_r, start_c]
	iterations = 0
	
	while len(open_list) > 0 and [row, col] != [goal_r, goal_c]:
		[row, col] = heapq.heappop(open_list)[1]         
		in_closed_list[row,col] = True                  
		adjacent_nodes = [[row+i, col+j] for [i,j] in adjacent_idx]
		for [r,c] in adjacent_nodes:
			if grid_map[r,c] == 1 or grid_map[r,c] == 2 or in_closed_list[r,c]: 
				continue
			
			g = g_values[row, col] + abs(row-r) + abs(col-c) + cost_map[r][c]
			h = abs(goal_r - r) + abs(goal_c - c)
			
			f = g + h                         
			if g < g_values[r,c]:           
				g_values[r,c]     = g           
				f_values[r,c]     = f           
				parent_nodes[r,c] = [row, col]   
			if not in_open_list[r,c]:
				in_open_list[r,c] = True              
				heapq.heappush(open_list, (f, [r,c]))            
			iterations += 1
			
	if [row, col] != [goal_r, goal_c]:
		print("Cannot calculate path by A* :'(")
		return []
	print("Path calculated after " + str(iterations) + " iterations.")
	path = []
	while parent_nodes[row, col][0] != -1:
		path.insert(0, [col, row])
		[row, col] = parent_nodes[row, col]
	return path

rospy.init_node("path_planning")
rospy.Subscriber("/map", OccupancyGrid, callback_map)
rospy.Subscriber("/base_link_coords", Float32MultiArray,callback_base_pos)
rospy.Subscriber("/obstacle_detect_angles",Float32MultiArray,callback_obstacle_angles)
rospy.Subscriber("/obstacle_detect_distances",Float32MultiArray,callback_obstacle_distances)
rospy.Subscriber("/clock",Clock,callback_time)
#pub_node_coords = rospy.Publisher("/meta_coords", Float32MultiArray, queue_size=10)
loop = rospy.Rate(10)
rospy.sleep(1)


# Nodo de origen
current_node = (26, 26)

if __name__ == '__main__':
	global time
	ini = time
	while not rospy.is_shutdown():
		
		while clock == False:
			print("Esperando reloj")
			loop.sleep()
		
		#print(ini)
		minimap = get_minimap(map_info)
		
		next_node = find_closest_node(minimap, current_node)
		# En path se tienen los nodos que debe de seguir el robot para llegar hasta el nodo final.
		# Pero path guarda los índices de los nodos, esto se debe convertir a coordenadas con node_to_coords()
		# Los indices se deben de pasar al revés porque el mapa que da el slam está al revés.
		path = a_star(current_node[1], current_node[0], next_node[1], next_node[0], np.copy(minimap))
		
		if path == []:
			meta = [0,0,0]
			control(meta)
			final = time
			print("Inicio:", ini)
			print("Final: ", final) 
			total = int(final - ini)
			print("Duración: ", total," segundos")
			print("El mapeo ha concluído :3")
			exit("Para guardar el mapa, ejecute el comando: rosrun map_server map_saver -f <nombre>")

		# Obtener lista de coordenadas que hacen la trayectoria.
		path_coords = list()
		for node in path:
			path_coords.append(node_to_coords(node))
		print(path_coords)
		
		#exit()
		#plt.imshow(minimap)
		#plt.show()
		
		# Seguir la trayectoria calculada hasta llegar al último nodo explorado. El último nodo no se explora en esta parte.
		if len(path_coords) > 1:
			for i in range(len(path_coords)-1):
				# Convertir nodos a coordenadas y publicarlas como meta a seguir
				coords = Float32MultiArray()
				coords.data = [path_coords[i][0], path_coords[i][1], 0]
				# Esperar a que el robot esté cerca del punto a llegar.
				while math.sqrt(math.pow(path_coords[i][0]-base_pos[0],2) + math.pow(path_coords[i][1]-base_pos[1],2)) > 0.2:
					#print("Publicando coordenadas meta")
					#pub_node_coords.publish(coords)
					control(coords.data)
					rospy.sleep(.2)
				
		# Explorar último nodo de la trayectoria.
		coords = Float32MultiArray()
		coords.data = [path_coords[-1][0], path_coords[-1][1], 0]
		# Esperar a que el estado del nodo inexplorado cambie a Explorado u Obstáculo
		while minimap[next_node[1], next_node[0]] == 1:
			# Actualizar minimapa y publicar coordenadas
			minimap = get_minimap(map_info)
			#print("Publicando coordenadas meta")
			control(coords.data)
			#pub_node_coords.publish(coords)
			rospy.sleep(1)
			
		# Tomar el siguiente nodo sin explorar
		current_node = (next_node[0], next_node[1])  	
		
		
