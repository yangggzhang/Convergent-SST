import csv
import numpy as np
import time
import openravepy

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

if __name__ == "__main__":

	filename = 'solution_path_gripper_2D_OP_4.csv'

	nodes = []
	with open(filename, 'rb') as csvfile:
		reader = csv.reader(csvfile, delimiter=',', quotechar='|')
		state_dim = int(reader.next()[0])
		num_nodes, num_particles = map(int,reader.next())
		i = 0
		while i < num_nodes:
			value = map(float, reader.next())
			if state_dim == 2:
				value.append(0.0)
			if reader.line_num == (3 + i * (num_particles+1)):
				nodes.append(value)
				i = i + 1


		env = Environment()
		env.SetViewer('qtcoin')
		collisionChecker = RaveCreateCollisionChecker(env,'ode')
		env.SetCollisionChecker(collisionChecker)

		env.Reset()
		# load a scene from ProjectRoom environment XML file
		env.Load('../OpenraveEnv/gripper_sys_4claw_simplified.env.xml')
		time.sleep(0.1)

		raw_input("Press any key to show the solution path...")

		robot = env.GetRobots()[0]
		robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.Z)

		handles = []
		for i in range(len(nodes)-1):
			# print array((nodes[i],nodes[i+1]))
			handles.append(env.drawlinestrip(points=array((nodes[i],nodes[i+1])),
                              						linewidth=1.0,
                                           			colors=array((1,0,0))))

		raw_input("Press any key to show the particle distribution...")

	with open(filename, 'rb') as csvfile:
		reader = csv.reader(csvfile, delimiter=',', quotechar='|')
		i = 0
		while i < num_nodes:
			value = map(float, reader.next())
			print reader.line_num
			print value
			if reader.line_num >= 3:
				handles.pop()
				if state_dim == 2:
					value.append(0.0)
				node_pos = value
				if robot.GetName() == "4Claw-Gripper":
					node_pos[0] = node_pos[0] - 0.3;				
					node_pos[1] = node_pos[1] - 0.3;
					node_pos[2] = node_pos[2] - 0.3;
				print "node_pos: ", node_pos
				i = i + 1
				particle_pos = []
				clrs = []
				j = 0
				while j < num_particles:
					value = map(float, reader.next())
					if state_dim == 2:
						value.append(0.0)
					particle_pos.append(value)
					print "particle_pos: ", value
					print j
					clrs.append(array([0,0,1]))
					j = j+1
				
				#Set the robot position
				robot.SetActiveDOFValues(node_pos)
				#Plot particle position
				handles.append(env.plot3(points=array(particle_pos),
							pointsize=0.05,
							colors=array(clrs),
							drawstyle=1))
				raw_input("Press any key to show the particle distribution...")


		raw_input("Press any key to exit...")
		env.Destroy()