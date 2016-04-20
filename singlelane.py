from Road import Road
from Vehicle import Car,Truck

import math
import random
import matplotlib.pyplot as plt
import numpy as np
from pprint import pprint
from mpl_toolkits.mplot3d import Axes3D
from operator import add,mul,sub,itemgetter

f, (ax1) = plt.subplots(1, sharex=True)
plt.title("Single Lane Model %s" % (r'$v_o=[13-33]m/s$'))
plt.xlabel("Time (s)")
ax1.set_xlim([0, 500])
plt.ylabel("Position (m)")


v0Truck = 80.0
lengthTruck = 12.0
v0Car = 120.0
lengthCar = 5.0
T = 1.5
a = 1.0
b = 3.0
s0 = 2.0
v0 = 23.0


delT = 0.25
iterations = 2000
random.seed(50)

def drange(start, stop, step):
	r = start
	while r < stop:
		yield r
		r += step

def s(v, delV):
	return s0 + v*T + (v*delV)/(2*(a*b)**(0.5))

def IDM(x, x1, t):
	xprime = x.velocity
	sa = x1.position - x.position - x1.length
	delVa = x.velocity - x1.velocity
	vprime = a*(1 - (x.velocity/v0Car)**4 - (s(x.velocity, delVa)/sa)**2)
	return [xprime, vprime]

def rk4(f, h, n, x0, t0):
	t = [t0]
	for j in range(n-1):
		t.append(t[j]+h)

	numCars = len(x0.lanes[0])
	cars = [[[Car(pos=0,vel=0,c="b") for _ in range(len(x0.lanes[0]))] for _ in range(n)] for _ in range(len(x0.lanes))]
	for i,lane in enumerate(x0.lanes):
		cars[i][0] = lane
	# Iteration Loop
	for i in range(1, n):
		for lane in range(len(x0.lanes)):
			# Move leading car forward at constant speed
			leadingCar = cars[lane][i-1][0]
			cars[lane][i][0].position = leadingCar.position + leadingCar.velocity*h
			cars[lane][i][0].velocity = leadingCar.velocity
			cars[lane][i][0].length = leadingCar.length
			# Move other cars forward using RK4 with IDM
			for j in range(1, numCars):
				c = cars[lane][i-1][j]
				c1 = cars[lane][i-1][j-1]
				
				k1 = f(c, c1, t[i-1])

				tempC = map(add, [c.position, c.velocity], [h/2.0*el for el in k1])
				c = Car(pos=tempC[0], vel=tempC[1], c=c.color)
				k2 = f(c, c1, t[i-1]+h/2.0)

				tempC = map(add, [c.position, c.velocity], [h/2.0*el for el in k2])
				c = Car(pos=tempC[0], vel=tempC[1], c=c.color)
				k3 = f(c, c1, t[i-1]+h/2.0)

				tempC = map(add, [c.position, c.velocity], [h*el for el in k3])
				c = Car(pos=tempC[0], vel=tempC[1], c=c.color)
				k4 = f(c, c1, t[i-1]+h)

				k2 = [2.0*el for el in k2]
				k3 = [2.0*el for el in k3]

				step = [h/6.0*el for el in (map(sum, zip(k1, k2, k3, k4)))]
				temp = map(add, [cars[lane][i-1][j].position, cars[lane][i-1][j].velocity], step)
				cars[lane][i][j].position = temp[0]
				cars[lane][i][j].velocity = temp[1]

	return cars,t

lanes = []
for lane in range(3):
	lanes.append([])
	for pos in drange(0, 10000, 50):
		if pos == 0:
			v = v0
		else:
			v = v0+random.uniform(-10,10)
		p = pos+random.uniform(0,0)
		c = Car(pos=-1*p, vel=v, c="blue")
		lanes[lane].append(c)

lanes[0][0].velocity = 33.0
lanes[1][0].velocity = 33.0
lanes[2][0].velocity = 33.0
road = Road(lanes)


data, t = rk4(IDM, delT, iterations, road, 0.0)
'''
vs = []
for iteration in data[0]:
	velocities = []
	for car in iteration:
		velocities.append(car.velocity)
	vs.append(sum(velocities)/float(len(iteration)))
ax1.plot(t, vs)
plt.show()
'''
for carNum in range(len(data[0][0])):
	ps = [el[carNum].position for el in data[0]]
	ax1.plot(t, ps)
plt.show()
