from Vehicle import Car,Truck

import math
import random
import matplotlib.pyplot as plt
import numpy as np
from progressbar import *
from pprint import pprint
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.legend_handler import HandlerLine2D
from operator import add,mul,sub,itemgetter



NaN = 1e12

v0Truck = 80.0
lengthTruck = 12.0
v0Car = 120.0
lengthCar = 5.0
T = 1.5
a = 1.0
b = 3.0
s0 = 2.0
v0 = 30.0
polite = 0.3
ath = 0.1
bsafe = 4.0
SL = 35.0

delT = 0.25
iterations = 7000
random.seed(50)

def drange(start, stop, step):
	r = start
	while r < stop:
		yield r
		r += step

def IDM(x, x1, t):
	xprime = x.velocity
	sa = x1.position - x.position - x1.length
	delVa = x.velocity - x1.velocity
	s = s0 + x.velocity*T + (x.velocity*delVa)/(2*(a*b)**(0.5))
	vprime = a*(1 - (x.velocity/v0Car)**4 - (s/sa)**2)
	return [xprime, vprime]

def max_num_cars(lanes):
	max_num = len(lanes[0])
	for i in range(1, len(lanes)):
		if len(lanes[i]) > max_num:
			max_num = len(lanes[i])
	return max_num

def nearest_neighbor(lanes, lane, iteration, c):
	if lane == 0 or lane == 2:
		minimum_distance = NaN
		l = 1
		for i, car in enumerate(lanes[1][iteration]):
			dist = car.dist(c)
			if dist < minimum_distance and car.position <= c.position:
				minimum_distance = dist
				minimum_car = car
				minimum_car_i = i
	else:
		lminimum_distance = NaN
		for i, car in enumerate(lanes[0][iteration]):
			dist = car.dist(c)
			if dist < lminimum_distance and car.position <= c.position:
				lminimum_distance = dist
				lminimum_car = car
				lminimum_car_i = i
		rminimum_distance = NaN
		for i, car in enumerate(lanes[2][iteration]):
			dist = car.dist(c)
			if dist < rminimum_distance and car.position <= c.position:
				rminimum_distance = dist
				rminimum_car = car
				rminimum_car_i = i
		if rminimum_distance == NaN and lminimum_distance != NaN:
			l = 0
			minimum_distance = lminimum_distance
			minimum_car = lminimum_car
			minimum_car_i = lminimum_car_i
		elif lminimum_distance == NaN and rminimum_distance != NaN:
			l = 2
			minimum_distance = rminimum_distance
			minimum_car = rminimum_car
			minimum_car_i = rminimum_car_i
		elif lminimum_distance == NaN and rminimum_distance == NaN:
			minimum_distance = 0.0
		elif rminimum_distance > lminimum_distance:
			l = 2
			minimum_distance = rminimum_distance
			minimum_car = rminimum_car
			minimum_car_i = rminimum_car_i
		else:
			l = 0
			minimum_distance = lminimum_distance
			minimum_car = lminimum_car
			minimum_car_i = lminimum_car_i			

	
	if minimum_distance == 0.0 or minimum_distance == NaN:
		return None
	else:
		return [minimum_car, minimum_car_i, l]

def rk4(f, x, x1, t, h):
	try:
		k1 = f(x, x1, t)
		tempC = map(add, [x.position, x.velocity], [h/2.0*el for el in k1])
		c = Car(pos=tempC[0], vel=tempC[1], c=x.color)
		k2 = f(c, x1, t+h/2.0)
		tempC = map(add, [c.position, c.velocity], [h/2.0*el for el in k2])
		c = Car(pos=tempC[0], vel=tempC[1], c=c.color)
		k3 = f(c, x1, t+h/2.0)
		tempC = map(add, [c.position, c.velocity], [h*el for el in k3])
		c = Car(pos=tempC[0], vel=tempC[1], c=c.color)
		k4 = f(c, x1, t+h)
		k2 = [2.0*el for el in k2]
		k3 = [2.0*el for el in k3]
		step = [h/6.0*el for el in (map(sum, zip(k1, k2, k3, k4)))]
		if step[1] >= -1*bsafe:
			return step
		else:
			return [None, None]
	except:
		return [None, None]

def generate_trajectory(f, h, n, x0, t0):
	# SETUP	
	num_lanes = len(x0)
	t = [t0]
	for j in range(n-1):
		t.append(t[j]+h)
	cars = [[[Car(pos=0,vel=0,c="b") for _ in range(len(x0[0]))] for _ in range(n)] for _ in range(len(x0))]
	for i,lane in enumerate(x0):
		cars[i][0] = lane
	
	print "Generating Trajectory..."
	widgets = ['Percent Done: ', Percentage(), ' ', AnimatedMarker(), ' ', ETA()]
	bar = ProgressBar(widgets=widgets, maxval=n).start()
	# Iteration Loop
	for i in range(1, n):
		bar.update(i)
		# DETERMINE WHICH CARS SWITCH
		laneChanges = []
		for l,car in enumerate([el[i-1][0] for el in cars]):
			cars[l][i][0].position = car.position
			cars[l][i][0].velocity = car.velocity
		for j in range(1, max_num_cars([el[i-1] for el in cars])):
			for lane in range(num_lanes):
				if j < len(cars[lane][i-1]):
					if j < len(cars[lane][i]):
						cars[lane][i][j].position = cars[lane][i-1][j].position
						cars[lane][i][j].velocity = cars[lane][i-1][j].velocity
					else:
						cars[lane][i].append(Car(pos=cars[lane][i-1][j].position, vel=cars[lane][i-1][j].velocity, c=cars[lane][i-1][j].color))
					c = cars[lane][i][j]
					c_leading = cars[lane][i][j-1]

					nn = nearest_neighbor(cars, lane, i-1, c)
					if nn != None:
						# Calculate Incentive
						anprime = rk4(f, nn[0], c, t[i-1], h)[1]
						if anprime != None and anprime >= -1*bsafe:
							acprime = rk4(f, c, cars[nn[2]][i-1][nn[1]-1], t[i-1], h)[1]
							ac = rk4(f, c, c_leading, t[i], h)[1]
							an = rk4(f, cars[nn[2]][i-1][nn[1]], cars[nn[2]][i-1][nn[1]-1], t[i-1], h)[1]
							if acprime != None and ac != None and an != None:
								if j < len(cars[lane][i-1])-1:
									c_following = cars[lane][i-1][j+1]
									aoprime = rk4(f, c_following, c_leading, t[i-1], h)[1]
									ao = rk4(f, c_following, c, t[i], h)[1]			
								else:
									aoprime = 0.0
									ao = 0.0
								if aoprime != None and ao != None:
									incentive = (acprime-ac) + polite*((anprime-an)+(aoprime-ao))
									if incentive > ath:
										alreadyMovedTo = False
										for change in laneChanges:
											if change["car_to"] == nn[0]:
												alreadyMovedTo = True
										if alreadyMovedTo == False:
											laneChanges.append({"lane_from": lane, "lane_to": nn[2], "car_to": nn[0], "car": c})

		# Make changes
		for change in laneChanges:
			for k, CAR in enumerate(cars[change["lane_to"]][i]):
				if CAR == change["car_to"]:
					cars[change["lane_to"]][i].insert(k, change["car"])
					for x, C in enumerate(cars[change["lane_from"]][i]):
						if C == change["car"]:
							del cars[change["lane_from"]][i][x]
					break

		# Removes Cars with pos = 0, vel = 0
		for l, lane in enumerate([el[i] for el in cars]):
			cars[l][i] = [c for c in cars[l][i] if c != Car(pos=0.0, vel=0.0,c=c.color)]

		# MOVE CARS FORWARD
		for lane in range(num_lanes):
			numCars = len(cars[lane][i])
			# Move leading car forward at constant speed
			leadingCar = cars[lane][i][0]
			cars[lane][i][0].position = leadingCar.position + leadingCar.velocity*h
			cars[lane][i][0].velocity = leadingCar.velocity
			cars[lane][i][0].length = leadingCar.length
			# Move other cars forward using RK4 with IDM
			for j in range(1, numCars):
				c = cars[lane][i][j]
				c_leading = cars[lane][i][j-1]
				step = rk4(f, c, c_leading, t[i-1], h)
				if step != [None, None]:
					if step[1] + cars[lane][i][j].velocity <= SL:
						if j < len(cars[lane][i]):
							cars[lane][i][j].position += step[0]
							cars[lane][i][j].velocity += step[1]
						else:
							p = cars[lane][i][j].position + step[0]
							v = cars[lane][i][j].velocity + step[1]
							cars[lane][i].append(Car(p, v, c.color))
	bar.finish()
	return cars,t

def min_max_pos(iteration):
	maxPos = -1*NaN
	minPos = NaN
	for car in iteration:
		if car.position > maxPos:
			maxPos = car.position
	for car in iteration:
		if car.position < minPos:
			minPos = car.position
	return [minPos, maxPos]

def plot_data(i, p, data, t):
	f, (ax1, ax2) = plt.subplots(2, sharex=True)
	print "Plotting..."
	lines = []
	for l, lane in enumerate(data):
		ds = []
		for iteration in lane:
			mmp = min_max_pos(iteration)
			densities = []
			for pos in drange(mmp[0], mmp[1], 1000):
				num_cars_kilo = 0.0
				for car in iteration:
					if car.position >= pos and car.position <= (pos + 1000):
						num_cars_kilo += 1
				densities.append(num_cars_kilo)
			kms = float(math.ceil((mmp[1]-mmp[0])/1000.0))
			ds.append(sum(densities)/kms)
		lines.append(ax1.plot(t, ds)[0])


	for lane in data:
		vs = []
		for iteration in lane:
			velocities = []
			for car in iteration:
				velocities.append(car.velocity)
			vs.append(sum(velocities)/float(len(iteration)))
		ax2.plot(t, vs)

	'''
	for lane in data:
		ss = []
		for iteration in lane:
			spacings = []
			for c, car in enumerate(iteration):
				spacings.append(abs(iteration[c-1].position - car.position - iteration[c-1].length))
			ss.append(sum(spacings)/float(len(iteration)))
		ax3.plot(t, ss)
	'''
	f.legend(lines,["Lane %d" % l for l in range(3)])
	#plt.suptitle("Multi-lane Simulation " + r'$v_o=[%.2f-%.2f]m/s$'% (v0-p, v0))
	plt.suptitle("Multi-lane Simulation " + r'$b=%.2f$' % p)
	ax1.set_ylabel("Avg Density (cars/km)")
	ax1.set_ylim([0, 30])
	ax2.set_ylabel("Avg Velocity (m/s)")
	ax2.set_ylim([10, 50])
	#ax3.set_ylabel("Avg Spacing (m)")
	#ax3.set_ylim([70, 1000])
	plt.xlabel("Time")
	plt.savefig("plots/multi/animation/b3/%s.png"%str(i).zfill(3))
	plt.close()


iteration = 0
'''
for vel in drange(0.25, 15.0, 0.5):
	lanes = []
	for lane in range(3):
		lanes.append([])
		for pos in drange(0, 10000, 50):
			p = pos+random.uniform(0,0)
			v = v0+random.uniform(-1*vel,0)
			c = Car(pos=-1*p, vel=v, c="blue")
			lanes[lane].append(c)

	data, t = generate_trajectory(IDM, delT, iterations, lanes, 0.0)
	plot_data(i, vel, data, t)
	i += 1

for p in drange(0.0, 1.51, 0.05):
	polite = p
	lanes = []
	for lane in range(3):
		lanes.append([])
		for pos in drange(0, 10000, 50):
			p = pos+random.uniform(0,0)
			v = v0+random.uniform(-5,5)
			c = Car(pos=-1*p, vel=v, c="blue")
			lanes[lane].append(c)

	data, t = generate_trajectory(IDM, delT, iterations, lanes, 0.0)
	plot_data(iteration, polite, data, t)
	iteration += 1

'''
for brake in drange(0.0, 4.0, 0.25):
	b = brake
	lanes = []
	for lane in range(3):
		lanes.append([])
		for pos in drange(0, 10000, 50):
			p = pos+random.uniform(0,0)
			v = v0+random.uniform(-5,5)
			c = Car(pos=-1*p, vel=v, c="blue")
			lanes[lane].append(c)

	data, t = generate_trajectory(IDM, delT, iterations, lanes, 0.0)
	plot_data(iteration, b, data, t)
	iteration += 1
'''
lanes = []
for lane in range(3):
	lanes.append([])
	for pos in drange(0, 10000, 50):
		p = pos+random.uniform(0,0)
		v = v0+random.uniform(-5,5)
		c = Car(pos=-1*p, vel=v, c="blue")
		lanes[lane].append(c)

data, t = generate_trajectory(IDM, delT, iterations, lanes, 0.0)
f, (ax1) = plt.subplots(1, sharex=True)
ps = []
vs = []
for lane in data:
	for iteration in lane:
		for C, car in enumerate(iteration):
			ps.append(abs(iteration[C-1].position - car.position - iteration[C-1].length))
			vs.append(car.velocity)
ax1.scatter(ps, vs)
plt.title("Multi-lane Simulation")
ax1.set_xlabel("Spacing (m)")
ax1.set_ylabel("Velocity (m/s)")
plt.show()
'''

