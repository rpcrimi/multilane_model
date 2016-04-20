class Road(object):
	def __init__(self, lanes):
		self.lanes = lanes

	def posVtime(self, laneNums, variation):
		fig = plt.figure()
		axes = fig.add_subplot(111)

		for lane in self.lanes:
			ps = []
			ts = []
			for vehicle in lane:
				vs.append(vehicle[j].velocity)
				ps.append(vehicle[j].position)
			axes.plot(t, ps)
		
		plt.title("IDM Simulation: %d cars, %s/% variation in Vi" % (len(self.lanes[0], variation)))
		axes.set_xlabel("Time")
		axes.set_ylabel("Position")
		plt.show()

	def render(self, f, kwargs):
		f(**kwargs)