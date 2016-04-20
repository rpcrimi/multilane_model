class Vehicle(object):
	def __init__(self, pos, vel, length, c):
		self.position = pos
		self.velocity = vel
		self.length = length
		self.color = c

	def dist(self, c):
		return abs(self.position - c.position)

	def __repr__(self):
		return str(self)

	def __str__(self):
		return "pos: %f\t vel: %f" % (self.position, self.velocity)

	def __eq__(self, other):
		if isinstance(other, self.__class__):
			return self.__dict__ == other.__dict__
		else:
			return False

	def __ne__(self, other):
		return not self.__eq__(other)

class Car(Vehicle):
	def __init__(self, pos, vel, c):
		super(Car, self).__init__(pos, vel, 5.0, c)


class Truck(Vehicle):
	def __init__(self, pos, vel, c):
		super(Truck, self).__init__(pos, vel, 12.0, c)
