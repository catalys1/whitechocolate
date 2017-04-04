


class SkillStateMachine(object):

	def __init__(self):
		self.running = False
		self.states = []


	def start(self, args):
		pass


	def update(self, args):
		pass


class Rotate90StateMachine(SkillStateMachine):

	def __init__(self):
		super(Rotate90StateMachine,self).__init__()
		self.states = [0]
		self.state = 0
		self.theta_start = 0


	def start(self, args):
		self.theta_start = args
		self.running = True


	def update(self, args):

		# desired position - need to figure out how to make the controller
		# work with this properly (wrap around)
		desired = (theta_start + 90) % 360

		if self.running
			if args > desired - tol and args < desired + tol:
				self.running = False
				self.theta_start = 0