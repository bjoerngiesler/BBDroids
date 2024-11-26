from math import pi, sqrt

class CurioResLPF:
	def __init__(self, cutoff, samplefreq):
		self.order = 1
		self.omega0 = 2*pi*cutoff
		self.dt = 1.0/samplefreq
		self.x = [0.0]*(self.order+1)
		self.y = [0.0]*(self.order+1)
		self.a = [0.0]*self.order
		self.b = [0.0]*(self.order+1)

		self.setCoef()

	def setCoef(self):
		alpha = self.omega0*self.dt
		if self.order == 1:
			self.a[0] = -(alpha-2.0)/(alpha+2.0)
			self.b[0] = alpha / (alpha+2.0)
			self.b[1] = alpha / (alpha+2.0)
		else:
			alphaSq = alpha*alpha
			beta = (1, sqrt(2), 1)
			D = alphaSq*beta[0] + 2*alpha*beta[1] + 4*beta[2]
			self.b[0] = alphaSq/D
			self.b[1] = 2*self.b[0]
			self.b[2] = self.b[0]
			self.a[0] = -(2*alphaSq*beta[0] - 8*beta[2])/D
			self.a[1] = -(beta[0]*alphaSq - 2*beta[1]*alpha + 4*beta[2])/D

	def filter(self, xn):
		self.y[0] = 0
		self.x[0] = xn
		for k in range(0, self.order):
			self.y[0] = self.y[0] + self.a[k]*self.y[k+1] + self.b[k]*self.x[k]
		self.y[0] = self.y[0] + self.b[self.order]*self.x[self.order]
		for k in range(self.order, 0, -1):
			self.y[k] = self.y[k-1]
			self.x[k] = self.x[k-1]

		return self.y[0]

class CurioResHPF:
	def __init__(self, cutoff, samplefreq):
		self.order = 1
		self.a = [0]*self.order
		self.b = [0]*(self.order+1)
		self.omega0 = 2*pi*cutoff
		self.dt = 1.0/samplefreq
		self.tn1 = -self.dt
		self.x = [0.0]*(self.order+1)
		self.y = [0.0]*(self.order+1)

		self.setCoef()

	def setCoef(self):
		alpha = self.omega0 * self.dt
		if self.order == 1:
			alphaFactor = 1/(1+alpha/2.0)
			self.a[0] = -(alpha/2.0 - 1)*alphaFactor
			self.b[0] = alphaFactor
			self.b[1] = -alphaFactor
		elif self.order == 2:
			dtSq = self.dt*self.dt
			c = (self.omega0*self.omega0, sqrt(2)*self.omega0, 1)
			d = c[0]*dtSq + 2*c[1]*self.dt + 4*c[2]
			self.b[0] = 4.0/d
			self.b[1] = -8.0/d
			self.b[2] = 4.0/d
			self.a[0] = -(2*c[0]*dtSq - 8*c[2])/d
			self.a[1] = -(c[0]*dtSq - 2*c[1]*self.dt + 4*c[2])/d

	def filter(self, xn):
		self.y[0] = 0
		self.x[0] = xn

		for k in range(0, self.order):
			self.y[0] = self.y[0] + self.a[k]*self.y[k+1] + self.b[k]*self.x[k]
		self.y[0] = self.y[0] + self.b[self.order] * self.x[self.order]
		for k in range(self.order, 0, -1):
			self.y[k] = self.y[k-1]
			self.x[k] = self.x[k-1]
		return self.y[0]

class CurioResBPF:
	def __init__(self, f0, fw, samplefreq):
		self.order = 2
		self.omega0 = 6.28318530718*f0
		self.domega = 6.28318530718*fw
		self.dt = 1.0/samplefreq
		self.tn1 = -self.dt
		self.x = [0.0]*(self.order+1)
		self.y = [0.0]*(self.order+1)		
		self.a = [0.0]*(self.order+1)		
		self.b = [0.0]*(self.order+1)

		self.setCoef()

	def setCoef(self):
		alpha = self.omega0*self.dt
		alphaSq = alpha*alpha
		Q = self.omega0/self.domega
		D = alphaSq + 2*alpha/Q + 4
		self.b[0] = 2*alpha/(Q*D)
		self.b[1] = 0
		self.b[2] = -self.b[0]
		self.a[0] = 0
		self.a[1] = -(2*alphaSq - 8)/D
		self.a[2] = -(alphaSq - 2*alpha/Q + 4)/D

	def filter(self, xn):
		self.y[0] = 0
		self.x[0] = xn

		for k in range(0, self.order+1):
			self.y[0] = self.y[0] + self.a[k]*self.y[k] + self.b[k]+self.x[k]

		for k in range(self.order, 0, -1):
			self.y[k] = self.y[k-1]
			self.x[k] = self.x[k-1]

		return self.y[0]

class MultiFilter:
	def __init__(self, f0, f1, freq):
		self.lpf1 = CurioResLPF(f0, freq)
		self.lpf2 = CurioResLPF(f1, freq)
	def filter(self, xn):
		#return self.lpf1.filter(xn)
		return self.lpf2.filter(xn) - self.lpf1.filter(xn)
