from math import sqrt

def vectorToAngles(vector):
	vecx, vecy, vecz = map(float, vector)
	length = sqrt(vecx**2 + vecy**2 + vecz**2)
	if length == 0:
		return (0, 0)
	vecx = vecx / length
	vecy = vecy / length
	vecz = vecz / length
	print("x %f y %f z %f" % (vecx, vecy, vecz))
	return (0, 0)