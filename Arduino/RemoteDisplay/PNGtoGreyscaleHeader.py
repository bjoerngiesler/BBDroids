#!/usr/bin/env python3

import png
import sys

r = png.Reader(sys.argv[1])
p = r.read()
image = list(p[2])

print("Found %dx%d image" % (len(image[0])/2, len(image)))

outfile = open(sys.argv[2], "w")

print("static unsigned int width=%d;" % (len(image[0])/2), file=outfile)
print("static unsigned int height=%d;" % len(image), file=outfile)

print("#define HEADER_PIXEL_GREYSCALE(data,pixel) { pixel = int((*data-32)*2.71); data++; }", file = outfile)
print("static const char *data = ", file = outfile);

for row in image:
	print("\"", end='', file = outfile)
	i = 0
	while i<len(row):
		pixel = row[i]
		alpha = row[i+1]
		i = i+2
		normed = int(pixel*0.37 + 32)
		if chr(normed) == "\"" or chr(normed) == "\\":
			normed += 1
		print(chr(normed), file=outfile, end='')
	print("\"", file=outfile)
print("\"\";", file=outfile)
