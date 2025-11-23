#!/usr/bin/env python3

import os

f = open("systemsoundlist.txt", "r")
h = open("SystemSounds.h", "w");

print("namespace SystemSounds {", file=h)

lines = f.readlines()
for line in lines:
	num = line[:3]
	text = line[4:].strip()
	if text == "":
		continue
	print(num)
	print(text)
	
	constname = ""
	words = text.split()
	for w in words[:-1]:
		w = w.replace(".", "")
		w = w.replace(",", "")
		constname = constname + w.upper() + "_"
	w = words[-1].replace(".", "")
	w = w.replace(",", "")
	constname = constname + w.upper()
	print(("  static const uint8_t " + constname + " = %d;") % int(num), file=h)

	os.system("say -v Samantha -o aif/\""+num+text+"\".aif \"" + text + "\"")
	os.system("ffmpeg -i aif/\""+num+text+"\".aif wav/\""+num+text+"\".wav")
	os.system("ffmpeg -i aif/\""+num+text+"\".aif mp3/\""+num+text+"\".mp3")

print("};", file=h)
