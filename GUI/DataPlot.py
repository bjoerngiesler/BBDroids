import dearpygui.dearpygui as dpg

import os.path

class DataPlot:
	def __init__(self, labels, maxnumpoints):
		self.curves_ = []
		for i in range(len(labels)):
			self.curves_.append([])
		self.timedata_ = []
		self.series_ = []
		self.labels_ = labels
		self.maxnumpoints_ = maxnumpoints
		self.recordFile_ = None
		self.fileFirstLine_ = False
		self.paused_ = False

	def recordCallback(self):
		if self.recordFile_ is None:
			fname = dpg.get_value(self.csvText_)
			if fname=="":
				fname = "bb8.csv"
			elif not fname.endswith(".csv"):
				fname = fname + ".csv"
			if os.path.isfile(fname):
				print("Appending to " + fname)
				self.recordFile_ = open(fname, "a")
				self.fileFirstLine_ = False
			else:
				print("Creating " + fname)
				self.recordFile_ = open(fname, "w")
				self.fileFirstLine_ = True
			dpg.configure_item(self.recordButton_, label="Stop")
		else:
			self.recordFile_.close()
			self.recordFile_ = None
			dpg.configure_item(self.recordButton_, label="Record")

	def pauseCallback(self):
		if self.paused_ == True:
			dpg.configure_item(self.pauseButton_, label="Pause")
			self.paused_ = False
		else:
			dpg.configure_item(self.pauseButton_, label="Continue")
			self.paused_ = True

	def createGUI(self, width, height, label):
		with dpg.group(horizontal=True):
			self.recordButton_ = dpg.add_button(label="Record", callback=self.recordCallback)
			self.csvText_ = dpg.add_input_text(label="CSV")
			self.pauseButton_ = dpg.add_button(label="Pause", callback=self.pauseCallback)

		with dpg.plot(label=label, width=width, height=height-30	):
			dpg.add_plot_legend()
			self.xAxis_ = dpg.add_plot_axis(dpg.mvXAxis, label="t")
			self.yAxis_ = dpg.add_plot_axis(dpg.mvYAxis, label="value")
			for i in range(len(self.labels_)):
				self.series_.append(dpg.add_line_series([], [], label=self.labels_[i], parent=self.yAxis_))

	def addDataVector(self, timestamp, vector):
		# collect data
		if len(vector) != len(self.labels_):
			print("Trying to add %d-vector to data plot with size %d" & (len(vector), len(self.labels_)))
			return
		for i in range(len(vector)):
			self.curves_[i].append(vector[i])
			while len(self.curves_[i]) > self.maxnumpoints_:
				del self.curves_[i][0]
		self.timedata_.append(timestamp)
		while len(self.timedata_) > self.maxnumpoints_:
			del self.timedata_[0]

		if self.paused_ == True:
			return

		# configure plot
		for i in range(len(self.curves_)):
			dpg.set_value(self.series_[i], [self.timedata_, self.curves_[i]])
		dpg.set_axis_limits(self.xAxis_, self.timedata_[0], self.timedata_[-1])

		if self.recordFile_ is None:
			return

		# record if desired
		if self.fileFirstLine_ is True:
			print("# timestamp; ", end='', file=self.recordFile_)
			for label in self.labels_[:-1]:
				print(label+"; ", end='', file=self.recordFile_)
			print(self.labels_[-1], file=self.recordFile_)
			self.fileFirstLine_ = False
		print("%f; " % timestamp, end='', file=self.recordFile_)
		for v in vector[:-1]:
			print("%f; " % v, end='', file=self.recordFile_)
		print("%f" % vector[-1], file=self.recordFile_)		


