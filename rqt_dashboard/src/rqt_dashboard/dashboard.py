import os
import rospy, rospkg
import sys
import math
import yaml
from itertools import izip_longest

from operator import add, sub

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from PyQt4 import QtGui, QtCore
from rqt_plot.rosplot import ROSData, RosPlotException
from std_msgs.msg import Float32
from last_letter_msgs.msg import RefCommands

class DashboardGrid(QtGui.QWidget):
	def __init__(self):
		super(DashboardGrid, self).__init__()
		self.setFixedSize(900, 900)
		self.setWindowTitle('UAV Dashboard')
		self.setAutoFillBackground(True)

		prefix = '{}'.format(rospy.get_namespace())
		rospack = rospkg.RosPack()
		print("NAMESPACE IS %s" % rospy.get_namespace())
		uav_name = rospy.get_namespace().split("/")[2]
		filename =  rospack.get_path('last_letter') + '/data/parameters/aircraft/' + uav_name + '/dashboard.yaml'
		data = yaml.load(open(filename).read())
		gauges = []
		self.line = QtGui.QHBoxLayout()

		self.commands = RefCommands()
		initPos = rospy.get_param('init/position',[0])
		self.commands.altitude = -initPos[-1]
		initVa = rospy.get_param('init/velLin',[0,0,0])
		self.commands.airspeed = math.sqrt(initVa[0]*initVa[0] + initVa[1]*initVa[1] + initVa[2]*initVa[2])
		self.pub = rospy.Publisher('refCommands', RefCommands, queue_size=1)
		self.pubTimer = QtCore.QTimer()
		self.pubTimer.timeout.connect(self.publishCommands)
		self.pubTimer.start(1000)

		for name in sorted(data.keys()): #sort based on name
			values = data[name]
			# print 'Adding: {}'.format(name)
			values['topic'] = prefix + values['topic']
			values['warning'] = zip(*[iter(values['warning'])]*2)
			values['danger'] = zip(*[iter(values['danger'])]*2)
			gauges.append(GaugeSimple(**values))
			gauges[-1].marker_set.connect(self.onClick)

		grouped_gauges = list(izip_longest(*(iter(gauges),)*3))

		for i in xrange(len(grouped_gauges)):
			setattr(self, 'column{}'.format(i), QtGui.QVBoxLayout())
			curr_column = getattr(self, 'column{}'.format(i))
			for g in grouped_gauges[i]:
				if g is not None:
					curr_column.addWidget(g)
					# curr_column.append(g.topic)
			self.line.addLayout(curr_column)
		self.setLayout(self.line)


	def onClick(self,comList):
		member = comList[0]
		value = comList[1]
		if member == 'Roll':
			self.commands.euler.x = value*math.pi/180
		if member == 'Yaw':
			self.commands.euler.z = value*math.pi/180
		if member == 'Pitch':
			self.commands.euler.y = value*math.pi/180
		elif member == 'Airspeed':
			self.commands.airspeed = value
		elif member == 'Geometric Altitude':
			self.commands.altitude = value

	def publishCommands(self):
		self.commands.header.stamp = rospy.Time.now()
		self.pub.publish(self.commands)
		self.pubTimer.start(1000)

#By Fadi
class GaugeSimple(QtGui.QWidget):
	''' Gauge pointer movement:
	minimum->maximum values: clockwise rotation
	maximum value > minimum-value
	'''
	marker_set = QtCore.pyqtSignal(list)

	def __init__(self, topic='/HUD', length=300.0, end_angle=300.0, min=0.0, max=100.0, main_points=11,
			warning=[], danger=[], multiplier='', units='', description=''):
		super(GaugeSimple, self).__init__()

		self.setFixedSize(300, 300)
		self.setWindowTitle('A Magnificent Gauge')
		self.setAutoFillBackground(True)
		self.redraw_interval = 40
		self.update_plot_timer = QtCore.QTimer()
		self.update_plot_timer.timeout.connect(self.update_plot)
		self._start_time = rospy.get_time()
		self._rosdata = ROSData(topic, self._start_time)

		self.min = min
		self.curr_value = min
		self.value_uncap = 0
		self.max = max
		self.length = length
		self.main_points = main_points
		self.start_angle = (end_angle + length) % 360
		self.end_angle = end_angle % 360
		self.is_circle = self.start_angle == self.end_angle

		self.gauge_ticks = []
		self.bounding_rect = QtCore.QRectF(25.0, 25.0, 250.0, 250.0)
		self.center = QtCore.QPointF(150.0, 150.0)
		self.warning = warning #zones
		self.danger = danger #zones
		self.center_radius = 5.0
		self.margin = 12
		self.units = units
		self.multiplier = multiplier
		self.description = description

		self.update_plot_timer.start(self.redraw_interval)

		#Various ui colors
		self.marker_tick_color = QtGui.QPen(QtGui.QColor('#FF9900'), 1.8)
		self.ui_color = QtGui.QPen(QtCore.Qt.green, 2.5)
		self.ui_color_tick = QtGui.QPen(QtCore.Qt.green, 1.5)
		self.gauge_color = QtGui.QPen(QtCore.Qt.lightGray, 2)
		self.warning_color = QtGui.QPen(QtCore.Qt.yellow, 2)
		self.warning_bg = QtGui.QRadialGradient(self.center, self.width()/3)
		self.warning_bg.setColorAt(0.0, QtCore.Qt.yellow)
		self.warning_bg.setColorAt(1.0, QtCore.Qt.black)
		self.danger_color = QtGui.QPen(QtCore.Qt.red, 2)
		self.danger_bg = QtGui.QRadialGradient(self.center, self.width()/3)
		self.danger_bg.setColorAt(0.0, QtCore.Qt.red)
		self.danger_bg.setColorAt(1.0, QtCore.Qt.black)

		self.current_bg = QtCore.Qt.black

		self.create_gauge()

	def detect_safe_zones(self):
		r = [(self.min, self.max)]
		unsafe = sorted(self.warning+self.danger, key=lambda i:i[0])

		for i in unsafe:
			temp = []
			for y in r:
				if i[0] > y[1] or i[1] < y[0]:
					temp.append(y)
				elif i[0]==y[0] and i[1]==y[1]:
					continue
				elif i[0]>y[0] and i[1]<y[1]:
					temp.append((y[0], i[0]))
					temp.append((i[1], y[1]))
				elif i[0]>y[0] and i[1]==y[1]:
					temp.append((i[0], i[1]))
				elif i[0]==y[0] and i[1]<y[1]:
					temp.append((i[1], y[1]))
			r = temp
		return r

	def create_gauge(self):
		def text_width(text):
			font = QtGui.QFont()
			metrics = QtGui.QFontMetrics(font)
			return metrics.width(text)

		#Main points
		divisor = self.main_points
		if self.start_angle != self.end_angle:
			divisor -= 1

		angle_step = self.length/divisor
		value_step = abs(self.max-self.min)/divisor

		#Gauge main line(the circular path)
		#Safe zones
		zones = map(self.val2deg_tuple, self.detect_safe_zones())
		self.gauge_safe = []
		for zone in zones:
			path = QtGui.QPainterPath()
			path.arcMoveTo(self.bounding_rect, self.start_angle-zone[0])
			path.arcTo(self.bounding_rect, self.start_angle-zone[0], -(zone[1]-zone[0]))
			self.gauge_safe.append(path)
		#Warning zones
		warning_zones =  map(self.val2deg_tuple, self.warning)
		self.gauge_warning = []
		for w in warning_zones:
			path = QtGui.QPainterPath()
			path.arcMoveTo(self.bounding_rect, self.start_angle-w[0])
			path.arcTo(self.bounding_rect, self.start_angle-w[0], -(w[1]-w[0]))
			self.gauge_warning.append(path)
		#Danger zones
		danger_zones =  map(self.val2deg_tuple, self.danger)
		self.gauge_danger = []
		for d in danger_zones:
			path = QtGui.QPainterPath()
			path.arcMoveTo(self.bounding_rect, self.start_angle-d[0])
			path.arcTo(self.bounding_rect, self.start_angle-d[0], -(d[1]-d[0]))
			self.gauge_danger.append(path)

		#Initial gauge position
		self.set_gauge(self.curr_value)

		for i in xrange(self.main_points):
			#Find the point on the curve
			angle = self.start_angle -i*angle_step
			value = self.min + i*value_step
			p = QtGui.QPainterPath()
			p.arcMoveTo(self.bounding_rect, angle)
			x, y = p.currentPosition().x(), p.currentPosition().y()
			x_new = x*0.9 + self.center.x()*0.1
			y_new = y*0.9 + self.center.y()*0.1

			x_text = x*0.8 + self.center.x()*0.2 - (text_width(str(round(value, 1)))-10)/2
			y_text = y*0.8 + self.center.y()*0.2 + 4

			tick_path = QtGui.QPainterPath()
			tick_path.moveTo(x_new, y_new)
			tick_path.lineTo(x, y)


			#Store the tick_length for the marker area
			self.gauge_ticks.append([QtCore.QPointF(x_text, y_text), value, tick_path])

			#Store the tick_length for the marker area
		self.tick_length = math.sqrt((x-x_new)**2+(y-y_new)**2)

	def val2deg(self, value):
		return self.length*((value-self.min)/abs(self.max-self.min))

	def val2deg_tuple(self, t):
		return map(self.val2deg, t)

	def deg2val(self, degrees):
		#Convert the given degress relative to the start_angle to the respective value
		return abs(self.max-self.min)*(degrees/self.length)+self.min

	def mouseReleaseEvent(self, e):
		self.mouseMoveEvent(e)

	def mouseMoveEvent(self, e):
		#marker_line and marker_value dont exist before the first call of this function
		click_pos = e.posF()

		x_coeff = (click_pos.x() - self.center.x())**2
		y_coeff = (click_pos.y() - self.center.y())**2
		dist = math.sqrt(x_coeff + y_coeff)

		w = self.bounding_rect.width()/2

		if w - self.tick_length <= dist <= w:
			#Find the angle between the start angle and the click point
			angle = self.angle_from_zero(self.center, click_pos, self.start_angle)
			#Return if the user clicked outside of the allowed range
			if self.deg2val(angle) > self.max or self.deg2val(angle) < self.min:
				return

			self.set_marker(self.deg2val(angle))

	def angle_from_zero(self, p1, p2, offset):
		angle = math.degrees(math.atan2(p1.y()-p2.y(), p2.x()-p1.x()))

		if angle < 0:
			angle += 360
		angle = offset - angle
		if angle < 0:
			angle += 360

		return angle

	def set_marker(self, value):
		#New values: marker_point
		#Emit the new value
		self.marker_value = max(min(value, self.max), self.min)
		self.marker_set.emit([self.description, self.marker_value])
		#Round it for presentation purposes
		self.marker_value = round(self.marker_value, 2)

		#Create the marker line
		p = QtGui.QPainterPath()
		p.arcMoveTo(self.bounding_rect, self.start_angle - self.val2deg(value))
		self.marker_point = p.currentPosition()
		self.draw_marker(y=3)

	def compute_marker_rotation(self):
		#Marker_point is already set and ready for use
		return self.angle_from_zero(self.center, self.marker_point, 90)

	def draw_marker(self, x=0, y=0, size=10):
		poly = QtGui.QPolygonF()
		poly.append(QtCore.QPointF(x-size, y))
		poly.append(QtCore.QPointF(x+size, y))
		poly.append(QtCore.QPointF(x+size, y-size))
		poly.append(QtCore.QPointF(x, y))
		poly.append(QtCore.QPointF(x-size, y-size))
		poly.append(QtCore.QPointF(x-size, y))

		self.marker_line = QtGui.QPainterPath()
		self.marker_line.addPolygon(poly)

		self.update()

	def update_plot(self):
		try:
			dump, value = self._rosdata.next()
			if value:
				self.value_uncap = round(value.pop(),1)
				#print value
				if len(value)>0:
					self.set_gauge(self.value_uncap)
				self.update_plot_timer.start(self.redraw_interval)
		except RosPlotException as e:
			QtCore.qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)

	def set_gauge(self, value):
		#Clamp between [min, max]
		self.curr_value = round(max(min(value, self.max), self.min),1)

		p = QtGui.QPainterPath()
		p.arcMoveTo(self.bounding_rect, self.start_angle-self.val2deg(self.curr_value))
		x, y = p.currentPosition().x(), p.currentPosition().y()

		self.gauge_line = QtGui.QPainterPath()
		self.gauge_line.moveTo(self.center)
		self.gauge_line.lineTo(x, y)
		self.update()

	def increment_gauge(self, step):
		#Clamp between (min, max)
		self.curr_value = max(min(self.curr_value + step, self.max), self.min)

		p = QtGui.QPainterPath()
		p.arcMoveTo(self.bounding_rect, self.start_angle-self.val2deg(self.curr_value))
		x, y = p.currentPosition().x(), p.currentPosition().y()

		self.gauge_line = QtGui.QPainterPath()
		self.gauge_line.moveTo(self.center)
		self.gauge_line.lineTo(x, y)
		self.update()

	def set_bg_color(self):
		#Determine the zone that the gauge arrow is inside
		#Is it in a warning zone?
		for w in self.warning:
			if w[0] <= self.curr_value <= w[1]:
				self.current_bg = self.warning_bg
				return
		#Or a danger zone?
		for d in self.danger:
			if d[0] <= self.curr_value <= d[1]:
				self.current_bg = self.danger_bg
				return
		#Don't be scared, you're safe!
		self.current_bg = QtCore.Qt.black

	def paintEvent(self, event):
		def center_text(text):
			rect = painter.boundingRect(self.bounding_rect, QtCore.Qt.AlignHCenter, text)
			return rect.width()/2

		painter = QtGui.QPainter(self)
		painter.setRenderHint(QtGui.QPainter.Antialiasing)

		#Draw the background
		self.set_bg_color()
		painter.fillRect(event.rect(), self.current_bg)

		painter.setBrush(QtCore.Qt.transparent)
		painter.setPen(self.gauge_color)
		painter.drawPath(self.gauge_line)

		#Draw the safe zones
		painter.setPen(self.ui_color)
		for s in self.gauge_safe:
			painter.drawPath(s)
		#Draw the warning zones
		painter.setPen(self.warning_color)
		for w in self.gauge_warning:
			painter.drawPath(w)
		#Draw the danger zones
		painter.setPen(self.danger_color)
		for d in self.gauge_danger:
			painter.drawPath(d)

		#Draw the marker line
		if getattr(self, 'marker_line', None):
			painter.setPen(self.marker_tick_color)
			painter.translate(self.marker_point)
			painter.rotate(self.compute_marker_rotation())
			painter.drawPath(self.marker_line)
			painter.resetTransform()

		#Draw the center circle
		painter.setPen(self.ui_color)
		painter.drawEllipse(self.center.x()-self.center_radius/2, self.center.y()-self.center_radius/2, self.center_radius, self.center_radius)

		#Draw the paths
		painter.setPen(self.ui_color_tick)
		for i, path in enumerate(self.gauge_ticks):
			if not (self.is_circle and i == (len(self.gauge_ticks))):
				painter.drawText(path[0], str(int(path[1])))
				painter.drawPath(path[2])

		#Draw the text labels
		painter.drawText(QtCore.QPointF(self.center.x()-center_text(str(self.value_uncap)), self.center.y()-40), str(self.value_uncap))
		painter.drawText(QtCore.QPointF(self.center.x()-center_text(self.multiplier), self.center.y()+20+self.margin), self.multiplier)
		painter.drawText(QtCore.QPointF(self.center.x()-center_text(self.units), self.center.y()+20+self.margin*2), self.units)
		painter.drawText(QtCore.QPointF(self.center.x()-center_text(self.description), self.center.y()+20+self.margin*3), self.description)

		painter.setPen(self.marker_tick_color)
		if getattr(self, 'marker_value', None):
			painter.drawText(QtCore.QPointF(self.center.x()-center_text(str(self.marker_value)), self.center.y()-20), str(self.marker_value))

		QtGui.QWidget.paintEvent(self, event)

class Dashboard(Plugin):

	def __init__(self, context):
		super(Dashboard, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('Dashboard')

		# Process standalone plugin command-line arguments
		from argparse import ArgumentParser
		parser = ArgumentParser()
		# Add argument(s) to the parser.
		parser.add_argument("-q", "--quiet", action="store_true",
			dest="quiet", help="Put plugin in silent mode")
		args, unknowns = parser.parse_known_args(context.argv())
		if not args.quiet:
			print 'arguments: ', args
			print 'unknowns: ', unknowns

			rospy.sleep(2.)
			self._layout = DashboardGrid()

		# Get path to UI file which is a sibling of this file
		# in this example the .ui and .py file are in the same folder
		##ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MyPlugin.ui')
		# Extend the widget with all attributes and children from UI file
		##loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._layout.setObjectName('MyPluginUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._layout.setWindowTitle(self._layout.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface
		context.add_widget(self._layout)

		# foo = yaml.load("- Hesperiidae - Papilionidae - Apatelodidae - Epiplemidae")
		# print foo

	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass

	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass

	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass

	#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

