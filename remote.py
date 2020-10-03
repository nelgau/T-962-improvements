#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import serial
import serial.threaded

import matplotlib
import matplotlib.pyplot as plt

import tkinter

import sys
import time
import csv

from threading import Thread
import logging


logger = logging.getLogger('T962A_remote')
logger.setLevel(logging.DEBUG)

# settings
#
FIELD_NAMES = 'Time,Temp0,Temp1,Temp2,Temp3,Set,Actual,Heat,Fan,ColdJ,Mode'
TTYs = ('/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2')
BAUD_RATE = 115200


def get_tty():
	for devname in TTYs:
		try:
			ser = serial.Serial()

			ser.port = devname
			ser.baudrate = BAUD_RATE
			ser.rtscts = False
			ser.dsrdtr = False

			ser.dtr = 0
			ser.rts = 0

			ser.open()

			print('Using serial port %s' % ser.name)

			return ser

		except Exception as e:
			print(e)
			print('Tried serial port %s, but failed.' % str(devname))
			pass

	return None

class T962Connection(serial.threaded.LineReader):
	def __init__(self, consumer):
		super(serial.threaded.LineReader, self).__init__()
		self.consumer = consumer

	def send_command(self, cmd):
		self.write_line(cmd)
		time.sleep(1)

	def set_profile(self, index, setpoints):
		if len(setpoints) > 48:
			error("Too many setpoints")

		self.send_command(f"select profile {index}")
		for i, temp in enumerate(setpoints):
			self.send_command(f"set setpoint {i} {int(temp)}")
		for i in range(len(setpoints), 48):
			self.send_command(f"set setpoint {i} 0")

	def connection_made(self, transport):
		super(T962Connection, self).connection_made(transport)
		logger.debug('Connected to device\n')

	def connection_lost(self, exc):
		if exc:
			logger.error(exc, exc_info=True)
		logger.debug('Disconnected from device\n')

	def handle_line(self, line):
		# print(line)

		# ignore 'comments'
		if line.startswith('#'):
			return

		# parse Profile name
		if line.startswith('Starting reflow with profile: '):
			profile = line[30:].strip()
			try:
				self.consumer.reflow_started(profile)
			except Exception as e:
				logger.error(e, exc_info=True)
			return

		if line.startswith('Reflow done'):
			try:
				self.consumer.reflow_finished()
			except Exception as e:
				logger.error(e, exc_info=True)
			return

		if line.startswith('Reflow interrupted by keypress'):
			try:
				self.consumer.reflow_interrupted()
			except Exception as e:
				logger.error(e, exc_info=True)
			return

		if line.startswith('Selected profile'):
			profile = line[20:].strip()
			try:
				self.consumer.reflow_started(profile)
			except Exception as e:
				logger.error(e, exc_info=True)
			return

		try:
			status = self.parse(line)
		except ValueError as e:
			if len(line.strip()) > 0:
				print('!!', line)
			return

		try:
			self.consumer.status_received(status)
		except Exception as e:
			logger.error(e, exc_info=True)

	def parse(self, line):
		values = list(map(str.strip, line.split(',')))
		# Convert all values to float, except the mode
		values = list(map(float, values[0:-1])) + [values[-1], ]

		fields = FIELD_NAMES.split(',')
		if len(values) != len(fields):
			raise ValueError('Expected %d fields, found %d' % (len(fields), len(values)))

		return dict(zip(fields, values))

class TemperatureProfile:
	def __init__(self):
		self.points = []

	def add_point(self, t, temperature):
		self.points.append((t, temperature))

	def interpolate(self, t):
		index = self.find_index(t)
		length = len(self.points)

		if index == 0:
			return self.points[0][1]
		elif index == length:
			return self.points[length - 1][1]
		else:
			t0, temp0 = self.points[index - 1]
			t1, temp1 = self.points[index]

			f = (t - t0) / (t1 - t0)
			return f * (temp1 - temp0) + temp0

	def find_index(self, t):
		index = 0
		while index < len(self.points):
			if t < self.points[index][0]:
				return index
			index += 1
		return index

class ReflowData:
	def __init__(self, profile):
		self.profile = profile
		self.statuses = []

	def append_status(self, status):
		self.statuses.append(status)

	def values_for_key(self, key):
		return [s[key] for s in self.statuses]

	def time_values(self):
		return self.values_for_key('Time')

	def temp0_values(self):
		return self.values_for_key('Temp0')

	def temp1_values(self):
		return self.values_for_key('Temp1')

	def temp2_values(self):
		return self.values_for_key('Temp2')

	def temp3_values(self):
		return self.values_for_key('Temp3')

	def setpoint_values(self):
		return self.values_for_key('Set')

	def average_values(self):
		return self.values_for_key('Actual')

	def cold_junction_values(self):
		return self.values_for_key('ColdJ')

	def heat_values(self):
		return list(map(lambda x: x * (100.0 / 256.0), self.values_for_key('Heat')))

	def fan_values(self):
		return list(map(lambda x: x * (100.0 / 256.0), self.values_for_key('Fan')))

class EventConsumer:
	def __init__(self):
		self.reflow_data = ReflowData('Unknown')

	def reflow_started(self, profile):
		self.reflow_data = ReflowData(profile)
		return

	def reflow_finished(self):
		return

	def reflow_interrupted(self):
		return

	def profile_selected(self, profile):
		return

	def status_received(self, status):
		if self.reflow_data == None:
			self.reflow_data = ReflowData('Unknown')
		self.reflow_data.append_status(status)


class ReflowView:
	def __init__(self):
		self.time_limits = [0, 400]
		self.temp_limits = [0, 275]
		self.pwm_limits = [-5, 105]

		self.fig = plt.figure(figsize=(15, 12))
		self.fig.canvas.set_window_title('Reflow Profile')

		self.temp_top_axes = self.fig.add_subplot(7, 1, (1, 2))
		self.temp_pcb_axes = self.fig.add_subplot(7, 1, (3, 4))
		self.temp_pid_axes = self.fig.add_subplot(7, 1, (5, 6))
		self.pwm_axes = self.fig.add_subplot(7, 1, 7)

		# Temperature Sensors (Top)

		self.temp_top_sp_plot, = self.temp_top_axes.plot([], [], label='Set Point', 	linewidth=2, color='#000000', zorder=1)
		self.temp_top_t0_plot, = self.temp_top_axes.plot([], [], label='Air Sensor A', 	linewidth=1, color='#008000', zorder=2)
		self.temp_top_t1_plot, = self.temp_top_axes.plot([], [], label='Air Sensor B', 	linewidth=1, color='#0000d0', zorder=2)

		self.temp_top_axes.grid(True, color='#dddddd')
		self.temp_top_axes.set_xticklabels([])
		self.temp_top_axes.set_ylabel('Temperature [°C]')
		self.temp_top_axes.legend(loc='upper left')

		# Temperature Sensors (Aux)

		self.temp_pcb_sp_plot, = self.temp_pcb_axes.plot([], [], label='Set Point', 	linewidth=2, color='#000000', zorder=1)
		self.temp_pcb_t2_plot, = self.temp_pcb_axes.plot([], [], label='PCB Sensor A', 	linewidth=1, color='#008000', zorder=2)
		self.temp_pcb_t3_plot, = self.temp_pcb_axes.plot([], [], label='PCB Sensor B', 	linewidth=1, color='#0000d0', zorder=2)

		self.temp_pcb_axes.grid(True, color='#dddddd')
		self.temp_pcb_axes.set_xticklabels([])
		self.temp_pcb_axes.set_ylabel('Temperature [°C]')
		self.temp_pcb_axes.legend(loc='upper left')

		# Temperature PID Inputs

		self.temp_pid_sp_plot, = self.temp_pid_axes.plot([], [], label='Set Point', linewidth=2, color='#000000', zorder=1)
		self.temp_pid_in_plot, = self.temp_pid_axes.plot([], [], label='PID Input', linewidth=1, color='#800000', zorder=2)
		
		self.temp_pid_axes.grid(True, color='#dddddd')
		self.temp_pid_axes.set_xticklabels([])
		self.temp_pid_axes.set_ylabel('Temperature [°C]')		
		self.temp_pid_axes.legend(loc='upper left')

		# PWM Outputs

		self.heater_plot, = self.pwm_axes.plot([], [], label='Heater', 	linewidth=1, color='#ee9900')
		self.fan_plot, 	  = self.pwm_axes.plot([], [], label='Fan', linewidth=1, color='#4444ff')
		
		self.pwm_axes.grid(True, color='#dddddd')
		self.pwm_axes.set_ylabel('Duty Cycle [%]')
		self.pwm_axes.set_xlabel('Time [seconds]')		
		self.pwm_axes.legend(loc='upper left')

		self.fig.canvas.toolbar.pack_forget()
		self.fig.tight_layout()



		self.profile = TemperatureProfile()
		self.profile.add_point(30, 25)
		self.profile.add_point(90, 150)
		self.profile.add_point(150, 180)
		self.profile.add_point(175, 230)
		self.profile.add_point(200, 230)
		self.profile.add_point(300, 25)


	def update(self, reflow_data):
		# Profile data

		ptimes = list(range(0, 400, 1))
		ptemps = [self.profile.interpolate(t) for t in ptimes]

		# Retrieve data points

		time_values = reflow_data.time_values()
		temp0_values = reflow_data.temp0_values()
		temp1_values = reflow_data.temp1_values()
		temp2_values = reflow_data.temp2_values()
		temp3_values = reflow_data.temp3_values()

		setpoint_values = reflow_data.setpoint_values()
		average_values = reflow_data.average_values()

		heater_values = reflow_data.heat_values()
		fan_values = reflow_data.fan_values()

		# Update axes limits

		self._update_limits_from_values(self.time_limits, time_values,
														  ptimes)


		self._update_limits_from_values(self.temp_limits, temp0_values,
														  temp1_values,
														  temp2_values,
														  temp3_values,
														  setpoint_values,
		 												  average_values,
		 												  ptemps)

		self._update_limits_from_values(self.pwm_limits,  heater_values,
														  fan_values)

		self._set_axes_limits()

		self.temp_top_t0_plot.set_data(time_values, temp0_values)
		self.temp_top_t1_plot.set_data(time_values, temp1_values)
		self.temp_top_sp_plot.set_data(time_values, setpoint_values)

		self.temp_pcb_t2_plot.set_data(time_values, temp2_values)
		self.temp_pcb_t3_plot.set_data(time_values, temp3_values)
		self.temp_pcb_sp_plot.set_data(ptimes, ptemps)
		
		self.temp_pid_in_plot.set_data(time_values, average_values)
		self.temp_pid_sp_plot.set_data(time_values, setpoint_values)

		self.heater_plot.set_data(time_values, heater_values)
		self.fan_plot.set_data(time_values, fan_values)

		plt.draw()

	def run_event_loop(self, interval):
		# The following code replaces the common pyplot idoim `pause()`.
		# Although the use of `pause(...)` is the standard technique for
		# writing non-blocking pyplot graphics, each time it's invoked,
		# it steals focus from the currently active window and moves
		# itself to the top of the window stack. It isn't ideal... 
		# 
		# Replacing an officially maintained solution with a hack, such
		# as the one below, has serious drawbacks. However, for now, it
		# works with the current versions and the alternative is janky.  
		backend = plt.rcParams['backend']
		if backend in matplotlib.rcsetup.interactive_bk:
			figManager = matplotlib._pylab_helpers.Gcf.get_active()
			if figManager is not None:
				canvas = figManager.canvas
				if canvas.figure.stale:
					canvas.draw()
				canvas.start_event_loop(interval)
				return		

	def _update_limits_from_values(self, limits, *values_list):
		for values in values_list:
			if len(values) == 0:
				continue

			min_value = min(values)
			max_value = max(values)

			if limits[0] > min_value:
				limits[0] = min_value
			if limits[1] < max_value:
				limits[1] = max_value

	def _set_axes_limits(self):
		self.temp_top_axes.set_xlim(*self.time_limits)
		self.temp_top_axes.set_ylim(*self.temp_limits)

		self.temp_pcb_axes.set_xlim(*self.time_limits)
		self.temp_pcb_axes.set_ylim(*self.temp_limits)

		self.temp_pid_axes.set_xlim(*self.time_limits)
		self.temp_pid_axes.set_ylim(*self.temp_limits)

		self.pwm_axes.set_xlim(*self.time_limits)
		self.pwm_axes.set_ylim(*self.pwm_limits)


def main():

	air_profile = TemperatureProfile()
	air_profile.add_point(30, 25)
	air_profile.add_point(90, 150)
	air_profile.add_point(150, 180)
	air_profile.add_point(175, 230)
	air_profile.add_point(200, 230)
	air_profile.add_point(300, 25)

	aptemps = [air_profile.interpolate(t) for t in range(0, 300, 10)]

	consumer = EventConsumer()
	reflow_view = ReflowView()

	plt.ion();
	plt.show()

	if False:
		reflow_view.update(consumer.reflow_data)
		while True:
			reflow_view.run_event_loop(1)

	port = get_tty()

	def build_connection():
		return T962Connection(consumer)

	try:
		with serial.threaded.ReaderThread(port, build_connection) as conn:
			try:
				conn.send_command("info")

				#conn.set_profile(5, aptemps)
				#conn.send_command("dump profile 5")

				# conn.send_command("select profile 5")
				# conn.send_command("reflow")

				while True:
					if consumer.reflow_data != None:
						reflow_view.update(consumer.reflow_data)
					reflow_view.run_event_loop(1)
			finally:
				conn.send_command("stop")
	except KeyboardInterrupt:
		pass 

if __name__ == '__main__':
	main()
