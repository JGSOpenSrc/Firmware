#! /usr/bin/python
from pylab import *
import os, sys

def useage():
    how_to_use = """
About: flightLogAnalyze produces plots of vehicle flight data from CSV log files.

How to use flightLogAnalyze.py:

python flightLogParse.py <csv log file>
Input: <csv log file> path to the comma separated value log file to analyze
                 """
    print(how_to_use)

if __name__ == "__main__":
    # Check for input file
    if len(sys.argv) < 2:
        useage()
    else: print(sys.argv[1])

    # Open the file for reading
    infile = open(sys.argv[1], 'r')

    lines = []
    for line in infile:
        lines.append(line)
    infile.close()

    # Get a list of headings and print them
    headings = lines[0].split(',')
    print(headings)

    # Split the lines into parts
    split_lines = []
    for line in lines[1:]:
        split_lines.append(line.split(','))

    # Get Vehicle attitude data
    indices = [headings.index('ATT_Roll'),headings.index('ATT_Pitch'), headings.index('ATT_Yaw'), headings.index('TIME_StartTime')]
    attitude = [[],[],[]]
    times = []
    for i in range(0,len(split_lines)):
        parts = split_lines[i]
        try:
            roll = float(parts[indices[0]])
            pitch = float(parts[indices[1]])
            yaw = float(parts[indices[2]])
            time = float(parts[indices[3]]) / 1E6
        except ValueError: continue
        attitude[0].append(roll)
        attitude[1].append(pitch)
        attitude[2].append(yaw)
        times.append(time)

    # Get attitude rates
    indices = [headings.index('ATT_RollRate'),headings.index('ATT_PitchRate'),headings.index('ATT_YawRate')]
    attitude_rate = [[],[],[]]
    for i in range(0,len(split_lines)):
        parts = split_lines[i]
        try:
            roll = float(parts[indices[0]])
            pitch = float(parts[indices[1]])
            yaw = float(parts[indices[2]])
        except ValueError: continue
        attitude_rate[0].append(roll)
        attitude_rate[1].append(pitch)
        attitude_rate[2].append(yaw)

    # Get local position data
    indices = [headings.index('LPOS_X'), headings.index('LPOS_Y'), headings.index('LPOS_Z')]
    lpos = [[],[],[]]
    for i in range(0,len(split_lines)):
        parts = split_lines[i]
        try:
            x = float(parts[indices[0]])
            y = float(parts[indices[1]])
            z = float(parts[indices[2]])
        except ValueError: continue
        lpos[0].append(x)
        lpos[1].append(y)
        lpos[2].append(z)

    # Get velocity data
    indices = [headings.index('LPOS_VX'), headings.index('LPOS_VY'), headings.index('LPOS_VZ')]
    vel = [[],[],[]]
    for i in range(0,len(split_lines)):
        parts = split_lines[i]
        try:
            vx = float(parts[indices[0]])
            vy = float(parts[indices[1]])
            vz = float(parts[indices[2]])
        except ValueError: continue
        vel[0].append(vx)
        vel[1].append(vy)
        vel[2].append(vz)

    # Get accelleration data
    indices = [headings.index('LPSP_AX'),headings.index('LPSP_AY'),headings.index('LPSP_AZ')]
    accel = [[],[],[]]
    for i in range(0,len(split_lines)):
        parts = split_lines[i]
        try:
            ax = float(parts[indices[0]])
            ay = float(parts[indices[1]])
            az = float(parts[indices[2]])
        except ValueError: continue
        accel[0].append(ax)
        accel[1].append(ay)
        accel[2].append(az)

    # Plot distance sensor data
    indices = [headings.index('DIST_Distance'), headings.index('LPOS_Dist')]
    distances = [[],[]]
    for i in range(0,len(split_lines)):
        parts = split_lines[i]
        try:
            distance = float(parts[indices[0]])
            lpos_distance = float(parts[indices[1]])
        except ValueError: continue
        distances[0].append(distance)
        distances[1].append(lpos_distance)

    # Get IMU data
    # Get optical flow data
    indices = [headings.index('FLOW_RX'),headings.index('FLOW_RY'),headings.index('FLOW_RZ'), headings.index('FLOW_Dist'), headings.index('FLOW_Qlty')]
    flow_data = [[],[],[],[],[]]
    for i in range(0,len(split_lines)):
        parts = split_lines[i]
        try:
            rx = float(parts[indices[0]])
            ry = float(parts[indices[1]])
            rz = float(parts[indices[2]])
            d = float(parts[indices[3]])
            q = float(parts[indices[4]])
        except ValueError: continue
        flow_data[0].append(rx)
        flow_data[1].append(ry)
        flow_data[2].append(rz)
        flow_data[3].append(d)
        flow_data[4].append(q)

    # Get Wind Estimate Data
    indices = [headings.index('WIND_X'), headings.index('WIND_Y')]
    wind_data = [[],[]]
    for i in range(0,len(split_lines)):
        parts = split_lines[i]
        try:
            wind_x = float(parts[indices[0]])
            wind_y = float(parts[indices[1]])
        except ValueError: continue
        wind_data[0].append(wind_x)
        wind_data[1].append(wind_y)

    # Figure 1: local position, velocity, and acceleration estimates
    figure(1)
    subplot(311)
    plot(times,accel[0])
    plot(times,accel[1])
    plot(times,accel[2])
    ylabel('Accelleration (meters per second squared)')
    xlabel('Time (seconds)')
    legend(['LPSP_AX', 'LPSP_AY', 'LPSP_AZ'])

    # Plot the local posiion data against time
    subplot(312)
    plot(times,vel[0])
    plot(times,vel[1])
    plot(times,vel[2])
    ylabel('Velocity (meters per second)')
    xlabel('Time (seconds)')
    legend(['LPOS_VX','LPOS_VY', 'LPOS_VZ'])

    # Plot velocity estimates against time
    subplot(313)

    plot(times,lpos[0])
    plot(times,lpos[1])
    plot(times,lpos[2])
    ylabel('Distance (meters)')
    xlabel('Time (seconds)')
    legend(['LPOS_X','LPOS_Y','LPOS_Z'])

    # Differentiate ultrasound sensor to get a feel for the spikes
    d = flow_data[3]
    diff = []
    for i in range(len(d)-1):
        diff.append(d[i+1]-d[i])

    # Attitudes, attitude rate, and accelleration
    figure(2)
    subplot(311)
    plot(times,attitude[0])
    plot(times,attitude[1])
    plot(times,attitude[2])
    ylabel('Angle (radians)')
    xlabel('Time (seconds)')
    legend(['roll','pitch','yaw'])

    subplot(312)
    plot(times,attitude_rate[0])
    plot(times,attitude_rate[1])
    plot(times,attitude_rate[2])
    ylabel('Angle rate (radians / second)')
    xlabel('Time (seconds)')
    legend(['roll rate','pitch rate','yaw rate'])

    # Optical Flow Data
    figure(3)
    subplot(211)
    plot(times,flow_data[0])
    plot(times,flow_data[1])
    plot(times,flow_data[2])
    ylabel('Flow integral')
    xlabel('Time (seconds)')
    legend(['RX','RY','RZ'])

    subplot(212)
    plot(times,flow_data[3])
    plot(times,flow_data[4])
    xlabel('Time (seconds)')
    legend(['D', 'Q'])

    # Miscellaneous Figure
    figure(4)

    # Spike analysis in ultrasound data
    subplot(311)
    diff = asarray(diff)
    dist = asarray(flow_data[3][:-1])
    plot(fabs(diff)/dist)
    plot(dist)
    ylabel('Distance (meters)')
    xlabel('Time (seconds)')
    legend(['Normalized difference', 'Measured distance'])

    # Plot of wind data against time
    subplot(312)
    plot(times,distances[0])
    plot(times,distances[1])
    ylabel('Distance (meters)')
    xlabel('Time (seconds)')
    legend(['Distance Sensor','Distance Estimate'])

    # Plot distance sensor data
    subplot(313)
    plot(times, wind_data[0])
    plot(times, wind_data[1])
    ylabel('?')
    xlabel('Time (seconds)')
    legend(['X Wind Component', 'Y Wind Component'])
    show()
