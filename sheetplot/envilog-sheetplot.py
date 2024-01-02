#!/usr/bin/python3

#alnwlsn 2024
#program for plotting data dump of enviromental logger
#makes some line graphs which fit onto a US letter sized paper

import pandas as pd 
import matplotlib.pyplot as plt
import matplotlib.dates as mdates 
import csv
import numpy as np

#inputs
input_dump_filename = 'envilog-time-capsule-2018.txt'
chart_title = 'Conditions inside the Winter Camp XVII (2018) Time Capsule'
start_skip = 177 #skip this many samples at the beginning

chart_linewidth = .5
chart_fontsize = 8
#######

base_filename = input_dump_filename[:input_dump_filename.rfind('.')]

#conversion to CSV file
csv_filename = base_filename+'.csv'
with open(input_dump_filename, 'r') as input_file:
    lines = input_file.readlines()
header_index = None
footer_index = 0
for i, line in enumerate(lines):
    if line.startswith('Index'): #header of csv starts with 'Index'
        header_index = i
    if 'nan' in line:
        footer_index = i-header_index-1
        break
header = lines[header_index].strip().split('\t')
data_lines = [line.strip().split('\t') for line in lines[header_index + 1:]]
data_lines = data_lines[start_skip:footer_index]
with open(csv_filename, 'w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(header)
    csv_writer.writerows(data_lines)

#read data back in
df = pd.read_csv(csv_filename)
df['Year'] = df['Year'] + 2000 #needs 4 digit year
df['DateTime'] = pd.to_datetime(df[['Year', 'Month', 'Day', 'Hour', 'Minute', 'Second']])

#unit conversions
df['Temperature (F)'] = df['Temperature (C)']*1.8+32
df['DS3231 Temperature (F)'] = df['DS3231 Temperature (C)']*1.8+32
df['Pressure (inHg)'] = df['Pressure (Pa)'] * 0.00029529983071445 

#define plot area
fig, axs = plt.subplots(4, 1, sharex=True, figsize=(8.5, 11), dpi=600)
fig.subplots_adjust(left=0.08, right=0.98, bottom=0.07, top=0.97)
fig.subplots_adjust(hspace=0) # Remove vertical space between axes
axs[0].set_title(chart_title, size=chart_fontsize+2) 

#plot each section
axs[0].plot(df['DateTime'],df['Temperature (F)'], linewidth=chart_linewidth)
axs[0].set_ylabel('Temperature °F (BME280)', size=chart_fontsize)
yl = range(int(min(df['Temperature (F)'])/10)*10, int(max(df['Temperature (F)'])/10)*10+1, 5)
axs[0].set_yticks(yl)
axs[0].set_yticklabels(yl, size=chart_fontsize)
axs[0].grid(linewidth=chart_linewidth)
axs[0].axhline(y=32, color='r', linestyle='--', linewidth=chart_linewidth)


axs[1].plot(df['DateTime'],df['DS3231 Temperature (F)'], linewidth=chart_linewidth)
axs[1].set_ylabel('Temperature °F (DS3232M)', size=chart_fontsize)
axs[1].set_yticks(yl)
axs[1].set_yticklabels(yl, size=chart_fontsize)
axs[1].grid(linewidth=chart_linewidth)
axs[1].axhline(y=32, color='r', linestyle='--', linewidth=chart_linewidth)

axs[2].plot(df['DateTime'],df['Pressure (inHg)'], linewidth=chart_linewidth)
axs[2].set_ylabel('Pressure inHg (BME280)', size=chart_fontsize)
dv = .2
yl = range(int(min(df['Pressure (inHg)'])/dv), int(max(df['Pressure (inHg)'])/dv))
yl = [round(dv*y,2) for y in yl]
axs[2].set_yticks(yl)
axs[2].set_yticklabels(yl, size=chart_fontsize)
axs[2].grid(linewidth=chart_linewidth)

axs[3].plot(df['DateTime'],df['Humidity (%)'], linewidth=chart_linewidth)
axs[3].set_ylabel('% Rel. Humidity (BME280)', size=chart_fontsize)
dv = 4
yl = range(int(min(df['Humidity (%)'])/10)*10, int(max(df['Humidity (%)'])/10)*10+1, 5)
axs[3].set_yticks(yl)
axs[3].set_yticklabels(yl, size=chart_fontsize)
axs[3].grid(linewidth=chart_linewidth)

#adjust x axis ticks and labels and ticklables
loc = mdates.MonthLocator(bymonth=[1,4,7,10])
chart_xticks = loc.tick_values(min(df['DateTime']),max(df['DateTime']))
axs[3].set_xticks(chart_xticks)
axs[3].set_xticklabels(chart_xticks,  rotation=45, ha='right', size=chart_fontsize)
axs[3].xaxis.set_major_formatter(mdates.DateFormatter('%Y-%m'))
axs[3].set_xlabel('Date', size=chart_fontsize)

#save it
plt.savefig(base_filename+'.png')
plt.savefig(base_filename+'.pdf')