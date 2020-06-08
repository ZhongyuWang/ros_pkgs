#!/usr/bin/env python
import rosbag, sys, csv
import time
import string
import os

# Read all bag files in folder
bagfile_list = [f for f in os.listdir(".") if f[-4:] == ".bag"]
bagfile_num = len(bagfile_list)
print "reading all " + bagfile_num + " bagfiles in current directory: \n"
for f in bagfile_list:
	print f


# Get fieldnames from bagfile
count = 0
for bagfile in bagfile_list:
	count += 1
	print "writing file " + str(count) + " of  " + bagfile_num + ": " + bagfile

	# Read fieldnames
	fieldnames = ['rostimestamp']
	bag = rosbag.Bag(bagfile)
	bag_msg = bag.read_messages()
	for topic, msg, t in bag_msg:
		if not topic[0] == "/":
			topic = "/" + topic
		msg_str = str(msg)
		msg_list = string.split(msg_str, '\n')
		field_list = []
		for nameValuePair in msg_list:
			splitPair = string.split(nameValuePair, ':')
			if (len(splitPair[0]) - len(splitPair[0].lstrip(' '))) / 2 >= len(field_list):
				field_list.append(splitPair[0].lstrip(' '))
			else:
				field_list[(len(splitPair[0]) - len(splitPair[0].lstrip(' '))) / 2] = splitPair[0].lstrip(' ')
			if len(splitPair) > 1:
				if splitPair[1] == " ":
					continue
				idx = (len(splitPair[0]) - len(splitPair[0].lstrip(' '))) / 2
				field = topic + ":" + ":".join(field_list[0:idx+1])
				if field not in fieldnames:
					fieldnames.append(field)
	print fieldnames

	# Write bagfile to csv
	with open(bagfile[:-4]+'.csv', 'w+') as csvfile:
		filewriter = csv.DictWriter(csvfile, fieldnames=fieldnames)
		filewriter.writeheader()
		bag = rosbag.Bag(bagfile)
		bag_msg = bag.read_messages()
		for topic, msg, t in bag_msg:
			if not topic[0] == "/":
				topic = "/" + topic
			msg_str = str(msg)
			msg_list = string.split(msg_str, '\n')
			# Write topic to CSV
			bag_dict = {"rostimestamp":str(t)}
			for nameValuePair in msg_list:
				splitPair = string.split(nameValuePair, ':')
				if (len(splitPair[0]) - len(splitPair[0].lstrip(' '))) / 2 >= len(field_list):
					field_list.append(splitPair[0].lstrip(' '))
				else:
					field_list[(len(splitPair[0]) - len(splitPair[0].lstrip(' '))) / 2] = splitPair[0].lstrip(' ')
				if len(splitPair) > 1:
					if splitPair[1] == " ":
						continue
					idx = (len(splitPair[0]) - len(splitPair[0].lstrip(' '))) / 2
					field = topic + ":" + ":".join(field_list[0:idx+1])
					if field in fieldnames:
						bag_dict[field] = splitPair[1]
			filewriter.writerow(bag_dict)
	bag.close()
print "Done reading all " + bagfile_num + " bag files."
