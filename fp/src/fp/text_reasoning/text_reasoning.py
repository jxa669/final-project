#!/usr/bin/env python
features = ["Japan","China","Russia"]
new_command = ""
class textreasoning(object):
	def update_command(self, command):
		global new_command
		if "stop" in command:
			new_command = "stop"
		elif "rotate" in command:
			if "left" in command:
				new_command = "rotate left"
			elif "right" in command:
				new_command = "rotate right"
			elif "backward" in command:
				new_command = "rotate backward"
			else:
				print("Please specify which way to rotate")
		elif "go" in command or "move" in command:
			for feature in features:
				if feature in command:
					if "from" in command:
						if "to " + feature in command:
							new_command = "move to " + feature
					else:
						new_command = "move to " + feature
					
			#rotate and move if left or right
			if "left" in command:
				new_command = "move left"
			elif "right" in command:
				new_command = "move right"
			elif "forward" in command:
				new_command = "move forward"
			elif "backward" in command:
				new_command = "move backward"
		else:
			print("Command not recognised")
		if new_command != "":
			#pass command to planning component
			print(new_command)
		new_command = ""

