#!/usr/bin/env python
features = {}
features['first lane'] = ["drink","snack"]
features['second lane'] = ["fruit","vegetable"]
features['third lane'] = ["bakery","desert"]
new_command = ""
directions = ""
class textreasoning(object):
	def update_command(self, command):
		command = command.lower()
		global new_command
		global directions
		global features
		if "stop" in command:
			new_command = "stop"
		elif "where" in command:
			for feature in features:
				for product in features[feature]:
					if product in command:
						directions = "direct to " + product + " in " + feature 
		elif "rotate" in command or "turn" in command:
			if "left" in command:
				new_command = "rotate left"
			elif "right" in command:
				new_command = "rotate right"
			elif "backward" in command:
				new_command = "rotate backward"
			else:
				print("Please specify which way to rotate")
		elif "go" in command or "move" in command:
			#rotate and move if left or right
			if "left" in command:
				new_command = "move left"
			elif "right" in command:
				new_command = "move right"
			elif "forward" in command:
				new_command = "move forward"
			elif "backward" in command:
				new_command = "move backward"
			for feature in features:
				if feature in command:
					if "from" in command:
						if "to " + feature in command:
							new_command = "move to " + feature
					else:
						new_command = "move to " + feature
		else:
			print("Command not recognised")
		if new_command != "":
			#pass command to planning component and reasoning component
			print(new_command)
		if directions != "":
			#pass directions to reasoning
			print(directions)
		new_command = ""
		directions = ""

