#!/usr/bin/env python
features = {}
features['aisle 1'] = ["entrance"]
features['aisle 2'] = ["admin office"]
features['aisle 3'] = ["click and collect"]
features['aisle 4'] = ["formal"]
features['aisle 5'] = ["casual"]
features['aisle 6'] = ["winter wear"]
features['aisle 7'] = ["shoe"]
features['aisle 8'] = ["kid"]
features['aisle 9'] = ["new born"]
features['aisle 10'] = ["toy"]
features['aisle 11'] = ["book", "magazine"]
features['aisle 12'] = ["bottled drinks", "snack"]
features['aisle 13'] = ["alcoholic drinks"]
features['aisle 14'] = ["oil","sugar","salt","spice"]
features['aisle 15'] = ["bread","confectionery"]
features['aisle 16'] = ["egg","milk","dairy"]
features['aisle 17'] = ["fruit"]
features['aisle 18'] = ["vegetable"]
features['aisle 19'] = ["meat"]
features['aisle 20'] = ["flour","grain","pulse"]
features['aisle 21'] = ["glass"]
features['aisle 22'] = ["canned","instant"]
features['aisle 23'] = ["detergent","clean"]
features['aisle 24'] = ["seafood"]
features['aisle 25'] = ["beauty","health"]
features['aisle 26'] = ["gift"]
features['aisle 27'] = ["pet"]
features['aisle 28'] = ["seasonal"]
features['aisle 29'] = ["sweet","chocolate"]
features['aisle 30'] = ["billing 1"]
features['aisle 31'] = ["billing 2"]

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
				for product in features[feature]:
					if product in command:
						command = "move to " + product + " in " + feature 
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

