#!/usr/bin/env python
features = {}
features['label1'] = ["entrance","exit"]
features['label2'] = ["admin office"]
features['label3'] = ["click and collect"]
features['label4'] = ["formal"]
features['label5'] = ["casual"]
features['label6'] = ["winter wear"]
features['label7'] = ["shoe"]
features['label8'] = ["kid"]
features['label9'] = ["new born"]
features['label10'] = ["restroom"]
features['label11'] = ["toy"]
features['label12'] = ["book", "magazine"]
features['label13'] = ["bottled drinks", "snack"]
features['label14'] = ["alcoholic drinks"]
features['label15'] = ["oil","sugar","salt","spice"]
features['label16'] = ["bread","confectionery"]
features['label17'] = ["egg","milk","dairy"]
features['label18'] = ["fruit"]
features['label19'] = ["vegetable"]
features['label20'] = ["meat"]
features['label21'] = ["flour","grain","pulse"]
features['label22'] = ["glass"]
features['label23'] = ["canned","instant"]
features['label24'] = ["detergent","clean"]
features['label25'] = ["seafood"]
features['label26'] = ["beauty","health"]
features['label27'] = ["gift"]
features['label28'] = ["pet"]
features['label29'] = ["seasonal"]
features['label30'] = ["sweet","chocolate"]
features['label31'] = ["billing"]

new_command = ""
directions = ""
class textreasoning(object):
	def voice_to_text(self, command):
		command = command.lower()
		global new_command
		global directions
		global features
		if "where" in command:
			for feature in features:
				for product in features[feature]:
					if product in command:
						return ["direction",feature] 
		elif "go" in command or "move" in command:
			for feature in features:
				for product in features[feature]:
					if product in command:
						return["command",feature]
		else:
			print("Command not recognised")
		new_command = ""
		directions = ""
		return

