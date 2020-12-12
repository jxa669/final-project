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
features['label10'] = ["toy"]
features['label11'] = ["book", "magazine"]
features['label12'] = ["bottled drinks", "snack"]
features['label13'] = ["alcoholic drinks"]
features['label14'] = ["oil","sugar","salt","spice"]
features['label15'] = ["bread","confectionery"]
features['label16'] = ["egg","milk","dairy"]
features['label17'] = ["fruit"]
features['label18'] = ["vegetable"]
features['label19'] = ["meat"]
features['label20'] = ["flour","grain","pulse"]
features['label21'] = ["glass"]
features['label22'] = ["canned","instant"]
features['label23'] = ["detergent","clean"]
features['label24'] = ["seafood"]
features['label25'] = ["beauty","health"]
features['label26'] = ["gift"]
features['label27'] = ["pet"]
features['label28'] = ["seasonal"]
features['label29'] = ["sweet","chocolate"]
features['label30'] = ["billing 1"]
features['label31'] = ["billing 2"]

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

