from naoqi import ALProxy
motion = ALProxy("ALMotion", "192.168.105.15", 9559)
config_max = motion.getFootGaitConfig("Max")
config_min = motion.getFootGaitConfig("Min")
config_default = motion.getFootGaitConfig("Default")
print(config_max)
print(config_min)
print(config_default)
motion.setStiffnesses("Body", 0)
