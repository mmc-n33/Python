""" 
CAR CONFIG 

This file is read by your car application's manage.py script to change the car
performance. 

EXMAPLE
-----------
import dk
cfg = dk.load_config(config_path='~/d3/config.py')
print(cfg.CAMERA_RESOLUTION)

"""


import os

#PATHS
CAR_PATH = PACKAGE_PATH = os.path.dirname(os.path.realpath(__file__))
DATA_PATH = os.path.join(CAR_PATH, 'data')
MODELS_PATH = os.path.join(CAR_PATH, 'models')

#VEHICLE
DRIVE_LOOP_HZ = 20
MAX_LOOPS = 100000

#CAMERA
CAMERA_TYPE = "PICAM"		# PICAM or WEBCAM
IMAGE_W = 160
IMAGE_H = 120
IMAGE_DEPTH = 3		# default RGB=3, make 1 for mono
			# CAMERA_RESOLUTION = (120, 160) #(height, width)
CAMERA_FRAMERATE = DRIVE_LOOP_HZ

#9865, over rides only if neeed, ie. TX2..
PCA9685_I2C_ADDR = 0x40
PCA9685_I2C_BUSNUM = None

#STEERING
STEERING_CHANNEL = 1
STEERING_LEFT_PWM = 260 #280 #310
STEERING_RIGHT_PWM = 450 #440 #450 #470 #490

#THROTTLE
THROTTLE_CHANNEL = 2
THROTTLE_FORWARD_PWM = 550 #575 #550
THROTTLE_STOPPED_PWM = 400
THROTTLE_REVERSE_PWM = 250
THROTTLE_RANGE = 20

#TRAINING
BATCH_SIZE = 128
TRAIN_TEST_SPLIT = 0.8
MAX_EPOCHS = 100
SHOW_PLOT = True
VEBOSE_TRAIN = True
USE_EARLY_STOP = True
EARLY_STOP_PATIENCE = 5
MIN_DELTA = .0005
PRINT_MODEL_SUMMARY = True	#print layers and weights to stdout
OPTIMIZER = None		#adam, sgd, rmsprop, etc.. None accepts default
LEARNING_RATE = 0.001		#only used when OPTIMIZER specified
LEARNING_RATE_DECAY = 0.0	#only used when OPTIMIZER specified

# model transfer options
FREEZE_LAYERS = False
NUM_LAST_LAYERS_TO_TRAIN = 7

#JOYSTICK
USE_JOYSTICK_AS_DEFAULT = True
JOYSTICK_MAX_THROTTLE = 0.3
JOYSTICK_STEERING_SCALE = 1.0
AUTO_RECORD_ON_THROTTLE = True

#RNN or 3D
SEQUENCE_LENGTH = 3

#IMU
HAVE_IMU = False

#LED
HAVE_RGB_LED = False
LED_INVERT = False		#COMMON ANODE?

#board pin number for pwm outputs
LED_PIN_R = 12
LED_PIN_G = 10
LED_PIN_B = 16

#LED status color, 0-100
LED_R = 0
LED_G = 0
LED_B = 1

#BEHAVIORS
TRAIN_BEHAVIORS = False
BEHAVIOR_LIST = ['Left_Lane', "Right_Lane"]
BEHAVIOR_LED_COLORS = [ (0, 10, 0), (10, 0, 0) ]  #RGB tuples 0-100 per chanel
