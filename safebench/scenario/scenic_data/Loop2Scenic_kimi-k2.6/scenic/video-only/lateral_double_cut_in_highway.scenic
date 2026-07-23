description = "Ego vehicle avoids collision with an emergency-braking leading vehicle on a highway."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param DISTANCE_BEHIND = Range(20, 30)

forwardSections = []
for lane in network.lanes:
	for section in lane.sections:
		if section.isForward:
			forwardSections.append(section)

targetSection = Uniform(*forwardSections)
advSpawnPt = new OrientedPoint in targetSection.centerline
egoSpawnPt = new OrientedPoint behind advSpawnPt by globalParameters.DISTANCE_BEHIND

param OPT_EGO_SPEED = Range(15, 20)
param OPT_BRAKE_DISTANCE = Range(10, 15)

behavior EgoBehavior(target_speed, avoidance_threshold):
	try:
		do FollowLaneBehavior(target_speed=target_speed)
	interrupt when withinDistanceToObjsInLane(self, avoidance_threshold):
		take SetThrottleAction(0)
		take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL,
	with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DISTANCE)

param OPT_ADV_SPEED = Range(15, 20)
param OPT_ADV_DRIVE_TIME = Range(4, 8)

behavior AdversarialBehavior(speed, drive_time):
	do FollowLaneBehavior(target_speed=speed) for drive_time seconds
	while True:
		take SetThrottleAction(0)
		take SetBrakeAction(1)

adv = new Car at advSpawnPt,
	with blueprint MODEL,
	with regionContainedIn None,
	with behavior AdversarialBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DRIVE_TIME)

TERM_DIST = 100

require 20 <= (distance from egoSpawnPt to advSpawnPt) <= 30
terminate when (distance to egoSpawnPt) > TERM_DIST