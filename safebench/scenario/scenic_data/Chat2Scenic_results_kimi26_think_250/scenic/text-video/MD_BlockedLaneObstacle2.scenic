description = "Blue ego vehicle stops behind a braking yellow truck at an intersection while adjacent traffic passes in the right lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*network.intersections)
egoLaneSec = Uniform(*filter(lambda s: s._laneToLeft is None and s._laneToRight is not None, [lane.sections[-1] for lane in intersection.incomingLanes]))
rightLaneSec = egoLaneSec._laneToRight
egoInitLane = egoLaneSec.lane
rightInitLane = rightLaneSec.lane
truckSpawnPt = new OrientedPoint in egoLaneSec.centerline
egoSpawnPt = new OrientedPoint behind truckSpawnPt by Range(5, 10)
car1SpawnPt = new OrientedPoint in rightLaneSec.centerline
car2SpawnPt = new OrientedPoint behind car1SpawnPt by Range(10, 20)

param EGO_SPEED = Range(5, 10)
param EGO_BRAKE_DIST = Range(8, 12)
param EGO_BRAKE = Range(0.5, 1.0)

behavior EgoBehavior():
	try:
		do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
	interrupt when withinDistanceToObjsInLane(self, globalParameters.EGO_BRAKE_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param TRUCK_SPEED = Range(5, 8)
param TRUCK_TIME = Range(3, 5)

behavior TruckBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED) for globalParameters.TRUCK_TIME seconds
	while True:
		take SetBrakeAction(1.0)

truck = new Truck at truckSpawnPt,
	with behavior TruckBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at car1SpawnPt,
	with behavior AdversaryBehavior()

param CAR2_SPEED = Range(7, 10)

behavior Car2Behavior():
	do FollowLaneBehavior(target_speed=globalParameters.CAR2_SPEED)

car2 = new Car at car2SpawnPt,
	with behavior Car2Behavior()

require 5 <= (distance from ego to truck) <= 10
require 5 <= (distance from ego to intersection) <= 60
terminate when (distance from ego to truck) < 4 and not (adversary in intersection) and not (car2 in intersection)