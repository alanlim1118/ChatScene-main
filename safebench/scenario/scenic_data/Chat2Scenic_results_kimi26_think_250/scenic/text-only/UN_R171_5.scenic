description = "Ego vehicle waits for a parallel car to clear the blind spot before changing lanes."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

egoLane = Uniform(*[lane for lane in network.lanes if lane.adjacentLanes])
advLane = Uniform(*egoLane.adjacentLanes)
startDist = Range(20, 100)
blindSpotOffset = Range(2, 5)
egoSpawnPt = new OrientedPoint following egoLane.orientation from egoLane.centerline.start for startDist
advSpawnPt = new OrientedPoint following advLane.orientation from advLane.centerline.start for startDist - blindSpotOffset

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE = Range(0.3, 0.6)
param BLIND_SPOT_DIST = Range(4, 7)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until withinDistanceToAnyCars(self, globalParameters.BLIND_SPOT_DIST)
	while withinDistanceToAnyCars(self, globalParameters.BLIND_SPOT_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)
	targetLaneSec = self.laneSection.laneToRight
	do LaneChangeBehavior(laneSectionToSwitch=targetLaneSec, target_speed=globalParameters.EGO_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with behavior AdversaryBehavior()

require (distance from ego to adversary) < 10
terminate when not (ego in egoLane)