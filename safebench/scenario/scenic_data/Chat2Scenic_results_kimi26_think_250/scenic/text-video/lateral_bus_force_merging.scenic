description = "Coach bus overtakes ego with minimal side clearance and cuts in closely ahead on a multi-lane highway."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*filter(lambda lane: all([sec._laneToLeft is not None and sec._laneToRight is not None for sec in lane.sections]), network.lanes))
egoSpawnPt = new OrientedPoint in initLane.centerline
rightLane = initLane.sectionAt(egoSpawnPt).laneToRight.lane
carSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for Range(10, 30)
advSpawnPt = new OrientedPoint in rightLane.centerline

param EGO_SPEED = Range(10, 15)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

param TRUCK_SPEED = Range(16, 22)
param CUT_IN_DIST = Range(10, 20)

behavior TruckBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED) until (distance from self to ego) < globalParameters.CUT_IN_DIST
	do LaneChangeBehavior(laneSectionToSwitch=self.laneSection.fasterLane, target_speed=globalParameters.TRUCK_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.TRUCK_SPEED)

truck = new Truck at advSpawnPt,
	with heading advSpawnPt.heading,
	with behavior TruckBehavior()