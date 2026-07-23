description = "Ego vehicle performs an emergency evasive lane change to avoid a braking lead vehicle and an oncoming SUV, resulting in a side-swipe with oncoming traffic."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*network.intersections)
egoInitLane = Uniform(*filter(lambda lane: all([sec._laneToLeft is not None and sec._laneToRight is not None for sec in lane.sections]) and lane.group is lane.road.forwardLanes and lane.road.backwardLanes is not None, intersection.incomingLanes))
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
leadSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(15, 30)
egoSection = egoInitLane.sectionAt(egoSpawnPt)
leftLaneSec = egoSection.laneToLeft
leftInitLane = leftLaneSec.lane
leftSpawnPt = new OrientedPoint in leftInitLane.centerline
advInitLane = Uniform(*egoInitLane.road.backwardLanes.lanes)
advSpawnPt = new OrientedPoint in advInitLane.centerline

param EGO_SPEED = Range(7, 10)
param EGO_EVASIVE_DIST = Range(10, 15)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until withinDistanceToObjsInLane(self, globalParameters.EGO_EVASIVE_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, is_oppositeTraffic=True, target_speed=globalParameters.EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED, is_oppositeTraffic=True)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_LEAD_SPEED = Range(4, 7)
param OPT_LEAD_BRAKE_TIME = Range(2, 5)

behavior LeadBrakeBehavior(speed, brake_time):
    do FollowLaneBehavior(target_speed=speed) for brake_time seconds
    take SetThrottleAction(0)
    while True:
        take SetBrakeAction(1)

LeadAgent = new Car at leadSpawnPt,
    with heading leadSpawnPt.heading,
    with behavior LeadBrakeBehavior(globalParameters.OPT_LEAD_SPEED, globalParameters.OPT_LEAD_BRAKE_TIME)

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt,
	with heading advSpawnPt.heading,
	with blueprint MODEL,
	with behavior AdversaryBehavior([advInitLane])

param FWD_SPEED = Range(7, 10)

behavior ForwardCarBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.FWD_SPEED, trajectory=trajectory)

forwardCar = new Car following advInitLane.centerline from advSpawnPt for Range(20, 40),
	with heading advSpawnPt.heading,
	with blueprint MODEL,
	with behavior ForwardCarBehavior([advInitLane])

param EGO_TERM_DIST = 10

require 30 <= (distance from egoSpawnPt to intersection) <= 50
require 20 <= (distance from advSpawnPt to intersection) <= 40

terminate when (ego intersects adversary) and ((distance from egoSpawnPt to ego) >= globalParameters.EGO_TERM_DIST)