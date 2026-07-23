description = "Ego vehicle follows a lead vehicle through a T-junction at night as an oncoming adversary turns left and another adversary approaches from the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is3Way, network.intersections))

straightManeuvers = [m for m in intersection.maneuvers if m.type is ManeuverType.STRAIGHT]
egoManeuver = straightManeuvers[0]
oncomingManeuver = [m for m in straightManeuvers if m.startLane.road is not egoManeuver.startLane.road][0]

egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

leadSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(15, 25)

oncomingInitLane = oncomingManeuver.startLane
oncomingTrajectory = [oncomingInitLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane]
oncomingSpawnPt = new OrientedPoint in oncomingInitLane.centerline

straightStartLanes = [m.startLane for m in straightManeuvers]
crossIncomingLanes = [l for l in intersection.incomingLanes if l not in straightStartLanes]
rightAdvInitLane = crossIncomingLanes[0]
rightAdvManeuver = Uniform(*rightAdvInitLane.maneuvers)
rightAdvTrajectory = [rightAdvInitLane, rightAdvManeuver.connectingLane, rightAdvManeuver.endLane]
rightAdvSpawnPt = new OrientedPoint in rightAdvInitLane.centerline

param EGO_SPEED = Range(5, 8)
param EGO_BRAKE = Range(0.5, 1.0)
SAFE_DIST = 20

behavior EgoBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, SAFE_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(7, 10)

behavior OncomingAdvBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

oncomingAdversary = new Car at oncomingSpawnPt,
	with blueprint MODEL,
	with behavior OncomingAdvBehavior(oncomingTrajectory)

behavior RightAdvBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

rightAdversary = new Car at rightAdvSpawnPt,
	with blueprint MODEL,
	with behavior RightAdvBehavior(rightAdvTrajectory)

param LEAD_SPEED = Range(5, 8)

behavior LeadBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=trajectory)

leadVehicle = new Car at leadSpawnPt,
	with blueprint MODEL,
	with behavior LeadBehavior(egoTrajectory)

require 40 <= (distance from egoSpawnPt to intersection) <= 60
require 15 <= (distance from leadSpawnPt to intersection) <= 45
require 40 <= (distance from oncomingSpawnPt to intersection) <= 60
terminate when ego in egoManeuver.endLane