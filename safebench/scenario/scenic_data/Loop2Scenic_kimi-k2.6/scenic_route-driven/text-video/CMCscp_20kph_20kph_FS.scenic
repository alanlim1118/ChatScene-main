description = "The ego vehicle is turning right at an intersection; the adversarial motorcyclist on the left of the target lane suddenly crosses the road and comes to a halt in the center road."

Town = 'Town05'
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_PARAM_OFFSET = 17

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN and m.intersection is not None and (m.intersection.is4Way or m.intersection.is3Way), egoInitLane.maneuvers))
intersection = egoManeuver.intersection
egoTrajectoryLine = egoManeuver.startLane.centerline + egoManeuver.connectingLane.centerline + egoManeuver.endLane.centerline

endLanePt = new OrientedPoint at egoManeuver.endLane.centerline.start,
	with heading egoInitLane.centerline.end.heading - 180 deg
motorcycleSpawnPt = new OrientedPoint ahead of endLanePt by - globalParameters.OPT_PARAM_OFFSET

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn None,
	with blueprint EGO_MODEL

param OPT_ADV_SPEED = Range(1, 5)
param OPT_ADV_DISTANCE = Range(25, 35)
OPT_STOP_DISTANCE = 1

behavior CrossAndStopBehavior(actor_reference, adv_speed, adv_distance, stop_reference, stop_distance):
	do CrossingBehavior(actor_reference, adv_speed, adv_distance) until (distance from self to stop_reference <= stop_distance)
	take SetBrakeAction(1)
	take SetThrottleAction(0)

AdvAgent = new Motorcycle at motorcycleSpawnPt,
	with heading motorcycleSpawnPt.heading,
	with regionContainedIn None,
	with behavior CrossAndStopBehavior(ego,globalParameters.OPT_ADV_SPEED,globalParameters.OPT_ADV_DISTANCE,egoTrajectoryLine, OPT_STOP_DISTANCE)

require 40 <= (distance to intersection) <= 60

MODEL = 'vehicle.lincoln.mkz_2017'

param weather = 'ClearNoon'
