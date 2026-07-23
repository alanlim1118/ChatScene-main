description = "Ego vehicle traveling at 50 km/h strikes a pedestrian crossing perpendicularly from the left."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is4Way and not m.intersection.isSignalized, egoInitLane.maneuvers))
intersection = egoManeuver.intersection
advRefPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(20, 40)
advSpawnPt = new OrientedPoint left of advRefPt by Range(3, 5), facing toward advRefPt

ego = new Car at egoSpawnPt,
	with blueprint MODEL

param OPT_ADV_PED_SPEED = Range(3, 5)

behavior PedestrianBehavior():
	do CrossingBehavior(ego, min_speed=globalParameters.OPT_ADV_PED_SPEED, threshold=25)

adversarial = new Pedestrian at advSpawnPt,
	with regionContainedIn None,
	with behavior PedestrianBehavior()

terminate when ego intersects adversarial
