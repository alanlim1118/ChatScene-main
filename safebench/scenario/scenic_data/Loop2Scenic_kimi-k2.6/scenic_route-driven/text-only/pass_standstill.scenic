description = "Using map ../../maps/Town03.xodr with carla map Town03 and weather ClearNoon"
param map = localPath('../../maps/Town03.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)

egoInitLane = network.laneAt(egoSpawnPt.position)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT and m.intersection is not None and m.intersection.is4Way, egoInitLane.maneuvers))
intersection = egoManeuver.intersection

advSpawnPt = new OrientedPoint following egoInitLane.orientation from egoSpawnPt for Range(15, 25)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with blueprint MODEL

behavior StationaryBehavior():
    while True:
        wait

gnome = new Pedestrian at advSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with behavior StationaryBehavior()

param OPT_INIT_DIST = Range(15, 25)
param OPT_TERM_DIST = Range(30, 50)

require (distance from egoSpawnPt to advSpawnPt) >= 15
terminate when (distance from ego to advSpawnPt) > 50