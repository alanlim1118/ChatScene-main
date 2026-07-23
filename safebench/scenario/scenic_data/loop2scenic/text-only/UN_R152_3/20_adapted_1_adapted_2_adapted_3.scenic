description = "No header settings provided"
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*filter(lambda lane:
    all([sec._laneToLeft is not None and sec._laneToLeft.isForward is not sec.isForward for sec in lane.sections]),
    network.lanes))

egoSpawnPt = new OrientedPoint on initLane.centerline

guardPipeSpawnPt = new OrientedPoint following initLane.orientation from egoSpawnPt for 30

stationaryVehicleSpawnPt = new OrientedPoint following initLane.orientation from guardPipeSpawnPt for 5

pedestrianSpawnPt = new OrientedPoint following initLane.orientation from stationaryVehicleSpawnPt for 5

bicycleSpawnPt = new OrientedPoint following initLane.orientation from pedestrianSpawnPt for 5

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with rolerole 'hero',
	with behavior EgoBehavior()

behavior StationaryBehavior():
    while True:
        wait

adversary = new Car at stationaryVehicleSpawnPt,
    with heading stationaryVehicleSpawnPt.heading,
    with regionContainedIn None,
    with behavior StationaryBehavior()

behavior StationaryPedestrianBehavior():
    while True:
        wait

pedestrian = new Pedestrian right of pedestrianSpawnPt by 2,
    facing pedestrianSpawnPt.heading,
    with regionContainedIn None,
    with behavior StationaryPedestrianBehavior()

behavior StationaryBicycleBehavior():
    while True:
        wait

bicycle = new Bicycle at bicycleSpawnPt,
    with heading bicycleSpawnPt.heading,
    with regionContainedIn None,
    with behavior StationaryBicycleBehavior()

chainbarrierSpawnPt = new OrientedPoint following initLane.orientation from guardPipeSpawnPt for 15
chainbarrier = new Prop at chainbarrierSpawnPt,
    with blueprint 'static.prop.chainbarrier'

param INIT_DIST = 30
param TERM_DIST = 80

require (distance to intersection) > 30
require (distance from guardPipeSpawnPt to intersection) > 30

terminate when (distance to egoSpawnPt) > 80