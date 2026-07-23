description = "Ego vehicle approaches a fallen shared bicycle and a tilted stationary vehicle encroaching into its lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

twoLaneRoads = []
for road in network.roads:
    if road.forwardLanes is not None:
        if len(road.forwardLanes.lanes) >= 2:
            twoLaneRoads.append(road)

egoRoad = Uniform(*twoLaneRoads)
egoLane = egoRoad.forwardLanes.lanes[0]
bikeLane = egoRoad.forwardLanes.lanes[1]

egoSpawnPt = new OrientedPoint in egoLane.centerline

param OPT_BIKE_AHEAD = Range(10, 20)
param OPT_ADV_AHEAD = Range(10, 20)
param OPT_LANE_OFFSET = Range(3.5, 4.5)
param OPT_ENCROACH = Range(1.0, 2.0)

bikeLateralPt = new OrientedPoint right of egoSpawnPt by globalParameters.OPT_LANE_OFFSET
bikeSpawnPt = new OrientedPoint ahead of bikeLateralPt by globalParameters.OPT_BIKE_AHEAD

advAheadPt = new OrientedPoint ahead of bikeSpawnPt by globalParameters.OPT_ADV_AHEAD
advSpawnPt = new OrientedPoint left of advAheadPt by globalParameters.OPT_ENCROACH

param OPT_EGO_SPEED = Range(8, 12)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

behavior BikeBehavior():
	while True:
		wait

bicycle = new Bicycle at bikeSpawnPt,
	with behavior BikeBehavior()

behavior AdversaryBehavior():
	while True:
		wait

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

TERM_DIST = 60

require 10 <= (distance from ego to bicycle) <= 21
require 10 <= (distance from bicycle to adversary) <= 21
terminate when (distance to egoSpawnPt) > TERM_DIST