description = "Ego vehicle overtakes a slowing grey sedan turning left across the lane, resulting in a side-impact collision before slowing and passing pedestrians."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'CloudyNoon'

param ADV_INIT_DIST = Range(15, 30)

road = Uniform(*filter(lambda r: r.forwardLanes is not None and r.backwardLanes is not None and len(r.forwardLanes.lanes) == len(r.backwardLanes.lanes) == 1, network.roads))
egoLane = road.forwardLanes.lanes[0]
egoSpawnPt = new OrientedPoint on egoLane.centerline
advSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.ADV_INIT_DIST
pedRefPt = new OrientedPoint following egoLane.orientation from advSpawnPt for Range(20, 30)
pedSpawnPt = new OrientedPoint right of pedRefPt by Range(3, 5)

param EGO_SPEED = Range(8, 12)
param EGO_SLOW_SPEED = Range(3, 5)
param EGO_OVERTAKE_DIST = Range(10, 20)

behavior EgoBehavior(speed, slow_speed, overtake_dist, lane_target):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToAnyCars(self, overtake_dist)
    do LaneChangeBehavior(laneSectionToSwitch=lane_target, is_oppositeTraffic=True, target_speed=speed)
    do FollowLaneBehavior(target_speed=slow_speed)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.EGO_SPEED,
        globalParameters.EGO_SLOW_SPEED,
        globalParameters.EGO_OVERTAKE_DIST,
        road.backwardLanes.lanes[0].sections[0]
    )

param PED_SPEED = Range(0.8, 1.4)

ped = new Pedestrian at pedSpawnPt,
    with behavior WalkForwardBehavior(globalParameters.PED_SPEED)

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 30
terminate when (distance from ego to pedSpawnPt) > 70
terminate after 30 seconds