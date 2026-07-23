description = "Ego vehicle executes a right lane change amidst surrounding traffic on a straight four-lane road."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

road = Uniform(*filter(lambda r: r.forwardLanes is not None and len(r.forwardLanes.lanes) >= 4, network.roads))
lane1 = road.forwardLanes.lanes[0]
lane2 = road.forwardLanes.lanes[1]
lane3 = road.forwardLanes.lanes[2]
lane4 = road.forwardLanes.lanes[3]
egoSpawnPt = new OrientedPoint in lane3.centerline
whiteSpawnPt = new OrientedPoint in lane4.centerline
yellowSpawnPt = new OrientedPoint in lane1.centerline
tealSpawnPt = new OrientedPoint in lane2.centerline
purpleSpawnPt = new OrientedPoint behind egoSpawnPt by 10
addCar1SpawnPt = new OrientedPoint following lane1.orientation from yellowSpawnPt for 25
addCar2SpawnPt = new OrientedPoint following lane2.orientation from tealSpawnPt for 25
buildingRefPt = new OrientedPoint in lane4.centerline
buildingSpawnPt = new OrientedPoint right of buildingRefPt by 6
egoTargetLane = lane4
yellowTargetLane = lane2

param OPT_EGO_SPEED = Range(8, 12)

behavior EgoBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=egoTargetLane.sections[0], target_speed=globalParameters.OPT_EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

advCar = new Car in lane1.centerline,
    facing lane1.orientation,
    with behavior FollowLaneBehavior()

param ADV_SPEED = Range(8, 12)

behavior LaneChangeAdvBehavior():
    targetLaneSec = self.laneSection.laneToRight
    do LaneChangeBehavior(
        laneSectionToSwitch=targetLaneSec,
        target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adv_lanechange = new Car in lane2.centerline,
    facing lane2.orientation,
    with behavior LaneChangeAdvBehavior()

adv_straight = new Car in lane4.centerline,
    facing lane4.orientation,
    with behavior FollowLaneBehavior()

adv_lane3 = new Car in lane3.centerline,
    facing lane3.orientation,
    with behavior FollowLaneBehavior()

adv_forward = new Car at addCar1SpawnPt,
    with behavior FollowLaneBehavior()

adv_addcar2 = new Car at addCar2SpawnPt,
    with behavior FollowLaneBehavior()

adv_straight_car = new Car in lane1.centerline,
    facing lane1.orientation,
    with behavior FollowLaneBehavior()

MIN_DIST = 5
TERM_DIST = 100

require (distance from ego to advCar) > MIN_DIST
require (distance from ego to adv_lanechange) > MIN_DIST
require (distance from ego to adv_straight) > MIN_DIST
require (distance from ego to adv_lane3) > MIN_DIST
require (distance from ego to adv_forward) > MIN_DIST
require (distance from ego to adv_addcar2) > MIN_DIST
require (distance from ego to adv_straight_car) > MIN_DIST

terminate when (ego in egoTargetLane) and ((distance to egoSpawnPt) > TERM_DIST)