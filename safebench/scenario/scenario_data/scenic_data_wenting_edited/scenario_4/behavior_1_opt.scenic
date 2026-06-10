description = "Ego bypasses a parked car by using the opposite lane, monitoring oncoming traffic, and then reacts to a sudden jaywalking pedestrian."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_GEO_BLOCKER_DISTANCE = Range(15, 25)
param OPT_GEO_PED_DISTANCE = Range(30, 40)
param OPT_GEO_SIDE_OFFSET = Range(3, 5)

laneSecsWithOpposite = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithOpposite.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithOpposite)
egoInitLane = egoLaneSec.lane
opposingLaneSec = egoLaneSec._laneToLeft
opposingLane = opposingLaneSec.lane

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

blockerSpawnPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for globalParameters.OPT_GEO_BLOCKER_DISTANCE

pedRefPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for globalParameters.OPT_GEO_PED_DISTANCE
pedSpawnPt = new OrientedPoint right of pedRefPt by globalParameters.OPT_GEO_SIDE_OFFSET

egoTrajectory = [egoInitLane]
bypassTrajectory = [opposingLane]

param OPT_EGO_SPEED = Range(5, 8)
param OPT_BYPASS_DIST = 15
param OPT_BRAKE_DIST = 10

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to blockerSpawnPt) < globalParameters.OPT_BYPASS_DIST
        do LaneChangeBehavior(laneSectionToSwitch=opposingLaneSec, is_oppositeTraffic=True, target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED, laneToFollow=opposingLane, is_oppositeTraffic=True)
    interrupt when withinDistanceToAnyPedestrians(self, globalParameters.OPT_BRAKE_DIST):
        take SetBrakeAction(1)
        terminate

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

Blocker = new Car at blockerSpawnPt,
    with heading blockerSpawnPt.heading,
    with regionContainedIn None

param OPT_PED_MIN_SPEED = 1.0
param OPT_PED_THRESHOLD = 20

behavior PedestrianBehavior():
    do CrossingBehavior(ego, globalParameters.OPT_PED_MIN_SPEED, globalParameters.OPT_PED_THRESHOLD)

ped = new Pedestrian at pedSpawnPt,
    facing -90 deg relative to ego.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior()

monitor TrafficLightManager():
    freezeTrafficLights()
    while True:
        if withinDistanceToTrafficLight(ego, 100):
            setClosestTrafficLightStatus(ego, "green")
        wait

require monitor TrafficLightManager()
require 15 <= (distance from egoSpawnPt to blockerSpawnPt) <= 25
require 30 <= (distance from egoSpawnPt to pedRefPt) <= 40
require 3 <= globalParameters.OPT_GEO_SIDE_OFFSET <= 5

terminate when (distance from ego to pedRefPt) > 20 and (distance from ego to egoSpawnPt) > 60
terminate after 60 seconds