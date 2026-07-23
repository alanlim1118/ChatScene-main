description = "Ego vehicle collides with a pedestrian emerging from behind an oncoming truck on a narrow rural road."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_GEO_TRUCK_DISTANCE = Range(20, 35)
param OPT_GEO_SUV_DISTANCE = Range(10, 20)
param OPT_GEO_PED_DISTANCE = Range(3, 6)

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

truckBasePt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for globalParameters.OPT_GEO_TRUCK_DISTANCE
truckSpawnPt = new OrientedPoint left of truckBasePt by Range(3.5, 4.5), facing toward egoSpawnPt

suvBasePt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for globalParameters.OPT_GEO_SUV_DISTANCE
suvSpawnPt = new OrientedPoint left of suvBasePt by Range(5, 7)

pedSpawnPt = new OrientedPoint behind truckSpawnPt by globalParameters.OPT_GEO_PED_DISTANCE

param OPT_EGO_SPEED = Range(7, 10)
param OPT_EGO_BRAKE_DISTANCE = Range(5, 8)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.OPT_EGO_BRAKE_DISTANCE):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        abort

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param OPT_TRUCK_SPEED = Range(8, 12)

truck = new Truck at truckSpawnPt,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_SPEED)

PED_MIN_SPEED = 1.0
PED_THRESHOLD = 20

behavior PedestrianBehavior():
    do CrossingBehavior(ego, PED_MIN_SPEED, PED_THRESHOLD)

ped = new Pedestrian at pedSpawnPt,
    facing -90 deg relative to ego.heading,
    with regionContainedIn None,
    with behavior PedestrianBehavior()

require 20 <= (distance from ego to truck) <= 36
require 23 <= (distance from ego to ped) <= 42
terminate when ego intersects ped