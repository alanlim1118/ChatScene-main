description = "Ego bypasses a parked car using the opposite lane, yielding to oncoming traffic, then reacts to an unexpected swerving motorcyclist."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param parkedCarDist = Range(15, 25)
param motoAheadDist = Range(45, 60)

laneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecs.append(laneSec)

egoLaneSec = Uniform(*laneSecs)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
roadDirection = egoLaneSec.orientation

parkedCarSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.parkedCarDist

oppSec = egoLaneSec._laneToLeft
tempPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.motoAheadDist
motoPos = oppSec.centerline.project(tempPt.position)
motoSpawnPt = new OrientedPoint at motoPos, facing oppSec.orientation[motoPos]

egoTrajectory = [egoLaneSec.lane]
motoTrajectory = [oppSec.lane]

param EGO_SPEED = Range(6, 10)
param BYPASS_DIST = 20
param BRAKE_DIST = 15

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory) until (distance from self to Blocker) < globalParameters.BYPASS_DIST
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec._laneToLeft, is_oppositeTraffic=True, target_speed=globalParameters.EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED, is_oppositeTraffic=True) until (distance from self to Blocker) > 10
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=False, target_speed=globalParameters.EGO_SPEED)
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)
    interrupt when (distance from self to AdvAgent) < globalParameters.BRAKE_DIST:
        take SetBrakeAction(1.0)
        terminate

behavior SwerveBehavior():
    try:
        do FollowTrajectoryBehavior(target_speed=10, trajectory=motoTrajectory) until (distance from self to ego) < 30
        take SetSteerAction(0.4) for 0.5 seconds
        take SetSteerAction(-0.4) for 0.5 seconds
        do FollowTrajectoryBehavior(target_speed=10, trajectory=motoTrajectory)
    interrupt when (distance from self to ego) < 5:
        take SetBrakeAction(1.0)
        terminate

Blocker = new Car at parkedCarSpawnPt

AdvAgent = new Car at motoSpawnPt,
    with blueprint 'vehicle.kawasaki.ninja',
    with behavior SwerveBehavior()

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

Blocker = new Car at parkedCarSpawnPt,
    with heading parkedCarSpawnPt.heading,
    with regionContainedIn None

param MOTO_SPEED = Range(8, 12)
param SWERVE_DIST = Range(25, 35)

behavior SwerveBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.MOTO_SPEED, is_oppositeTraffic=True) until (distance to ego) < globalParameters.SWERVE_DIST
        do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, is_oppositeTraffic=True, target_speed=globalParameters.MOTO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.MOTO_SPEED, is_oppositeTraffic=False)
    interrupt when (distance to ego) < 5:
        take SetBrakeAction(1.0)
        terminate

AdvAgent = new Motorcycle at motoSpawnPt,
    with behavior SwerveBehavior()

require 15 <= (distance to Blocker) <= 25
require 45 <= (distance to AdvAgent) <= 60
terminate when (distance to egoSpawnPt) > 80