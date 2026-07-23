description = "Ego vehicle loses control on snow and collides head-on with an oncoming sedan after a taxi cuts across its lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_TAXI_SEDAN_DIST = Range(10, 20)

candidateLaneSecs = []
for lane in network.lanes:
    if lane.sections and lane.maneuvers:
        lastSec = lane.sections[-1]
        if lastSec.isForward and lastSec._laneToLeft is not None:
            leftSec = lastSec._laneToLeft
            if not leftSec.isForward:
                leftLane = leftSec.lane
                hasLeft = False
                hasStraight = False
                for m in leftLane.maneuvers:
                    if m.type is ManeuverType.LEFT_TURN:
                        hasLeft = True
                    if m.type is ManeuverType.STRAIGHT:
                        hasStraight = True
                if hasLeft and hasStraight:
                    candidateLaneSecs.append(lastSec)

egoLaneSec = Uniform(*candidateLaneSecs)
oncomingLaneSec = egoLaneSec._laneToLeft
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
taxiSpawnPt = new OrientedPoint in oncomingLaneSec.centerline
sedanSpawnPt = new OrientedPoint behind taxiSpawnPt by globalParameters.OPT_TAXI_SEDAN_DIST

param OPT_EGO_SPEED = Range(10, 15)
param OPT_AVOID_DIST = Range(12, 18)

behavior EgoBehavior(speed, avoid_dist):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToAnyCars(self, avoid_dist)
    take SetBrakeAction(1.0)
    take SetSteerAction(1.0)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_AVOID_DIST)

param OPT_SEDAN_SPEED = Range(8, 12)

behavior SedanBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

sedan = new Car at sedanSpawnPt,
    with regionContainedIn oncomingLaneSec,
    with behavior SedanBehavior(globalParameters.OPT_SEDAN_SPEED)

require (distance from ego to sedan) > 5
terminate when (distance from ego to sedan) < 4