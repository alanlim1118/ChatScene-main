description = "Ego vehicle exits the travel lane to park in a gap between two stationary vehicles on a straight urban street."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_PARKED_OFFSET = Range(15, 25)
param OPT_PARKED_GAP = Range(10, 15)

laneSecsLeftmostWithRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is None and laneSec._laneToRight is not None:
            laneSecsLeftmostWithRight.append(laneSec)

egoLaneSec = Uniform(*laneSecsLeftmostWithRight)
rightLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

rearBasePos = rightLaneSec.centerline.project(egoSpawnPt.position)
parkedRearSpawnPt = new OrientedPoint following roadDirection from rearBasePos for globalParameters.OPT_PARKED_OFFSET
parkedFrontSpawnPt = new OrientedPoint following roadDirection from parkedRearSpawnPt for globalParameters.OPT_PARKED_GAP

param OPT_EGO_SPEED = Range(5, 8)
param OPT_PARK_TRIGGER_DIST = Range(6, 10)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) until (distance from self to parkedRearSpawnPt < globalParameters.OPT_PARK_TRIGGER_DIST)
    do LaneChangeBehavior(laneSectionToSwitch=rightLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
    take SetThrottleAction(0)
    take SetBrakeAction(1)
    take SetHandBrakeAction(True)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

adversary = new Car at parkedRearSpawnPt,
    with blueprint MODEL

adversary_front = new Car at parkedFrontSpawnPt,
    with blueprint MODEL