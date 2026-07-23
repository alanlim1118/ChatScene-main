description = "Ego vehicle emergency brakes when an aggressive black sedan cuts into its lane and then collides with a truck during a failed lane change."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_AHEAD_DIST = Range(20, 40)
param OPT_TRUCK_DIST = Range(15, 35)
param OPT_REAR_DIST = Range(-60, -30)

laneSecsWithLeftAndRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToRight is not None:
            laneSecsWithLeftAndRight.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftAndRight)
leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)

leftCarSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_AHEAD_DIST
rightTruckSpawnPt = new OrientedPoint following roadDirection from rightLanePt for globalParameters.OPT_TRUCK_DIST
advSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_REAR_DIST

param OPT_EGO_SPEED = Range(10, 15)
param OPT_EGO_AVOIDANCE = Range(8, 12)

behavior EgoBehavior(speed, avoidance):
    do DriveAvoidingCollisions(target_speed=speed, avoidance_threshold=avoidance)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_EGO_AVOIDANCE)

param OPT_ADV_SPEED = Range(10, 15)

behavior AdvTravelForward(speed):
    do FollowLaneBehavior(target_speed=speed)

adv = new Car at advSpawnPt,
    with behavior AdvTravelForward(globalParameters.OPT_ADV_SPEED)

param OPT_TRUCK_SPEED = Range(8, 12)

truck = new Truck at rightTruckSpawnPt,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_TRUCK_SPEED)

param OPT_ADV_CUT_SPEED = Range(12, 18)
param OPT_CUT_DIST = Range(20, 30)

behavior AggressiveCutIn():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_CUT_SPEED) until (distance from self to ego) < globalParameters.OPT_CUT_DIST
    do LaneChangeBehavior(laneSectionToSwitch=egoLaneSec, target_speed=globalParameters.OPT_ADV_CUT_SPEED)
    do LaneChangeBehavior(laneSectionToSwitch=rightLaneSec, target_speed=globalParameters.OPT_ADV_CUT_SPEED)

black_sedan = new Car at advSpawnPt,
    with color Color(0, 0, 0),
    with regionContainedIn None,
    with behavior AggressiveCutIn()

require 30 <= (distance from ego to black_sedan) <= 65
terminate when black_sedan intersects truck