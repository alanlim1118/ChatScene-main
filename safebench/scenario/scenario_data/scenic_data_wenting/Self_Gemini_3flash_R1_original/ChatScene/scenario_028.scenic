description = "Ego performs an unprotected left turn, yielding to oncoming traffic, but is forced to change lanes by a slow, lane-blocking adversarial vehicle from the right."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way and not i.isSignalized, network.intersections))

egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.LEFT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint on egoInitLane.centerline

oncomingManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
oncomingInitLane = oncomingManeuver.startLane
oncomingTrajectory = [oncomingInitLane, oncomingManeuver.connectingLane, oncomingManeuver.endLane]
oncomingSpawnPt = new OrientedPoint on oncomingInitLane.centerline

advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advInitLane = advManeuver.startLane
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = new OrientedPoint on advInitLane.centerline

egoVehicle = new Car at egoSpawnPt,
    with blueprint MODEL

oncomingVehicle = new Car at oncomingSpawnPt,
    with blueprint MODEL

adversaryVehicle = new Car at advSpawnPt,
    with blueprint MODEL

param EGO_SPEED = Range(5, 7)
param YIELD_THRESHOLD = 15

behavior EgoBehavior():
    yielded = False
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=egoTrajectory)
    interrupt when (not yielded) and (distance to oncomingVehicle < globalParameters.YIELD_THRESHOLD):
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        wait until (distance to oncomingVehicle > globalParameters.YIELD_THRESHOLD + 5)
        yielded = True
    
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until withinDistanceToObjsInLane(self, 15)
    
    target_sec = self.laneSection._laneToLeft if self.laneSection._laneToLeft else self.laneSection._laneToRight
    if target_sec:
        do LaneChangeBehavior(laneSectionToSwitch=target_sec, target_speed=globalParameters.EGO_SPEED)
    
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param ONCOMING_SPEED = Range(7, 10)

behavior OncomingBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ONCOMING_SPEED, trajectory=trajectory)

oncomingVehicle = new Car at oncomingSpawnPt,
    with blueprint MODEL,
    with behavior OncomingBehavior(oncomingTrajectory)

param ADV_SPEED = Range(2, 4)

behavior AdvBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)
    target_sec = self.laneSection._laneToLeft if self.laneSection._laneToLeft else self.laneSection._laneToRight
    if target_sec:
        do LaneChangeBehavior(laneSectionToSwitch=target_sec, target_speed=globalParameters.ADV_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversaryVehicle = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdvBehavior(advTrajectory)

param MIN_INTERSECTION_DIST = 15
param MAX_INTERSECTION_DIST = 30

require globalParameters.MIN_INTERSECTION_DIST <= (distance from egoSpawnPt to intersection) <= globalParameters.MAX_INTERSECTION_DIST
require globalParameters.MIN_INTERSECTION_DIST <= (distance from oncomingSpawnPt to intersection) <= globalParameters.MAX_INTERSECTION_DIST + 10
require globalParameters.MIN_INTERSECTION_DIST <= (distance from advSpawnPt to intersection) <= globalParameters.MAX_INTERSECTION_DIST + 10

terminate when (distance from ego to egoSpawnPt) > 100
terminate after 60 seconds