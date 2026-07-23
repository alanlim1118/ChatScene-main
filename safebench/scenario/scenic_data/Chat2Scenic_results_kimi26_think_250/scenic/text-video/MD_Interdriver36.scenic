description = "Ego vehicle approaches an intersection under an overpass and attempts a right-turn lane change but must yield to adversary vehicles in the target lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
advManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, intersection.maneuvers))
advInitLane = advManeuver.startLane
advLaneSec = Uniform(*filter(lambda s: s._laneToLeft is not None and s._laneToRight is None, advInitLane.sections))
egoLaneSec = advLaneSec._laneToLeft
egoInitLane = egoLaneSec.lane
egoSpawnPt = new OrientedPoint in egoInitLane.centerline
advSpawnPt1 = new OrientedPoint in advInitLane.centerline
advSpawnPt2 = new OrientedPoint behind advSpawnPt1 by Range(10, 20)
advSpawnPt3 = new OrientedPoint behind advSpawnPt2 by Range(10, 20)
advTrajectory = [advInitLane, advManeuver.connectingLane, advManeuver.endLane]
egoDir = egoSpawnPt.heading
advDir = advSpawnPt1.heading

param OPT_EGO_SPEED = Range(5, 8)
param OPT_YIELD_DIST = Range(8, 12)
param OPT_FOLLOW_TIME = Range(2, 4)

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED) for globalParameters.OPT_FOLLOW_TIME seconds
        do LaneChangeBehavior(laneSectionToSwitch=advLaneSec, target_speed=globalParameters.OPT_EGO_SPEED)
        rightManeuver = [m for m in advInitLane.maneuvers if m.type is ManeuverType.RIGHT][0]
        turnTrajectory = [advInitLane, rightManeuver.connectingLane, rightManeuver.endLane]
        do FollowTrajectoryBehavior(trajectory=turnTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToAnyCars(self, globalParameters.OPT_YIELD_DIST):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at advSpawnPt1,
	with blueprint MODEL,
	with color [1, 0, 0],
	with behavior AdversaryBehavior(advTrajectory)

param GREY_ADV_SPEED = Range(7, 10)

behavior GreyCarBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.GREY_ADV_SPEED, trajectory=trajectory)

adversary2 = new Car at advSpawnPt2,
	with blueprint MODEL,
	with color [0.5, 0.5, 0.5],
	with behavior GreyCarBehavior(advTrajectory)

param GREY_ADV_SPEED_2 = Range(7, 10)

behavior GreyCarBehavior2(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.GREY_ADV_SPEED_2, trajectory=trajectory)

adversary3 = new Car at advSpawnPt3,
	with blueprint MODEL,
	with color [0.5, 0.5, 0.5],
	with behavior GreyCarBehavior2(advTrajectory)