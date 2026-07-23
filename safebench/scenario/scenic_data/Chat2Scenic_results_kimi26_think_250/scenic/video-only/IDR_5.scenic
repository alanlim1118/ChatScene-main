description = "Ego vehicle approaches a four-way intersection at night and yields to multiple adversary vehicles entering simultaneously, including an oncoming left-turner and cross-traffic, navigating carefully to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

intersection = Uniform(*filter(lambda i: i.is4Way, network.intersections))
egoInitLane = Uniform(*filter(lambda l: any(m.type is ManeuverType.STRAIGHT for m in l.maneuvers), intersection.incomingLanes))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = new OrientedPoint in egoInitLane.centerline

advOppManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.reverseManeuvers))
advOppInitLane = advOppManeuver.startLane
advOppTrajectory = [advOppInitLane, advOppManeuver.connectingLane, advOppManeuver.endLane]
advOppSpawnPt = new OrientedPoint in advOppInitLane.centerline

advCrossManeuver1 = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoManeuver.conflictingManeuvers))
advCrossInitLane1 = advCrossManeuver1.startLane
advCrossTrajectory1 = [advCrossInitLane1, advCrossManeuver1.connectingLane, advCrossManeuver1.endLane]
advCrossSpawnPt1 = new OrientedPoint in advCrossInitLane1.centerline

advCrossManeuver2 = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, advOppManeuver.conflictingManeuvers))
advCrossInitLane2 = advCrossManeuver2.startLane
advCrossTrajectory2 = [advCrossInitLane2, advCrossManeuver2.connectingLane, advCrossManeuver2.endLane]
advCrossSpawnPt2 = new OrientedPoint in advCrossInitLane2.centerline

param EGO_SPEED = Range(5, 8)
param EGO_BRAKE = Range(0.5, 1.0)
SAFE_DIST = 20

behavior EgoBehavior(trajectory):
	try:
		do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
	interrupt when withinDistanceToAnyObjs(self, SAFE_DIST):
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(5, 8)

behavior AdversaryBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adversary = new Car at (new OrientedPoint in advOppInitLane.centerline),
	with blueprint MODEL,
	with behavior AdversaryBehavior([advOppInitLane, Uniform(*filter(lambda m: m.startLane is advOppInitLane, egoManeuver.conflictingManeuvers)).connectingLane, Uniform(*filter(lambda m: m.startLane is advOppInitLane, egoManeuver.conflictingManeuvers)).endLane])

behavior AdvForward(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

adv_cross1 = new Car at advCrossSpawnPt1,
	with blueprint MODEL,
	with behavior AdvForward(advCrossTrajectory1)

behavior AdvTurnBehavior(startLane):
	turnManeuver = Uniform(*filter(lambda m: m.type is not ManeuverType.STRAIGHT, startLane.maneuvers))
	do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=[startLane, turnManeuver.connectingLane, turnManeuver.endLane])

adv_turn = new Car at advCrossSpawnPt2,
	with blueprint MODEL,
	with behavior AdvTurnBehavior(advCrossInitLane2)

monitor IntersectionLights():
    freezeTrafficLights()
    while True:
        setClosestTrafficLightStatus(ego, "green", 200)
        setClosestTrafficLightStatus(adversary, "green", 200)
        setClosestTrafficLightStatus(adv_cross1, "green", 200)
        setClosestTrafficLightStatus(adv_turn, "green", 200)
        wait

require monitor IntersectionLights()
require 15 <= (distance from egoSpawnPt to intersection) <= 40
require 10 <= (distance from ego to adversary) <= 80
require 10 <= (distance from ego to adv_cross1) <= 80
require 10 <= (distance from ego to adv_turn) <= 80
terminate when (distance from ego to egoSpawnPt >= 50)