description = "Vehicle A misjudges distance of Vehicle B during lane change, leading to a rear-end collision."
param map = localPath('../../maps/Town10HD.xodr')
param carla_map = 'Town10HD'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithAdj = []
for lane in network.lanes:
	for sec in lane.sections:
		if sec.isForward:
			if (sec._laneToLeft and sec._laneToLeft.isForward) or (sec._laneToRight and sec._laneToRight.isForward):
				laneSecsWithAdj.append(sec)

egoSection = Uniform(*laneSecsWithAdj)
egoSpawnPt = new OrientedPoint in egoSection.centerline

adjSection = Uniform(*filter(lambda s: s is not None and s.isForward, [egoSection._laneToLeft, egoSection._laneToRight]))

projectedPt = adjSection.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following adjSection.orientation from projectedPt for Range(-25, -15)

egoTrajectory = [egoSection.lane]
advTrajectory = [adjSection.lane]

param EGO_SPEED = Range(10, 15)

behavior EgoBehavior(target_speed, target_lane):
	do FollowLaneBehavior(target_speed=target_speed) for Range(2, 4) seconds
	do LaneChangeBehavior(laneSectionToSwitch=target_lane, target_speed=target_speed)
	do FollowLaneBehavior(target_speed=target_speed)

ego = new Car at egoSpawnPt,
	with rolename 'hero',
	with regionContainedIn egoSection,
	with blueprint MODEL,
	with behavior EgoBehavior(globalParameters.EGO_SPEED, adjSection)

param ADV_SPEED = globalParameters.EGO_SPEED + Range(5, 10)
param BRAKE_DISTANCE = Range(8, 15)
param BRAKE_FORCE = Range(0.4, 0.7)

behavior AdversarialBehavior(speed, brake_dist, brake_val):
	do FollowLaneBehavior(target_speed=speed) until (distance from self to ego) < brake_dist
	while True:
		take SetBrakeAction(brake_val)

advAgent = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversarialBehavior(globalParameters.ADV_SPEED, globalParameters.BRAKE_DISTANCE, globalParameters.BRAKE_FORCE)

require 15 <= (distance from egoSpawnPt to advSpawnPt) <= 25
terminate when ego intersects advAgent