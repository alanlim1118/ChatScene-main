description = "Ego vehicle travels straight as an object enters from the right side after performing a lane change."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

road = Uniform(*filter(lambda r: r.forwardLanes is not None and len(r.forwardLanes.lanes) >= 3, network.roads))
upperLane = road.forwardLanes.lanes[0]
centralLane = road.forwardLanes.lanes[1]
lowerLane = road.forwardLanes.lanes[2]
upperSection = Uniform(*upperLane.sections)
centralSection = Uniform(*centralLane.sections)
lowerSection = Uniform(*lowerLane.sections)
egoSpawnPt = new OrientedPoint in upperSection.centerline
advSpawnPt = new OrientedPoint in lowerSection.centerline
egoTrajectory = [upperLane]
advTrajectory = [lowerLane, centralLane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
	do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior(egoTrajectory)

param ADV_SPEED = Range(6, 9)

behavior AdvBehavior():
	targetLaneSec = self.laneSection.fasterLane
	do LaneChangeBehavior(
			laneSectionToSwitch=targetLaneSec,
			target_speed=globalParameters.ADV_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdvBehavior()