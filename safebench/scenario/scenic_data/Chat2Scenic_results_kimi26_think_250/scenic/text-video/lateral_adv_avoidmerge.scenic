description = "A white SUV abruptly swerves left from the right adjacent lane into the ego vehicle's path to avoid a merging vehicle, causing a side-impact collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'WetCloudyNoon'

laneSecsWithTwoRightLanes = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward and laneSec._laneToRight._laneToRight is not None and laneSec._laneToRight._laneToRight.isForward:
            laneSecsWithTwoRightLanes.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithTwoRightLanes)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

rightLaneSec = egoLaneSec._laneToRight
advSpawnPt = new OrientedPoint in rightLaneSec.centerline

farRightLaneSec = rightLaneSec._laneToRight
mergeSpawnPt = new OrientedPoint in farRightLaneSec.centerline

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE = Range(0.8, 1.0)
COLLISION_DIST = 1.0

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) until withinDistanceToAnyCars(self, COLLISION_DIST)
	while True:
		take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(8, 12)
param ADV_FOLLOW_TIME = Range(2, 4)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) for globalParameters.ADV_FOLLOW_TIME seconds
    leftLaneSec = self.laneSection._laneToLeft
    do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with blueprint MODEL,
    with behavior AdvBehavior()

param MERGE_SPEED = Range(7, 10)
param MERGE_FOLLOW_TIME = Range(2, 4)

behavior MergeBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.MERGE_SPEED) for globalParameters.MERGE_FOLLOW_TIME seconds
	leftLaneSec = self.laneSection._laneToLeft
	do LaneChangeBehavior(laneSectionToSwitch=leftLaneSec, target_speed=globalParameters.MERGE_SPEED)

merger = new Car at mergeSpawnPt,
	with heading mergeSpawnPt.heading,
	with blueprint MODEL,
	with behavior MergeBehavior()

require (distance from ego to adversary) <= 20
terminate when intersects(ego, adversary)