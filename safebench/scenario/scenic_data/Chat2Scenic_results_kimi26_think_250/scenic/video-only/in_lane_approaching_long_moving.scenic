description = "Ego vehicle approaches a merging vehicle from the right while a parallel vehicle travels in the left lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

laneSecsWithLeftAndRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithLeftAndRight.append(laneSec)
egoLaneSec = Uniform(*laneSecsWithLeftAndRight)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leftLaneSec = egoLaneSec._laneToLeft
rightLaneSec = egoLaneSec._laneToRight
leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
rightLanePt = rightLaneSec.centerline.project(egoSpawnPt.position)
leftSpawnPt = new OrientedPoint at leftLanePt facing egoSpawnPt.heading
rightSpawnPt = new OrientedPoint at rightLanePt facing egoSpawnPt.heading

behavior EgoBehavior():
    do FollowLaneBehavior()

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(5, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at rightSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()

param ADV_LEFT_SPEED = Range(5, 10)

behavior LeftAdvBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_LEFT_SPEED)

adversary_left = new Car at leftSpawnPt,
	with blueprint MODEL,
	with behavior LeftAdvBehavior()