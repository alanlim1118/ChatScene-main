description = "Ego vehicle executes a lane change to the right while another vehicle follows in the adjacent left lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_ADV_START_DIST = Range(5, 10) * -1

laneSecsWithLeftRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithLeftRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

leftLaneSec = egoLaneSec._laneToLeft
leftLanePt = leftLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from leftLanePt for globalParameters.OPT_ADV_START_DIST

param EGO_SPEED = Range(8, 12)

behavior EgoBehavior():
    do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToRight, target_speed=globalParameters.EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with blueprint MODEL,
	with behavior AdversaryBehavior()