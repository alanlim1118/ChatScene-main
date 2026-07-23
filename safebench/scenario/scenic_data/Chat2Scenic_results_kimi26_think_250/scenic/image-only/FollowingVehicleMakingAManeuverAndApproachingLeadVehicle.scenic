description = "A vehicle performs a lane change on an urban road, closing in on a lead vehicle in the target lane while another vehicle continues in the original lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEAD_DIST = Range(20, 40)
param OPT_SAME_LANE_DIST = Range(10, 30)

laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRightLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

adjLaneSec = egoLaneSec._laneToRight
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
LeadSpawnPt = new OrientedPoint following roadDirection from adjLanePt for globalParameters.OPT_LEAD_DIST

SameLaneSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_SAME_LANE_DIST

param EGO_SPEED = Range(7, 10)
LANE_CHANGE_DELAY = 5

behavior EgoBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) for LANE_CHANGE_DELAY seconds
	do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToRight, target_speed=globalParameters.EGO_SPEED)
	do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
	with blueprint MODEL,
	with behavior EgoBehavior()

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at LeadSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

behavior SameLaneBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

sameLaneCar = new Car at SameLaneSpawnPt,
	with blueprint MODEL,
	with behavior SameLaneBehavior()

param TERM_DIST = 100

require 15 <= (distance from ego to adversary) <= 50

terminate when (distance to egoSpawnPt) > TERM_DIST