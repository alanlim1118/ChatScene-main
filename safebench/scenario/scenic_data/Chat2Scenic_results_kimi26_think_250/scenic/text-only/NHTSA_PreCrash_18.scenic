description = "Ego vehicle changes lanes to pass a lead vehicle in an urban area under clear daytime conditions."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

initLane = Uniform(*network.lanes)
advSpawnPt = new OrientedPoint in initLane.centerline
egoSpawnPt = new OrientedPoint behind advSpawnPt by Range(15, 30)

param OPT_EGO_SAFETY_DISTANCE = Range(7, 9)
param OPT_EGO_SPEED = Range(6, 8)
OPT_OVERTAKE_DISTANCE = 10

behavior EgoBehavior(ego_speed, overtake_distance, safety_distance, lane_change_target):
    try:
        do FollowLaneBehavior(target_speed=ego_speed) until (distance from self to LeadingAgent < overtake_distance)
        do LaneChangeBehavior(laneSectionToSwitch=lane_change_target, target_speed=ego_speed)
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when withinDistanceToObjsInLane(self, safety_distance):
        take SetBrakeAction(1)

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.OPT_EGO_SPEED,
        OPT_OVERTAKE_DISTANCE,
        globalParameters.OPT_EGO_SAFETY_DISTANCE,
        targetLaneSec
    )

param ADV_SPEED = Range(7, 10)

behavior AdversaryBehavior():
	do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
	with behavior AdversaryBehavior()