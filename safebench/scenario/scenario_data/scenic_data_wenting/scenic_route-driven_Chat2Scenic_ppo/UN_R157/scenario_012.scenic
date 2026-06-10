description = "Ego vehicle follows a lead, which evades, revealing a slow motorcycle. Ego must decelerate to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DIST_EGO_TO_LEAD = Range(10, 15)
param OPT_DIST_LEAD_TO_MOTO = Range(15, 25)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

initLane = network.laneAt(egoSpawnPt.position)

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_DIST_EGO_TO_LEAD
motoSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.OPT_DIST_LEAD_TO_MOTO

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(10, 15)
param OPT_MOTO_SPEED = Range(3, 5)
param OPT_EVASION_DIST = Range(8, 12)

behavior LeadBehavior(speed, evasion_dist):
    do FollowLaneBehavior(target_speed=speed) until withinDistanceToObjsInLane(self, evasion_dist)

    targetLane = None
    if self.laneSection._laneToLeft:
        targetLane = self.laneSection._laneToLeft
    elif self.laneSection._laneToRight:
        targetLane = self.laneSection._laneToRight

    if targetLane:
        do LaneChangeBehavior(laneSectionToSwitch=targetLane, target_speed=speed)

    do FollowLaneBehavior(target_speed=speed)

moto = new Motorcycle at motoSpawnPt,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_MOTO_SPEED)

leadCar = new Car at leadSpawnPt,
    with regionContainedIn initLane,
    with behavior LeadBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_EVASION_DIST)

require distance to intersection >= 50
terminate when ego intersects moto
terminate after 20 seconds
