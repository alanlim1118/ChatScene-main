description = "Ego vehicle follows a lead, which evades, revealing a slow motorcycle. Ego must decelerate to avoid collision."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.yamaha.yzf'
param weather = 'ClearNoon'

param DIST_EGO_TO_LEAD = Range(10, 15)
param DIST_LEAD_TO_MOTO = Range(15, 25)

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in initLane.centerline

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.DIST_EGO_TO_LEAD
motoSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.DIST_LEAD_TO_MOTO

param OPT_EGO_SPEED = Range(10, 15)
param OPT_BRAKE_DIST = Range(12, 18)

behavior EgoBehavior(speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
    terminate

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn initLane,
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DIST)

param ADV_SPEED = globalParameters.OPT_EGO_SPEED
param MOTO_SPEED = Range(3, 5)
param EVASION_DIST = Range(8, 12)

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
    with behavior FollowLaneBehavior(target_speed=globalParameters.MOTO_SPEED)

leadCar = new Car at leadSpawnPt,
    with regionContainedIn initLane,
    with behavior LeadBehavior(globalParameters.ADV_SPEED, globalParameters.EVASION_DIST)


require distance to intersection >= 50
terminate when ego intersects moto
terminate after 20 seconds