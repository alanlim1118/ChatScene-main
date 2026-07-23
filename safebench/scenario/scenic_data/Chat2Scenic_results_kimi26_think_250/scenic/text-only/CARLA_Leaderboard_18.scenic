description = "Ego vehicle performs an emergency brake or avoidance maneuver when a leading vehicle suddenly decelerates due to an obstacle."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEADING_DIST = Range(15, 30)
param OPT_PROP_DIST = Range(8, 15)

egoLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint in egoLane.centerline

LeadingSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.OPT_LEADING_DIST

PropSpawnPt = new OrientedPoint following egoLane.orientation from LeadingSpawnPt for globalParameters.OPT_PROP_DIST

param EGO_SPEED = Range(7, 10)
param EGO_BRAKE = Range(0.8, 1.0)
SAFE_DIST = 10

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, SAFE_DIST):
        take SetBrakeAction(globalParameters.EGO_BRAKE)

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

param LEAD_SPEED = Range(7, 10)
param LEAD_BRAKE = Range(0.8, 1.0)
param LEAD_SAFE_DIST = Range(8, 12)

behavior LeadBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)
    interrupt when (distance from self to PropSpawnPt) < globalParameters.LEAD_SAFE_DIST:
        take SetBrakeAction(globalParameters.LEAD_BRAKE)

leading = new Car at LeadingSpawnPt,
    with blueprint MODEL,
    with behavior LeadBehavior()

prop = new Debris at PropSpawnPt