description = "Ego vehicle detects and reacts to a stationary target vehicle in a curve, positioned with a 0.5m lane offset."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param ADV_OFFSET = 0.5
param DIST_TO_CURVE = Range(40, 60)

# Select a lane that provides sufficient length for the scenario
selectedLane = Uniform(*filter(lambda l: l.centerline.length > 100, network.lanes))

# Define a reference point on the lane centerline that serves as the curve location
# Using 'in' ensures the point is chosen within the centerline region
curvePoint = new OrientedPoint in selectedLane.centerline

# Position the adversarial car at the curve point with a 0.5m lateral offset
# This uses the 'right of' specifier to create the lane offset from the center
advSpawnPt = new OrientedPoint right of curvePoint by globalParameters.ADV_OFFSET

# Position the ego vehicle behind the curve point to place it on the straight section
# The 'behind' specifier ensures it follows the road geometry backwards
egoSpawnPt = new OrientedPoint behind curvePoint by globalParameters.DIST_TO_CURVE

param OPT_EGO_SPEED = Range(7, 10)
param OPT_BRAKE_DIST = Range(12, 15)

behavior EgoBehavior(speed, dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, dist):
        take SetBrakeAction(1)
        take SetThrottleAction(0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DIST)

## Adversary ##
adversary = new Car at advSpawnPt,
    with blueprint MODEL

param TERMINATION_DIST = 80
param MAX_TIME = 30

require egoSpawnPt can see adversary
terminate when (distance from ego to egoSpawnPt) > globalParameters.TERMINATION_DIST
terminate after globalParameters.MAX_TIME seconds