"""Scenario Description:

The ego vehicle proceeds forward along a narrow, snow-covered rural road under overcast winter daylight conditions, flanked by bare trees, utility poles, and low roadside structures including a building with red walls on the left and a white prefabricated shed on the right. An oncoming light-colored passenger car approaches in the opposite lane but suddenly loses traction on the icy road surface, causing it to skid sideways and drift directly into the ego vehicle's path. This loss of control creates an imminent head-on or side-swipe collision scenario as the sliding vehicle crosses the center of the roadway, while a second vehicle remains visible further ahead in the distance. The completely snow-packed road surface and flat gray sky contribute to reduced traction and moderate visibility, highlighting the hazardous winter driving conditions captured by the dashcam.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'  # Rural map with narrow roads suitable for this scenario
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ONCOMING_CAR_MODEL = "vehicle.tesla.model3"  # Light-colored passenger car
DISTANT_CAR_MODEL = "vehicle.audi.a2"

param OPT_EGO_SPEED = Range(8, 12)           # Moderate speed on icy road
param OPT_ONCOMING_SPEED = Range(10, 14)     # Oncoming car initial speed
param OPT_SKID_TRIGGER_DIST = Range(25, 40)  # Distance at which oncoming car begins to skid
param OPT_SKID_LATERAL_VEL = Range(2.5, 4.5) # Lateral velocity during skid (m/s)
param OPT_SKID_YAW_RATE = Range(-30, -15)    # Yaw rate during skid (deg/s), negative = clockwise spin
param OPT_DISTANT_CAR_DIST = Range(80, 120)  # Distance of second vehicle ahead
param OPT_FRICTION = 0.15                    # Very low friction for icy/snowy surface

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoWinterDriving():
    """Ego drives forward cautiously on snowy road."""
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, 15):
        take SetThrottleAction(0), SetBrakeAction(1)

behavior OncomingSkidBehavior(trigger_dist, lateral_vel, yaw_rate):
    """Oncoming car drives normally then loses traction and skids across centerline."""
    # Phase 1: Normal oncoming driving
    while distance from self to ego > trigger_dist:
        take SetThrottleAction(0.6), SetSteerAction(0)
    
    # Phase 2: Loss of traction - apply lateral force and yaw to simulate skid
    # Reduce throttle and introduce steering instability
    take SetThrottleAction(0.0)
    take SetBrakeAction(0.1)
    
    # Simulate skid by applying lateral movement toward ego's lane
    # The car drifts sideways across the centerline
    do SkidDriftBehavior(lateral_vel, yaw_rate) for 4 seconds
    
    # After skid, come to rest or continue sliding slowly
    take SetThrottleAction(0), SetBrakeAction(0.8)

behavior SkidDriftBehavior(lateral_vel, yaw_rate):
    """Simulates uncontrolled lateral drift by setting velocity components."""
    while True:
        # Apply lateral velocity toward ego's lane (positive = leftward in CARLA frame)
        # and rotational yaw to simulate loss of control
        take SetVelocityAction(Vector(0, lateral_vel, 0)), \
             SetAngularVelocityAction(Vector(0, 0, yaw_rate))

behavior DistantCarBehavior():
    """Second vehicle drives normally far ahead."""
    do FollowLaneBehavior(target_speed=Range(6, 10))

#################################
# SCENARIO SPECIFICATION        #
#################################

# Place ego on a straight rural road segment
egoLane = Uniform(*filter(lambda l: l.isForward and not l.isIntersection, network.lanes))
egoSpawn = new OrientedPoint in egoLane.centerline

ego = new Car at egoSpawn,
    with blueprint EGO_MODEL,
    with behavior EgoWinterDriving(),
    with regionContainedIn None

# Oncoming car in opposing lane
oncomingLane = egoLane.oppositeLane
require oncomingLane is not None

oncomingSpawn = new OrientedPoint in oncomingLane.centerline,
    facing opposite of egoSpawn.heading

oncomingCar = new Car at oncomingSpawn,
    with blueprint ONCOMING_CAR_MODEL,
    with color Vector(0.9, 0.9, 0.85),  # Light-colored
    with behavior OncomingSkidBehavior(
        globalParameters.OPT_SKID_TRIGGER_DIST,
        globalParameters.OPT_SKID_LATERAL_VEL,
        globalParameters.OPT_SKID_YAW_RATE
    ),
    with regionContainedIn None

# Ensure oncoming car starts at reasonable distance for the skid to develop
require 50 <= distance from ego to oncomingCar <= 80

# Second vehicle further ahead in ego's direction
distantSpawn = new OrientedPoint following egoLane.orientation from egoSpawn \
    for globalParameters.OPT_DISTANT_CAR_DIST

distantCar = new Car at distantSpawn,
    with blueprint DISTANT_CAR_MODEL,
    with behavior DistantCarBehavior(),
    with regionContainedIn None

# Environmental and scene constraints
# Require rural road characteristics (narrow, non-intersection)
require egoLane.width <= 8.0
require not egoLane.isIntersection

# Set weather parameters for overcast winter conditions
param weather = WeatherConditions(
    cloudiness=100,
    precipitation=0,
    precipitationDeposits=80,   # Snow deposits on road
    windIntensity=30,
    sunAzimuthAngle=180,
    sunAltitudeAngle=15,        # Low winter sun angle
    fogDensity=40,              # Moderate visibility reduction
    wetness=60                  # Wet/icy surface appearance
)

terminate after 20 seconds