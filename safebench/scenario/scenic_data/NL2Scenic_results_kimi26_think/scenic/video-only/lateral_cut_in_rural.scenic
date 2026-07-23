"""Scenario Description:

The ego vehicle travels along a narrow two-way rural road under adverse weather conditions, including dense fog, light rain, and a wet, low-friction road surface. Visibility is limited, and roadside trees, vegetation, and buildings further restrict the driving environment. The ego vehicle follows traffic while encountering oncoming vehicles. As an opposing vehicle attempts to avoid a roadside hazard, it suddenly crosses into the ego lane with little warning. Due to the narrow roadway, limited lateral clearance, reduced tire grip, and insufficient reaction time, the ego vehicle cannot safely evade the intrusion. The maneuver results in a side-swipe collision, creating a challenging emergency scenario that tests perception, decision-making, and collision avoidance under degraded environmental conditions.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

EGO_SPEED = 8
ADV_SPEED = 10

# Scenario geometry
ADV_DISTANCE_TO_HAZARD = [50, 80]
EGO_HAZARD_PROXIMITY = 10      # meters; ensures ego is opposite the hazard

# Behavior thresholds
SAFETY_DIST = 20
CRASH_DIST = 5
SWERVE_DIST = 15
STEER_VALUE = -0.8             # hard swerve toward the ego lane

TERM_DIST = 120

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, adversary):
    try:
        do FollowLaneBehavior(speed)
    interrupt when (distance to adversary) < SAFETY_DIST:
        take SetBrakeAction(1.0)
    interrupt when (distance to adversary) < CRASH_DIST:
        terminate

behavior SwerveBehavior():
    while True:
        take SetSteerAction(STEER_VALUE)

behavior AdversaryBehavior(speed, hazard):
    try:
        do FollowLaneBehavior(speed)
    interrupt when (distance to hazard) < SWERVE_DIST:
        do SwerveBehavior()

#################################
# SPATIAL RELATIONS             #
#################################

# Select a narrow two-way road (assumed two lanes, one in each direction)
road = Uniform(*network.roads)
egoLane = Uniform(*road.lanes)
advLane = Uniform(*filter(lambda l: l is not egoLane, road.lanes))

# Place hazard on the roadside of the adversary's lane
hazardPt = new OrientedPoint on advLane.centerline offset by 0 @ Range(2, 3)

# Place adversary approaching the hazard
advSpawnPt = new OrientedPoint at hazardPt offset by -Range(*ADV_DISTANCE_TO_HAZARD) @ 0

# Place ego on the opposite lane, roughly aligned with the hazard
egoSpawnPt = new OrientedPoint on egoLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Roadside hazard (e.g., debris)
hazard = new Trash at hazardPt

# Adversary vehicle in oncoming lane
adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior(ADV_SPEED, hazard)

# Ego vehicle
ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(EGO_SPEED, adversary)

# Ensure ego is opposite the hazard and adversary is approaching it
require (distance from ego to hazard) < EGO_HAZARD_PROXIMITY
require ADV_DISTANCE_TO_HAZARD[0] <= (distance from adversary to hazard) <= ADV_DISTANCE_TO_HAZARD[1]

terminate when (distance to egoSpawnPt) > TERM_DIST