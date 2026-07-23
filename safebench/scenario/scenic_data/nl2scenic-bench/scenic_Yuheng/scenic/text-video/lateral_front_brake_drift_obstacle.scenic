"""Scenario Description:

Under nighttime conditions on a city street, the ego vehicle proceeds forward while following a white passenger car. The lead vehicle suddenly brakes, its red taillights flaring brightly, which forces the ego vehicle into an emergency swerve to the right to avoid a rear-end collision. This evasive action, however, steers the ego vehicle directly into a large, illuminated blue advertising sign or light box that is standing as a stationary hazard in the right lane or shoulder area, resulting in an impact with the obstacle.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
LEAD_CAR_MODEL = "vehicle.tesla.model3"  # White passenger car approximation

param OPT_EGO_SPEED = Range(8, 12)
param OPT_LEAD_SPEED = Range(8, 12)
param OPT_FOLLOW_DISTANCE = Range(15, 25)
param OPT_BRAKE_TRIGGER_DIST = Range(12, 18)
param OPT_SWERVE_OFFSET = Range(3.0, 4.5)  # Lateral offset for emergency swerve to the right
param OPT_SIGN_DISTANCE_AHEAD = Range(20, 35)  # Distance ahead of ego spawn where sign is placed

STEPS_PER_SEC = 10

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadCarBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_LEAD_SPEED)
    interrupt when (distance from self to ego) < globalParameters.OPT_BRAKE_TRIGGER_DIST:
        take SetBrakeAction(1.0)  # Sudden hard brake, taillights flare automatically in CARLA
        while True:
            take SetBrakeAction(1.0)

behavior EgoSwerveBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance from self to LeadCar) < globalParameters.OPT_BRAKE_TRIGGER_DIST:
        # Emergency swerve to the right to avoid rear-end collision
        do LaneChangeBehavior(direction='right', target_speed=globalParameters.OPT_EGO_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lanes that have a right neighbor (for swerve target and sign placement)
laneSecsWithRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            laneSecsWithRight.append(laneSec)

require len(laneSecsWithRight) > 0

egoLaneSec = Uniform(*laneSecsWithRight)
rightLaneSec = egoLaneSec._laneToRight

# Ego spawn point in the left lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Lead car spawn point ahead of ego in same lane
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_FOLLOW_DISTANCE

# Sign/hazard position: in the right lane or shoulder, ahead of ego's initial position
signBasePt = rightLaneSec.centerline.project(egoSpawnPt.position)
signSpawnPt = new OrientedPoint following roadDirection from signBasePt for globalParameters.OPT_SIGN_DISTANCE_AHEAD,
    with heading signBasePt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Nighttime weather
param timeOfDay = 22  # 10 PM
param weather = Weather(precipitation=0, cloudiness=0.8, fog=0.1, sunAltitude=-10)

# Lead vehicle: white passenger car that suddenly brakes
LeadCar = new Car at leadSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint LEAD_CAR_MODEL,
    with color (1.0, 1.0, 1.0),  # White color
    with behavior LeadCarBehavior()

# Ego vehicle: follows lead car, then swerves right upon lead braking
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoSwerveBehavior()

# Stationary hazard: large illuminated blue advertising sign/light box in right lane
BlueSign = new Object at signSpawnPt,
    with regionContainedIn rightLaneSec,
    with shape BoxShape(width=2.5, length=0.5, height=3.0),
    with color (0.0, 0.2, 0.8),  # Blue illuminated sign
    with isStatic True

# Ensure sufficient road length for the scenario to play out
require distance to intersection >= 60