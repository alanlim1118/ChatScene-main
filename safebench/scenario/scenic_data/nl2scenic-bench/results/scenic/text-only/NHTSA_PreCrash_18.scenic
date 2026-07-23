"""Scenario Description:

Vehicle is changing lanes or passing in an urban area, in daylight, under clear weather conditions, at a non-junction with a posted speed limit of 55 mph; and closes in on a lead vehicle.

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

# 55 mph ≈ 24.6 m/s
TARGET_SPEED = 24.6
LEAD_SPEED = Uniform(18, 22)  # Lead vehicle is slower to trigger closing in / passing
INITIAL_GAP = Range(30, 50)   # Initial distance to lead vehicle
BRAKE_DIST = Range(8, 12)     # Safety braking distance threshold
PASS_TRIGGER_DIST = Range(15, 25)  # Distance at which ego initiates lane change/pass

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(targetLaneSec):
    try:
        do FollowLaneBehavior(target_speed=TARGET_SPEED) until (distance from self to LeadVehicle < PASS_TRIGGER_DIST)
        do LaneChangeBehavior(laneSectionToSwitch=targetLaneSec, target_speed=TARGET_SPEED)
        do FollowLaneBehavior(target_speed=TARGET_SPEED)
    interrupt when withinDistanceToObjsInLane(self, BRAKE_DIST):
        take SetBrakeAction(1)

behavior LeadBehavior():
    do FollowLaneBehavior(target_speed=LEAD_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have an adjacent forward lane (for passing)
laneSecsWithAdjacent = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithAdjacent.append(laneSec)

assert len(laneSecsWithAdjacent) > 0, 'No suitable lane sections with adjacent left lane found.'

egoLaneSec = Uniform(*laneSecsWithAdjacent)
adjLaneSec = egoLaneSec._laneToLeft

# Spawn points along the centerline, away from intersections
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for INITIAL_GAP

#################################
# SCENARIO SPECIFICATION        #
#################################

# Weather and lighting: daylight, clear
param weather = 'Clear'
param timeOfDay = 'Day'

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(adjLaneSec)

LeadVehicle = new Car at leadSpawnPt,
    with regionContainedIn egoLaneSec,
    with heading ego.heading,
    with behavior LeadBehavior()

# Ensure non-junction: both vehicles must be far from any intersection
require distance from ego to intersection >= 100
require distance from LeadVehicle to intersection >= 100

# Terminate after sufficient travel distance
terminate when distance from ego to egoSpawnPt > 200