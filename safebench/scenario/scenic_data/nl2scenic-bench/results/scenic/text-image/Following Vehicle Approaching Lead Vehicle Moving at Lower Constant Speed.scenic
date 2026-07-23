"""Scenario Description:

In a top-down schematic view of a traffic scenario, a vehicle travels straight along a lane in an urban area during daylight with clear weather conditions. The road segment is a non-junction with a posted speed limit of 55 mph or more, delineated by a dotted lane marking above and a solid road edge below. The subject vehicle, depicted as a larger, boxy vehicle in the rear, is following a lead vehicle, which appears as a smaller car ahead in the same lane. Both vehicles display arrows indicating forward movement, and the situation involves the rear vehicle closing in on the lead vehicle, which is maintaining a lower constant speed.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.ford.mustang"  # Larger, boxy vehicle for the rear/ego
LEAD_MODEL = "vehicle.tesla.model3"  # Smaller car for the lead vehicle

# Speeds in m/s (55 mph ≈ 24.6 m/s)
param OPT_LEAD_SPEED = Range(18, 22)       # Lead vehicle maintains lower constant speed (~40-49 mph)
param OPT_EGO_SPEED = Range(26, 30)        # Ego vehicle faster to close in (~58-67 mph, above 55 mph limit)
param OPT_INITIAL_GAP = Range(40, 60)      # Initial distance between ego and lead vehicle
param OPT_BRAKE_DISTANCE = Range(8, 15)    # Safety braking distance for ego

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadBehavior(target_speed):
    """Lead vehicle maintains a constant lower speed."""
    do FollowLaneBehavior(target_speed=target_speed)

behavior ClosingInBehavior(target_speed, brake_dist):
    """Ego vehicle follows lane at higher speed but brakes if too close to lead."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=brake_dist):
        take SetBrakeAction(1)

#################################
# SPATIAL RELATIONS             #
#################################

# Find non-junction road sections with sufficient length and forward lanes
validLaneSections = []
for lane in network.lanes:
    for section in lane.sections:
        if (section.isForward 
            and not section.isIntersection 
            and section.length >= 150
            and section._laneToLeft is None):  # Rightmost lane has solid road edge on right
            validLaneSections.append(section)

require len(validLaneSections) > 0

egoLaneSec = Uniform(*validLaneSections)

# Spawn points along the selected lane
leadSpawnPt = new OrientedPoint in egoLaneSec.centerline
egoSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for -globalParameters.OPT_INITIAL_GAP

#################################
# SCENARIO SPECIFICATION        #
#################################

# Lead vehicle: smaller car maintaining lower constant speed
leadVehicle = new Car at leadSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint LEAD_MODEL,
    with behavior LeadBehavior(globalParameters.OPT_LEAD_SPEED)

# Ego vehicle: larger boxy vehicle closing in from behind
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior ClosingInBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DISTANCE)

# Ensure we are far from any intersection (non-junction requirement)
require distance to intersection >= 100

# Terminate after ego has traveled sufficient distance or closed gap significantly
terminate when (distance from ego to leadVehicle < 10) or (distance from ego to egoSpawnPt > 200)