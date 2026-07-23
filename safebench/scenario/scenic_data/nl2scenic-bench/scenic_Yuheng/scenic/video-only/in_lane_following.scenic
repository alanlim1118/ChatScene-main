"""Scenario Description:

In this top-down simulation view, the ego vehicle, represented by a red rectangular block, travels straight along a grey road marked with dashed white lane dividers. The ego vehicle is following a white vehicle with black stripes that is positioned ahead in the same lane, maintaining a consistent following distance as both vehicles proceed from right to left. The scene begins with a blue area visible on the right side of the screen, which disappears as the vehicles advance onto the uniform grey road surface, while telemetry data at the bottom indicates the ego vehicle's speed fluctuates between approximately 60 and 80 km/h.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
LEAD_MODEL = "vehicle.tesla.model3"

# Speed in m/s: 60 km/h ≈ 16.67 m/s, 80 km/h ≈ 22.22 m/s
param EGO_MIN_SPEED = 16.67
param EGO_MAX_SPEED = 22.22
param LEAD_SPEED = Range(16.67, 22.22)
param FOLLOW_DISTANCE = Range(15, 25)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoFollowingBehavior(min_speed, max_speed):
    """Ego follows the lead vehicle, adjusting speed between min and max."""
    target_speed = Uniform(min_speed, max_speed)
    do FollowLaneBehavior(target_speed=target_speed)

behavior LeadVehicleBehavior(speed):
    """Lead vehicle drives at a constant speed in its lane."""
    do FollowLaneBehavior(target_speed=speed)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Find a suitable straight lane section for spawning
laneSections = []
for lane in network.lanes:
    for sec in lane.sections:
        if sec.isForward and sec.length > 80:
            laneSections.append(sec)

require len(laneSections) > 0
chosenLaneSec = Uniform(*laneSections)

# Spawn ego vehicle on the chosen lane
egoSpawnPt = new OrientedPoint in chosenLaneSec.centerline

# Spawn lead vehicle ahead of ego in the same lane
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.FOLLOW_DISTANCE

# Ego vehicle (red)
ego = new Car at egoSpawnPt,
    with regionContainedIn chosenLaneSec,
    with blueprint EGO_MODEL,
    with color (1.0, 0.0, 0.0),
    with behavior EgoFollowingBehavior(globalParameters.EGO_MIN_SPEED, globalParameters.EGO_MAX_SPEED)

# Lead vehicle (white with black stripes - approximated by white color)
LeadAgent = new Car at leadSpawnPt,
    with regionContainedIn chosenLaneSec,
    with blueprint LEAD_MODEL,
    with color (1.0, 1.0, 1.0),
    with behavior LeadVehicleBehavior(globalParameters.LEAD_SPEED)

# Ensure sufficient road ahead for the scenario to play out
require distance to intersection >= 100