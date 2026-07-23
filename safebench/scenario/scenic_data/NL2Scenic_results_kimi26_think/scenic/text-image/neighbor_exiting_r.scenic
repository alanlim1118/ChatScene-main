"""Scenario Description:

In a top-down schematic view of a multi-lane roadway, a blue ego vehicle travels straight forward in the upper lane, indicated by a straight horizontal arrow pointing to the right. Positioned in the adjacent lane below, a pink adversarial vehicle executes a maneuver to steer away towards the right, depicted by a curved pink arrow that sweeps downward and to the right, signifying an object exiting from the right side of the ego vehicle's path.

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
ADV_MODEL = "vehicle.tesla.model3"

# RGB colors
EGO_COLOR = [0, 0, 1]      # Blue
ADV_COLOR = [1, 0, 1]      # Pink

param OPT_EGO_SPEED = Range(8, 12)
param OPT_ADV_SPEED = Range(8, 12)
param OPT_ADV_DIST = Range(10, 20)          # How far ahead the adversary spawns
param OPT_LANE_CHANGE_DIST = Range(15, 25)  # Distance to ego at which adversary steers away

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior AdvBehavior(speed, target_lane, trigger_dist):
    # Follow the adjacent lane until within a threshold of the ego,
    # then change lane to the right (away from the ego's path).
    do FollowLaneBehavior(target_speed=speed) until (distance from self to ego < trigger_dist)
    do LaneChangeBehavior(laneSectionToSwitch=target_lane, is_oppositeTraffic=False, target_speed=speed)
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Select forward lane sections that have at least two lanes to the right:
# ego in the left lane, adversary in the middle lane, and a target lane further right.
laneSecsWithTwoRight = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward and 
            laneSec._laneToRight is not None and 
            laneSec._laneToRight._laneToRight is not None):
            laneSecsWithTwoRight.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithTwoRight)
advLaneSec = egoLaneSec._laneToRight
targetLaneSec = advLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle: blue, traveling straight in the upper lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color EGO_COLOR,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED)

# Adversarial vehicle: pink, in the adjacent lane below, steers away to the right
AdvAgent = new Car at advSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn advLaneSec,
    with blueprint ADV_MODEL,
    with color ADV_COLOR,
    with behavior AdvBehavior(
        globalParameters.OPT_ADV_SPEED,
        targetLaneSec,
        globalParameters.OPT_LANE_CHANGE_DIST
    )

require distance to intersection >= 100  # Ensure a long, straight roadway