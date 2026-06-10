"""Scenario Description:
While the ego vehicle is traveling on a straight highway, a lead vehicle from the left lane 
merges abruptly into the ego lane with a very short Time-to-Collision (TTC).
"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.audi.etron"

# Highway speeds in m/s (approx 80-100 km/h)
param EGO_SPEED = Range(22, 28)
param ADV_SPEED = Range(20, 24)

# Trigger distance for the abrupt merge to ensure a short TTC
# At highway speeds, 15-25 meters represents a very tight gap
param MERGE_TRIGGER_DIST = Range(15, 25)

# Initial longitudinal gap
param INITIAL_GAP = Range(30, 40)

#################################
# AGENT BEHAVIORS               #
#################################

behavior AdvBehavior(target_lane_sec):
    # Drive in the left lane initially
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED) until (distance from self to ego < globalParameters.MERGE_TRIGGER_DIST)
    
    # Abruptly merge into the ego's lane
    try:
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=globalParameters.ADV_SPEED)
    interrupt when withinDistanceToAnyCars(self, 5):
        # Continue driving after merge or if too close
        pass
        
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for highway lane sections that have a lane to their left
laneSecsWithLeftLane = []
for ls in network.laneSections:
    # We look for a lane that has a neighbor on the left, both moving forward (highway)
    if ls.isForward and ls.laneToLeft and ls.laneToLeft.isForward:
        # Town06 has long highway stretches; we ensure we aren't at a junction
        if not ls.lane.road.intersections:
            laneSecsWithLeftLane.append(ls)

# Select a random valid section for the Ego
egoLaneSec = Uniform(*laneSecsWithLeftLane)
advLaneSec = egoLaneSec.laneToLeft

# Define spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
# Adversary starts ahead in the left lane
advSpawnPt = new OrientedPoint following roadDirection from (advLaneSec.centerline.project(egoSpawnPt.position)) for globalParameters.INITIAL_GAP

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdvBehavior(egoLaneSec)

# Requirements to ensure a valid highway setup
require distance to intersection >= 50
require (distance from ego to adversary) > 20

# Terminate after the interaction
terminate when (distance from ego to egoSpawnPt) > 200