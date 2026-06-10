"""Scenario Description:

The other vehicle suddenly merges in front of the ego vehicle.
The ego vehicle is driving at a steady speed in its lane, while an adversarial vehicle 
in an adjacent lane accelerates, pulls ahead, and performs a sudden lane change 
into the ego vehicle's lane.

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
ADV_MODEL = "vehicle.audi.tt"

EGO_SPEED = Range(8, 12)
ADV_SPEED = EGO_SPEED + Range(3, 5)

# Distance ahead of ego to initiate the merge
MERGE_TRIGGER_DIST = Range(10, 15)

# Weather setup
WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon', 'ClearSunset']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyObjs(self, 8):
        # Emergency braking if the adversarial car merges too close
        take SetBrakeAction(1.0)

behavior AdvMergeBehavior(target_lane_sec, speed, trigger_dist):
    # Drive in the starting lane until it is sufficiently ahead of the ego
    do FollowLaneBehavior(target_speed=speed) until (distance from self to ego > trigger_dist)
    
    # Perform the sudden lane change into the ego's lane
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=speed)
    
    # Continue driving after the merge
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a lane to their left for the adversarial vehicle to start in
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward and 
            laneSec.laneToLeft is not None and 
            laneSec.laneToLeft.isForward):
            laneSecsWithLeftLane.append(laneSec)

# Select a random valid lane section for the ego
egoLaneSec = Uniform(*laneSecsWithLeftLane)
# The adversary will start in the lane to the left
advLaneSec = egoLaneSec.laneToLeft

# Define spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
# Adversary starts slightly behind or at the same level to demonstrate the pull-ahead and merge
advSpawnPt = new OrientedPoint in advLaneSec.centerline,
                at egoSpawnPt offset along (egoSpawnPt.heading - 90 deg) by 3.5, # side offset
                offset along egoSpawnPt.heading by Range(-5, 0)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED)

adv = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdvMergeBehavior(egoLaneSec, ADV_SPEED, MERGE_TRIGGER_DIST)

#################################
# CONSTRAINTS & TERMINATION     #
#################################

# Ensure we are on a straight stretch of road and not in an intersection
require distance to intersection >= 50
require (distance from ego to adv) < 15

# Terminate when the maneuver is complete and distance increases
terminate when (distance from ego to adv) > 50 and (ego.speed > 0)