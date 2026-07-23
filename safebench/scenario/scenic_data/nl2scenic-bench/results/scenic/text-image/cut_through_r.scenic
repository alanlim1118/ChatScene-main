"""Scenario Description:

In a top-down schematic view of a three-lane road, a blue ego vehicle travels straight forward in the center lane, indicated by a straight blue arrow pointing ahead. To its right and slightly behind, a pink adversarial vehicle executes a cut-through maneuver, crossing directly in front of the ego vehicle's path from the right lane towards the left lane, as illustrated by a curved pink trajectory arrow that intersects the center lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
from scenic.domains.driving.controllers import *

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(6, 8)
param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(1.1, 1.3)
param OPT_CUT_TRIGGER_DIST = Range(15, 25)  # Distance at which adv begins cut-in
param OPT_BRAKE_DISTANCE = Range(5, 8)     # Ego braking threshold

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, globalParameters.OPT_BRAKE_DISTANCE):
        take SetThrottleAction(0)
        take SetBrakeAction(1)

behavior CutInBehavior(target_speed, trigger_dist):
    # Follow lane until close enough to ego, then execute lane change to the left
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to ego < trigger_dist)
    do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToLeft, target_speed=target_speed)
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have both a left and right neighbor (i.e., center lane of 3+ lane road)
centerLaneSections = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToLeft is not None 
            and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is not None 
            and laneSec._laneToRight.isForward):
            centerLaneSections.append(laneSec)

egoLaneSec = Uniform(*centerLaneSections)
rightLaneSec = egoLaneSec._laneToRight

# Ego spawn point in center lane
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Adversary spawn point in right lane, slightly behind ego
rightLaneProj = rightLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from rightLaneProj for Range(-20, -10)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in center lane traveling straight
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color (0, 0, 1, 1),  # Blue
    with behavior EgoBehavior()

# Adversarial vehicle in right lane, slightly behind, executing cut-in to the left
AdvAgent = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn rightLaneSec,
    with blueprint ADV_MODEL,
    with color (1, 0.4, 0.7, 1),  # Pink
    with behavior CutInBehavior(globalParameters.OPT_ADV_SPEED, globalParameters.OPT_CUT_TRIGGER_DIST)

require distance to intersection >= 100  # Keep scenario away from intersections