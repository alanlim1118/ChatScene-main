"""Scenario Description:

The ego vehicle and the lead vehicle travel at a high highway speed of 100 km/h in a synchronized flow 
until the lead vehicle suddenly exits the lane to reveal a stationary target (a stalled car).

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

# 100 km/h is approximately 27.78 m/s
TARGET_SPEED = 27.78

# Vehicle Models
EGO_MODEL = "vehicle.tesla.model3"
TARGET_MODEL = "vehicle.volkswagen.t2"
LEAD_MODEL = "vehicle.audi.etron"

# Distances
# GAP: Distance between Ego and Lead car
# REVEAL_DIST: Distance between Lead car and stationary target when Lead car starts lane change
# TARGET_OFFSET: Total distance from Ego's start to the stationary target
param GAP = Range(15, 20)
param REVEAL_DIST = Range(35, 45)
param TARGET_OFFSET = Range(100, 120)

WEATHER_OPTIONS = ['ClearNoon', 'CloudyNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadCarBehavior(reveal_dist, target_speed, escape_lane, obstacle):
    try:
        # Drive at highway speed in the same lane as the stationary obstacle
        do FollowLaneBehavior(target_speed=target_speed) until (distance from self to obstacle) < reveal_dist
        
        # Sudden lane change to reveal the obstacle to the ego vehicle behind
        do LaneChangeBehavior(laneSectionToSwitchTo=escape_lane, target_speed=target_speed)
        
        # Continue driving in the new lane
        do FollowLaneBehavior(target_speed=target_speed)
    except Exception:
        # Fallback to avoid collision if the lane change fails or another vehicle is in the way
        take SetBrakeAction(1)

behavior EgoBehavior(target_speed):
    # Ego vehicle follows the lane at highway speed
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Filter for highway lanes in Town06 that have adjacent lanes for an escape maneuver
lane_secs = []
for lane in network.lanes:
    for sec in lane.sections:
        # We look for forward-facing lanes on the highway (high speed limit)
        if sec.isForward and (lane.speedLimit is None or lane.speedLimit >= 20):
            # Must have a side lane to change into
            if (sec.laneToLeft and sec.laneToLeft.isForward) or (sec.laneToRight and sec.laneToRight.isForward):
                lane_secs.append(sec)

# Select a suitable highway section
egoLaneSec = Uniform(*lane_secs)

# Determine which side the lead vehicle will exit to
if egoLaneSec.laneToLeft and egoLaneSec.laneToLeft.isForward:
    escape_lane = egoLaneSec.laneToLeft.lane
else:
    escape_lane = egoLaneSec.laneToRight.lane

# Starting point for the ego vehicle
egoSpawnPt = OrientedPoint on egoLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# 1. The Stationary Target (stalled vehicle revealed by the lead car)
target_car = new Car at (egoSpawnPt offset along roadDirection by globalParameters.TARGET_OFFSET),
    with blueprint TARGET_MODEL,
    with color (1, 0, 0) # Red to indicate a hazard

# 2. The Ego Vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with speed TARGET_SPEED,
    with behavior EgoBehavior(TARGET_SPEED)

# 3. The Lead Vehicle (masking the target)
lead_car = new Car following roadDirection from ego for globalParameters.GAP,
    with blueprint LEAD_MODEL,
    with speed TARGET_SPEED,
    with behavior LeadCarBehavior(
        globalParameters.REVEAL_DIST, 
        TARGET_SPEED, 
        escape_lane, 
        target_car
    )

#################################
# CONSTRAINTS                   #
#################################

# Ensure there is enough road to reach high speed and perform the maneuver without intersections
require distance to intersection >= 100
require (distance from lead_car to target_car) > globalParameters.REVEAL_DIST + 10

# Terminate after the event is likely over
terminate when distance from ego to egoSpawnPt > globalParameters.TARGET_OFFSET + 50