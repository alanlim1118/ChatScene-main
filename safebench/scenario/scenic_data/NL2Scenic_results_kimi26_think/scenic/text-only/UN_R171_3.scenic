"""Scenario Description:

The ego vehicle initiates a driver-requested lane change on a multi-lane road, but must detect and yield to a high-speed vehicle approaching from the rear in the target lane, delaying the maneuver until the overtaking vehicle has safely passed.

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

EGO_MODEL = 'vehicle.lincoln.mkz_2017'

param EGO_SPEED = Range(5, 7)
param ADV_SPEED = Range(12, 15)      # High-speed overtaking vehicle
param SAFE_GAP = Range(25, 35)      # Minimum clearance to target-lane vehicle before lane change
param ADV_SPAWN_DIST = Range(10, 20) # Spawn distance behind ego in the target lane

INIT_DIST = 60
TERM_TIME = 8

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_lane_sec, ego_speed, safe_gap):
    # Wait until the high-speed vehicle in the target lane has safely passed
    do FollowLaneBehavior(target_speed=ego_speed) until (distance to adversary) > safe_gap
    do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, target_speed=ego_speed)
    do FollowLaneBehavior(target_speed=ego_speed) for TERM_TIME seconds
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a forward lane that has an adjacent lane to the left (the target lane)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
targetLaneSec = egoLaneSec._laneToLeft
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place the adversary in the target lane, behind the ego
targetLanePt = targetLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from targetLanePt for -globalParameters.ADV_SPAWN_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(targetLaneSec, globalParameters.EGO_SPEED, globalParameters.SAFE_GAP)

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn targetLaneSec,
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

require (distance to intersection) > INIT_DIST
require (distance from adversary to intersection) > INIT_DIST