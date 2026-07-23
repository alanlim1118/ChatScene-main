"""Scenario Description:

This traffic scenario illustrates an oncoming situation on a straight road divided into two lanes by a dashed center line. At the start of the sequence, labeled "@oncoming start," a green ego vehicle is positioned in the top lane traveling to the left, while a red vehicle is in the bottom lane traveling to the right, approaching the ego vehicle from the opposite direction. The sequence concludes at the "@oncoming end" stage, where the red vehicle has successfully passed the green ego vehicle, and both cars continue driving straight in their respective lanes away from each other.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town01'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(5, 10)
param OPT_START_DIST = Range(30, 50)
param OPT_TERM_DIST = 60

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
advLaneSec = egoLaneSec._laneToLeft

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint in advLaneSec.centerline

# Ensure the adversary is ahead of the ego so that they approach each other
aheadPt = new OrientedPoint following roadDirection from egoSpawnPt for 1
behindPt = new OrientedPoint following roadDirection from egoSpawnPt for -1

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

# Adversary car setup
adversary = new Car at advSpawnPt,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with behavior AdvBehavior()

# Constraints to create an oncoming encounter
require (distance from egoSpawnPt to advSpawnPt) > globalParameters.OPT_START_DIST
require (distance from egoSpawnPt to advSpawnPt) < globalParameters.OPT_START_DIST + 20
require (distance from advSpawnPt to aheadPt) < (distance from advSpawnPt to behindPt)

# Terminate once the vehicles have passed and moved apart
terminate when (distance from ego to adversary) > globalParameters.OPT_TERM_DIST