"""Scenario Description:

On a road marked by a central dashed white line, a blue ego vehicle travels straight forward within the lower lane. Directly ahead in the same lane, a pink adversarial vehicle executes a maneuver described as "object exiting parallel reversing to the left," visually depicted by a purple trajectory line that shows the vehicle backing up and curving upward toward the adjacent upper lane.

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

param OPT_ADV_DISTANCE = Range(20, 35)
param OPT_ADV_THROTTLE = Range(0.3, 0.6)
param OPT_ADV_STEER = Range(-0.5, -0.3)  # Leftward steer while reversing

#################################
# AGENT BEHAVIORS               #
#################################

behavior ReverseCurveLeftBehavior(throttle, steer):
    # Continuously reverse with leftward steering to curve toward the adjacent upper lane
    while True:
        take SetReverseAction(True)
        take SetSteerAction(steer)
        take SetThrottleAction(throttle)

#################################
# SPATIAL RELATIONS             #
#################################

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing (-(yaw + 90) deg)
egoLaneSec = network.laneSectionAt(egoSpawnPt)

advSpawnPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for globalParameters.OPT_ADV_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue ego vehicle traveling straight in the lower lane
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color Color(0, 0, 1)

# Pink adversarial vehicle directly ahead, reversing to the left toward adjacent upper lane
AdvAgent = new Car at advSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with color Color(1, 0.4, 0.7),
    with behavior ReverseCurveLeftBehavior(globalParameters.OPT_ADV_THROTTLE, globalParameters.OPT_ADV_STEER)

require (distance to intersection) >= 60
