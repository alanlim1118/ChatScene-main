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

param OPT_EGO_SPEED = Range(3, 6)
param OPT_EGO_BRAKE_DIST = Range(10, 20)
param OPT_ADV_DISTANCE = Range(20, 35)
param OPT_ADV_THROTTLE = Range(0.3, 0.6)
param OPT_ADV_STEER = Range(-0.5, -0.3)  # Leftward steer while reversing

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, brake_dist):
        take SetBrakeAction(1)
        take SetThrottleAction(0)
    terminate

behavior ReverseCurveLeftBehavior(throttle, steer):
    # Continuously reverse with leftward steering to curve toward the adjacent upper lane
    while True:
        take SetReverseAction(True)
        take SetSteerAction(steer)
        take SetThrottleAction(throttle)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a forward lane section that has an adjacent lane to the left (upper lane)
eligibleLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None:
            eligibleLaneSecs.append(laneSec)

egoLaneSec = Uniform(*eligibleLaneSecs)

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
advSpawnPt = new OrientedPoint following egoLaneSec.orientation from egoSpawnPt for globalParameters.OPT_ADV_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue ego vehicle traveling straight in the lower lane
ego = new Car at egoSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with color Color(0, 0, 1),
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_EGO_BRAKE_DIST)

# Pink adversarial vehicle directly ahead, reversing to the left toward adjacent upper lane
AdvAgent = new Car at advSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn egoLaneSec,
    with color Color(1, 0.4, 0.7),
    with behavior ReverseCurveLeftBehavior(globalParameters.OPT_ADV_THROTTLE, globalParameters.OPT_ADV_STEER)

require (distance to intersection) >= 60