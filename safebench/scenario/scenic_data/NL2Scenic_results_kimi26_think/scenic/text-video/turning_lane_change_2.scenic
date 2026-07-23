"""Scenario Description:

The ego vehicle travels forward in the left lane of a busy urban road beneath a large elevated concrete structure during the late afternoon, maintaining a speed of approximately 55 km/h. To the right, commercial buildings and heavy traffic are visible. Suddenly, a white passenger sedan from the far right lanes aggressively cuts across two lanes of traffic, disregarding the flow of vehicles. The white car swerves directly into the ego vehicle's path, resulting in a sudden side-impact collision as the ego vehicle is unable to avoid the intruding vehicle.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.tesla.model3"  # Passenger sedan

param OPT_EGO_SPEED = 15.5            # ~55 km/h in m/s
param OPT_ADV_SPEED = Range(16, 20)   # Aggressively faster than ego
param OPT_ADV_BEHIND_DIST = Range(10, 20)
param OPT_TRIGGER_DIST = Range(20, 35)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoCruiseBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior AggressiveCutInBehavior(middle_lane, left_lane, adv_speed):
    try:
        do FollowLaneBehavior(target_speed=adv_speed)
    interrupt when (distance from self to ego <= globalParameters.OPT_TRIGGER_DIST):
        # Aggressively swerve across two lanes into the ego's lane
        do LaneChangeBehavior(laneSectionToSwitch=middle_lane, target_speed=adv_speed)
        do LaneChangeBehavior(laneSectionToSwitch=left_lane, target_speed=adv_speed)
        do FollowLaneBehavior(target_speed=adv_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a leftmost lane of a 3-lane forward section
laneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward and 
            laneSec._laneToLeft is None and 
            laneSec._laneToRight is not None and 
            laneSec._laneToRight._laneToRight is not None and 
            laneSec._laneToRight._laneToRight._laneToRight is None):
            laneSecs.append(laneSec)

egoLaneSec = Uniform(*laneSecs)
middleLaneSec = egoLaneSec._laneToRight
rightLaneSec = middleLaneSec._laneToRight

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Project ego position onto the far right lane and place adversary behind
rightLaneProjPt = rightLaneSec.centerline.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following roadDirection from rightLaneProjPt for -globalParameters.OPT_ADV_BEHIND_DIST

# Background traffic points to simulate a busy road
middleLaneProjPt = middleLaneSec.centerline.project(egoSpawnPt.position)
bgMiddleSpawn = new OrientedPoint following roadDirection from middleLaneProjPt for 50
bgRightSpawn = new OrientedPoint following roadDirection from rightLaneProjPt for 70

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle: left lane, steady speed
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoCruiseBehavior(globalParameters.OPT_EGO_SPEED)

# Adversarial white sedan: far right lane, cuts across two lanes into ego
adv = new Car at advSpawnPt,
    with heading egoSpawnPt.heading,
    with regionContainedIn None,
    with blueprint ADV_MODEL,
    with color (1, 1, 1),
    with behavior AggressiveCutInBehavior(middleLaneSec, egoLaneSec, globalParameters.OPT_ADV_SPEED)

# Background traffic in middle and right lanes to create busy road context
bgCarMiddle = new Car at bgMiddleSpawn,
    with regionContainedIn middleLaneSec,
    with behavior FollowLaneBehavior(target_speed=12)

bgCarRight = new Car at bgRightSpawn,
    with regionContainedIn rightLaneSec,
    with behavior FollowLaneBehavior(target_speed=12)

# Ensure we are on a straight stretch away from intersections
require distance to intersection >= 80

terminate after 20 seconds