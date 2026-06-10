"""Scenario Description:
Vehicle is passing another vehicle in a rural area, in daylight, under clear weather conditions, 
at a non-junction with a posted speed limit of 55 mph or more; 
and encroaches into another vehicle traveling in the opposite direction.
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

# 55 mph is approximately 24.58 m/s
EGO_SPEED = Range(24.5, 26)
LEAD_SPEED = Range(12, 15)  # Slower vehicle being passed
ONCOMING_SPEED = Range(22, 25)

OVERTAKE_START_DIST = Range(15, 20)
INITIAL_LEAD_DIST = Range(30, 40)
INITIAL_ONCOMING_DIST = Range(150, 200)

WEATHER_OPTIONS = ['ClearNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoOvertakingBehavior(target_lane_sec, lead_car):
    # Drive behind the lead car
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED) until (distance from self to lead_car < OVERTAKE_START_DIST)
    except CollisionViolation: # Handle if lead car is too slow
        pass

    # Start the encroachment (overtaking into opposite lane)
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, is_oppositeTraffic=True, target_speed=EGO_SPEED)
    
    # Continue driving in the opposite lane to represent the encroachment/conflict
    do FollowLaneBehavior(target_speed=EGO_SPEED, is_oppositeTraffic=True)

behavior OncomingBehavior():
    do FollowLaneBehavior(target_speed=ONCOMING_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find bidirectional road sections suitable for overtaking (not in intersections)
laneSecsWithOpposite = []
for lane in network.lanes:
    for laneSec in lane.sections:
        # Check if there is an opposite lane group and we aren't too close to an intersection
        if laneSec.group.opposite is not None and not laneSec.road.intersections:
            # Check if road length is sufficient for a passing maneuver
            if laneSec.road.centerline.length > 300:
                laneSecsWithOpposite.append(laneSec)

require len(laneSecsWithOpposite) > 0
egoLaneSec = Uniform(*laneSecsWithOpposite)

# Get a corresponding lane section on the opposite side
oppLaneSec = Uniform(*egoLaneSec.group.opposite.sections)

# Define spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for INITIAL_LEAD_DIST

# Oncoming vehicle spawns far ahead in the opposite lane
oncomingSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for INITIAL_ONCOMING_DIST,
    with heading egoSpawnPt.heading + 180 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

lead_car = new Car at leadSpawnPt,
    with behavior FollowLaneBehavior(target_speed=LEAD_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with behavior EgoOvertakingBehavior(oppLaneSec, lead_car)

oncoming_car = new Car at oncomingSpawnPt,
    with behavior OncomingBehavior()

# Ensure we are not starting at an intersection
require (distance to intersection) > 50

# Safety requirements to ensure the scenario sets up correctly
require (distance from ego to oncoming_car) > 100
require (distance from lead_car to oncoming_car) > 80

# Terminate after ego has completed a significant portion of the road or passed oncoming
terminate when (distance from ego to egoSpawnPt) > 250