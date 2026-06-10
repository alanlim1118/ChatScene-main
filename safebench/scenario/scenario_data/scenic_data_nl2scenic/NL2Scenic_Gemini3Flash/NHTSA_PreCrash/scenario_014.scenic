"""Scenario Description:
Vehicle is changing lanes in an urban area, in daylight, under clear weather conditions, 
at a non-junction with a posted speed limit of 55 mph or more (approx 24.5 m/s); 
and then encroaches into another vehicle traveling in the same direction.
"""

#################################
# MAP AND MODEL                 #
#################################

# Town06 contains long many-lane highways with high speed limits (90-100 km/h), 
# which fits the 55+ mph requirement.
Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
ADV_MODEL = "vehicle.audi.etron"

# 55 mph is approximately 24.5 m/s.
param EGO_SPEED = Range(25, 30)
param ADV_SPEED = Range(24, 26)

# Distance at which the ego vehicle initiates the encroaching lane change
param ENCROACH_DISTANCE = Range(10, 15)

WEATHER_OPTIONS = ['ClearNoon']
param weather = Uniform(*WEATHER_OPTIONS)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_lane_sec):
    try:
        # Drive forward until getting close to the target vehicle to simulate encroachment
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) \
            until (distance from self to AdvAgent < globalParameters.ENCROACH_DISTANCE)
        
        # Initiate lane change into the lane where AdvAgent is traveling
        do LaneChangeBehavior(laneSectionToSwitchTo=target_lane_sec, target_speed=globalParameters.EGO_SPEED)
        
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    
    interrupt when withinDistanceToObjsInLane(self, 5):
        # Emergency braking if a collision is imminent during the encroachment
        take SetBrakeAction(1.0)

behavior AdvBehavior():
    # Maintain steady speed in the target lane
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find road sections that have an adjacent lane to the left for a lane change
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        # Filter for forward lanes with a left adjacent lane (same direction)
        if laneSec.isForward and laneSec.laneToLeft is not None and laneSec.laneToLeft.isForward:
            # Ensure it's not in an intersection
            if not laneSec.road.intersection:
                laneSecsWithLeftLane.append(laneSec)

# Select a random valid section
egoLaneSec = Uniform(*laneSecsWithLeftLane)
targetLaneSec = egoLaneSec.laneToLeft

# Define spawn points
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline
# Spawn the adversary slightly ahead in the left lane
advSpawnPt = new OrientedPoint in targetLaneSec.centerline,
    beyond egoSpawnPt by Range(20, 30)

#################################
# SCENARIO SPECIFICATION        #
#################################

AdvAgent = new Car at advSpawnPt,
    with blueprint ADV_MODEL,
    with behavior AdvBehavior()

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(targetLaneSec)

# Constraints to ensure the scenario starts correctly
require (distance to intersection) > 50
require (distance from AdvAgent to intersection) > 50

# Terminate after some time or if the lane change is completed
terminate when (distance from ego to egoSpawnPt) > 150