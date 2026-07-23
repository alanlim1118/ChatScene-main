"""Scenario Description:

Vehicle is going straight in an urban area, in daylight, under clear weather conditions, at a non-junction with a posted speed limit of 55 mph or more; and closes in on a lead vehicle moving at lower constant speed.

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

# Speed parameters (m/s): 55 mph ≈ 24.6 m/s
MIN_SPEED_LIMIT = 24.6
EGO_SPEED = Range(25, 30)        # Ego drives above the speed limit
LEAD_SPEED = Range(15, 20)       # Lead vehicle at lower constant speed
INITIAL_GAP = Range(80, 120)     # Initial distance between ego and lead vehicle
BRAKE_DISTANCE = Range(15, 25)   # Distance at which ego begins to brake

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadVehicleBehavior(speed):
    while True:
        do FollowLaneBehavior(target_speed=speed)

behavior EgoClosingBehavior(speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(0.8)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a non-junction road segment with sufficient speed limit
# Filter for lanes that are not part of any intersection and have adequate speed
nonJunctionLanes = filter(
    lambda l: not any(l.intersects(i.region) for i in network.intersections),
    network.lanes
)

# Prefer longer straight segments for closing-in scenario
longLanes = filter(lambda l: len(l.centerline) > 150, nonJunctionLanes)
egoLane = Uniform(*longLanes) if longLanes else Uniform(*nonJunctionLanes)

# Spawn points along the lane centerline
egoSpawnPt = new OrientedPoint in egoLane.centerline
leadSpawnPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for INITIAL_GAP

#################################
# WEATHER AND TIME              #
#################################

param weather = 'ClearNoon'

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with behavior EgoClosingBehavior(EGO_SPEED, BRAKE_DISTANCE)

lead_vehicle = new Car at leadSpawnPt,
    with behavior LeadVehicleBehavior(LEAD_SPEED)

# Ensure both vehicles remain on the same lane segment
require egoLane is lead_vehicle.lane

# Terminate when ego has closed in significantly or passed the lead vehicle
terminate when distance from ego to lead_vehicle < 5
terminate when simulation().currentTime > 30