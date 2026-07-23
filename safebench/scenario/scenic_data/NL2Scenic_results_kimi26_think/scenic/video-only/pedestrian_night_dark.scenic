"""Scenario Description:

The ego vehicle travels along a dark, unlit highway at night, maintaining a steady course until a pedestrian wearing light-colored clothing suddenly emerges from the left side of the road. The individual is walking across the lanes and remains invisible until illuminated by the ego vehicle's headlights, creating a critical hazard. As the pedestrian crosses directly in front of the car from left to right, the ego vehicle is forced into a sudden emergency braking and swerving maneuver to avoid a collision. The pedestrian successfully crosses the vehicle's path and moves toward the right shoulder, disappearing from view as the ego vehicle continues forward on the highway.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town04'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

param HIGHWAY_SPEED = Range(12, 18)       # Ego speed (m/s)
param PED_TRIGGER_DIST = Range(35, 50)    # Distance at which pedestrian begins to cross
param BRAKE_DIST = Range(20, 30)          # Distance at which ego reacts
param PED_SPEED = Range(1.2, 1.8)         # Pedestrian walking speed (m/s)
param SWERVE_STEER = Range(0.3, 0.6)      # Steering input for emergency swerve
param PED_SPAWN_DISTANCE = Range(30, 50)  # Distance ahead to place pedestrian
param PED_LATERAL_OFFSET = Range(2, 4)    # Lateral offset from left lane edge

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    try:
        do FollowLaneBehavior(globalParameters.HIGHWAY_SPEED)
    interrupt when withinDistanceToAnyPedestrians(self, globalParameters.BRAKE_DIST):
        while withinDistanceToAnyPedestrians(self, 8):
            take SetThrottleAction(0), SetBrakeAction(1), SetSteerAction(globalParameters.SWERVE_STEER)

behavior CrossFromLeft():
    while distance from self to ego > globalParameters.PED_TRIGGER_DIST:
        wait
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(globalParameters.PED_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a long straight road to serve as the highway
highwayRoad = Uniform(*filter(lambda r: r.length > 200, network.roads))
egoLane = Uniform(*highwayRoad.lanes)

# Ego spawn point on the highway
egoStart = new OrientedPoint on egoLane.centerline

# Point ahead where the pedestrian emerges from the left
aheadPt = new OrientedPoint following egoLane.orientation from egoStart for globalParameters.PED_SPAWN_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# Nighttime setting
param weather = "ClearNight"

ego = new Car at egoStart,
    with regionContainedIn None,
    with behavior EgoBehavior()

# Pedestrian emerges from the left side and crosses left-to-right
ped = new Pedestrian left of aheadPt by globalParameters.PED_LATERAL_OFFSET,
    facing -90 deg relative to aheadPt,
    with regionContainedIn None,
    with behavior CrossFromLeft()

# Ensure the hazard is placed ahead of the ego
require (distance from ego to aheadPt) > 0

terminate when (distance from ego to ped) > 60
terminate after 30 seconds