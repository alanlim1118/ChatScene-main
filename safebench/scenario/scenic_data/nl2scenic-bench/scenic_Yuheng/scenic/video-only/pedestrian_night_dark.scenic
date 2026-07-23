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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
PEDESTRIAN_MODEL = "walker.pedestrian.0014"  # Light-colored clothing pedestrian

param EGO_SPEED = Range(8, 12)               # Highway speed in m/s
param PED_WALK_SPEED = Range(1.2, 1.8)       # Walking speed across road
param BRAKE_TRIGGER_DIST = Range(18, 25)     # Distance to start emergency braking
param SWERVE_OFFSET = Range(1.5, 2.5)        # Lateral offset for swerve maneuver
param PED_SPAWN_LATERAL = Range(4, 6)        # How far left of lane center pedestrian starts
param PED_CROSS_DISTANCE = Range(30, 40)     # Distance ahead where pedestrian begins crossing

#################################
# AGENT BEHAVIORS               #
#################################

behavior EmergencyBrakeAndSwerve():
    """Ego follows lane then performs emergency brake + swerve when pedestrian is close."""
    try:
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)
    interrupt when withinDistanceToAnyPedestrians(self, globalParameters.BRAKE_TRIGGER_DIST):
        take SetThrottleAction(0), SetBrakeAction(1), SetSteerAction(-0.3)
        do FollowLaneBehavior(target_speed=0) for 3 seconds
        take SetBrakeAction(0), SetSteerAction(0)
        do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

behavior CrossLeftToRight(ego_ref, trigger_dist, walk_speed):
    """Pedestrian waits until ego is near, then crosses from left to right."""
    while distance from self to ego_ref > trigger_dist:
        wait
    # Walk perpendicular to road heading (left to right relative to ego)
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(walk_speed)
    # Continue walking until well past the ego's path
    do CrossingBehavior(ego_ref, walk_speed, 15)
    take SetWalkingSpeedAction(0)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Select a straight highway segment
highwaySegment = Uniform(*filter(lambda s: s.isHighway and len(s.lanes) >= 2, network.roads))
egoLane = Uniform(*highwaySegment.lanes)

egoSpawnPt = new OrientedPoint in egoLane.centerline

ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with regionContainedIn None,
    with behavior EmergencyBrakeAndSwerve()

# Place pedestrian on the left side of the ego's lane, ahead at crossing distance
pedSpawnBase = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.PED_CROSS_DISTANCE
pedSpawnPt = new OrientedPoint left of pedSpawnBase by globalParameters.PED_SPAWN_LATERAL,
    with heading pedSpawnBase.heading - 90 deg  # Facing right (across the road)

pedestrian = new Pedestrian at pedSpawnPt,
    with blueprint PEDESTRIAN_MODEL,
    with regionContainedIn None,
    with behavior CrossLeftToRight(ego, globalParameters.PED_CROSS_DISTANCE, globalParameters.PED_WALK_SPEED)

# Ensure scenario runs long enough for full interaction
terminate after 45 seconds