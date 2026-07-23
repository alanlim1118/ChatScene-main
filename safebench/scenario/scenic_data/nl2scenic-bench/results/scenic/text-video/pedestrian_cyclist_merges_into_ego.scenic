"""Scenario Description:

The ego vehicle travels along a straight, two-lane rural road under clear, sunny skies, passing houses and trees on either side. A bicyclist is initially observed riding along the right shoulder in the same direction as traffic. Suddenly, the cyclist turns sharply to the left, cutting directly across the ego vehicle's path without checking for oncoming traffic. This abrupt maneuver forces the ego vehicle into a sudden emergency braking and swerving scenario, causing the speed to drop rapidly from roughly 54 km/h to 15 km/h, and results in a side or t-bone collision as the cyclist crosses the lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

# Ego speed ~54 km/h = 15 m/s
EGO_SPEED = 15.0
# Post-braking target ~15 km/h = 4.17 m/s (used as threshold reference)
CYCLIST_SPEED = Range(3.0, 5.0)
CROSS_TRIGGER_DISTANCE = Range(25, 35)
BRAKE_DIST = Range(8, 12)
SHOULDER_OFFSET = Range(1.5, 2.5)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior CyclistRideThenCutBehavior(trigger_distance, cross_speed):
    """Cyclist rides along shoulder then cuts sharply left across ego's path."""
    do FollowLaneBehavior(speed=cross_speed * 0.6) until (distance from self to ego <= trigger_distance)
    # Sharp left turn across the road
    take SetSteeringAction(-1.0)
    take SetThrottleAction(1.0)
    do FollowHeadingBehavior(self.heading - 90 deg, speed=cross_speed) for 3 seconds
    take SetBrakeAction(0.5)
    do WaitBehavior()

behavior EgoEmergencyBrakeBehavior(trajectory, brake_dist):
    """Ego follows trajectory but brakes hard when cyclist is detected nearby."""
    try:
        do FollowTrajectoryBehavior(target_speed=EGO_SPEED, trajectory=trajectory)
    interrupt when (withinDistanceToObjsInLane(self, brake_dist)):
        take SetThrottleAction(0)
        take SetBrakeAction(1.0)
        take SetSteeringAction(Uniform(-0.3, 0.3))  # Slight swerve attempt
        do WaitBehavior() for 5 seconds
        abort
    terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment suitable for rural two-lane driving
straightRoads = filter(lambda s: len(s.lanes) >= 2 and s.length > 100, network.roadSections)
roadSection = Uniform(*straightRoads)

egoLane = Uniform(*filter(lambda l: l.isForward, roadSection.lanes))
shoulderLane = egoLane._laneToRight if egoLane._laneToRight is not None else egoLane

egoTrajectory = [egoLane]
egoSpawnPt = new OrientedPoint in egoLane.centerline

# Place cyclist ahead on the right shoulder
cyclistLongitudinalOffset = Range(40, 60)
cyclistBasePt = new OrientedPoint following roadDirection from egoSpawnPt for cyclistLongitudinalOffset
cyclistProjectPt = shoulderLane.centerline.project(cyclistBasePt.position)
cyclistHeading = shoulderLane.orientation[cyclistProjectPt]
cyclistSpawnPt = new OrientedPoint at cyclistProjectPt offset by SHOULDER_OFFSET@-90 deg,
    facing cyclistHeading

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoEmergencyBrakeBehavior(egoTrajectory, BRAKE_DIST),
    with speed EGO_SPEED

cyclist = new Bicycle at cyclistSpawnPt,
    with heading cyclistHeading,
    with regionContainedIn None,
    with behavior CyclistRideThenCutBehavior(CROSS_TRIGGER_DISTANCE, CYCLIST_SPEED)

require distance from egoSpawnPt to cyclistSpawnPt >= 30
require distance from egoSpawnPt to cyclistSpawnPt <= 80
terminate when (distance from ego to cyclist) < 2 or (elapsedTime > 15)