'''Vehicle is turning left or right at an intersection-related area, in daylight, under clear weather conditions, with a posted speed limit of 45 mph or less, and then loses control due to wet or slippery roads and runs off the road'''
Town = 'Town05'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    # Follow the lane at the speed defined by OPT_ADV_SPEED until a condition is met
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (
        distance to some_target < globalParameters.OPT_ADV_DISTANCE)
    
    # Perform a lane change to an adjacent lane
    targetLaneSec = network.laneSectionAt(self).adjacentLanes[0]
    do LaneChangeBehavior(laneSectionToSwitch=targetLaneSec, target_speed=globalParameters.OPT_ADV_SPEED)

    # Wait for a number of steps directly specified by OPT_WAIT_STEPS
    for _ in range(globalParameters.OPT_WAIT_STEPS):
        wait

    # Stop the vehicle after waiting
    do SetSpeedAction(0)

param OPT_ADV_SPEED = Range(5, 10)  # Speed range for the vehicle
param OPT_ADV_DISTANCE = Range(0, 20)  # Distance threshold for stopping the follow behavior
param OPT_WAIT_STEPS = Range(0, 20)  # Directly used range for wait steps before stopping
intersection = Uniform(*filter(lambda i: i.is4Way or i.is3Way, network.intersections))
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, intersection.maneuvers))
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]
egoSpawnPt = OrientedPoint in egoInitLane.centerline

ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
# Defining adversarial maneuvers as those conflicting with the ego's straight path
advManeuvers = filter(lambda i: i.type == ManeuverType.LEFT_TURN, egoManeuver.conflictingManeuvers)
advManeuver = Uniform(*advManeuvers)
advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]
advSpawnPt = advManeuver.connectingLane.centerline[0]  # Initial point on the connecting lane's centerline
IntSpawnPt = advManeuver.connectingLane.centerline.start  # Start of the connecting lane centerline

param OPT_GEO_Y_DISTANCE = Range(-10, 10)
# Setting up the adversarial agent
AdvAgent = Car following roadDirection from IntSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE,
    with heading IntSpawnPt.heading,
    with regionContainedIn None,
    with behavior AdvBehavior()

# Requirements to ensure the adversarial agent's relative position and trajectory are correctly aligned with the scenario's needs
require 70 deg <= RelativeHeading(AdvAgent) <= 110 deg  # Ensuring the agent approaches from an angle indicative of crossing
require any([AdvAgent.position in traj for traj in [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]])