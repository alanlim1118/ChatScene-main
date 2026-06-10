'''The ego-vehicle encounters an obstacle / unexpected entity on the road and must perform an emergency brake or an avoidance maneuver.'''
Town = 'Town03'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    while (distance to self) > 60:
        wait  # The adversarial vehicle waits until it is within a significant range to impact the ego's maneuver.
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory) until (distance from self to egoTrajectory) < globalParameters.OPT_BRAKE_DISTANCE
    # Once within a critical distance of the ego's trajectory, the adversarial vehicle initiates a sudden braking maneuver.
    while True:
        take SetBrakeAction(globalParameters.OPT_BRAKE)  # Continues to apply brakes to simulate an emergency or unexpected stop.

param OPT_ADV_SPEED = Range(5, 15)  # The speed at which the adversarial vehicle approaches the intersection.
param OPT_BRAKE_DISTANCE = Range(0, 4)  # The critical distance at which the adversarial vehicle begins its sudden stop.
param OPT_BRAKE = Range(0, 1)  # The intensity of the braking action, potentially simulating an abrupt or gentle stop.
# Identifying lane sections with no adjacent lanes
laneSecsWithNoAdjacent = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is None and laneSec._laneToRight is None:
            laneSecsWithNoAdjacent.append(laneSec)

# Selecting a random lane section from identified sections for the ego vehicle
egoLaneSec = Uniform(*laneSecsWithNoAdjacent)
egoSpawnPt = OrientedPoint in egoLaneSec.centerline

# Ego vehicle setup
ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
param OPT_GEO_Y_DISTANCE = Range(10, 30)  # Frontal distance range

FrontSpawnPtOpp = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Unknown (The scenario does not specify the type of adversarial object, so it cannot be restricted to Car, Pedestrian, Bicycle, or Motorcycle.) at FrontSpawnPtOpp,
    with heading FrontSpawnPtOpp.heading + 180 deg,  # Opposite direction to the car
    with regionContainedIn None,
    with behavior AdvBehavior()