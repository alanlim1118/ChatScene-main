'''While performing a maneuver, the ego-vehicle encounters a stopped vehicle in the road and must perform an emergency brake or an avoidance maneuver.'''
Town = 'Town01'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    while (distance to self) > 60:
        wait  # The adversarial vehicle waits until it is sufficiently close to pose a threat.
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory) until (distance from self to egoTrajectory) < globalParameters.OPT_BRAKE_DISTANCE
    # Executes a sudden braking maneuver once it is within a critical distance of the ego's trajectory.
    while True:
        take SetBrakeAction(globalParameters.OPT_BRAKE)  # Initiates a hard brake to simulate abrupt stopping in the intersection.

param OPT_ADV_SPEED = Range(5, 15)  # The speed at which the adversarial vehicle approaches the intersection.
param OPT_BRAKE_DISTANCE = Range(0, 4)  # The critical distance at which the adversarial begins to brake.
param OPT_BRAKE = Range(0, 1)  # The intensity of the braking action, ranging from a slight decrease in speed to a full stop.
# Collecting lane sections that have a left lane (opposite traffic direction) and no right lane (single forward road)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToRight is None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithLeftLane.append(laneSec)

# Selecting a random lane section that matches the criteria
egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = OrientedPoint in egoLaneSec.centerline

# Ego vehicle setup
ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
param OPT_GEO_Y_DISTANCE = Range(10, 30)  # Frontal distance range

FrontSpawnPtOpp = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Car at FrontSpawnPtOpp,
    with heading FrontSpawnPtOpp.heading + 180 deg,  # Opposite direction to the car
    with regionContainedIn None,
    with behavior AdvBehavior()