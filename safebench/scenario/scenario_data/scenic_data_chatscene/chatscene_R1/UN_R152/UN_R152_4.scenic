'''the subject vehicle changes the lane in front of the signboard which is positioned in the centre of the lane and notifies the driver that the lane is reduced'''
Town = 'Town03'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    while (distance to self) > 60:
        wait  # Wait until the vehicle is close enough to influence the ego's path.
    do FollowTrajectoryBehavior(globalParameters.OPT_ADV_SPEED, advTrajectory) until (distance from self to egoTrajectory) < globalParameters.OPT_ADV_DISTANCE
    # Executes a left turn maneuver.
    take SetSteerAction(globalParameters.OPT_STEER)  # Full left turn
    while True:
        take SetSpeedAction(0)  # Eventually stops after completing the turn

param OPT_ADV_SPEED = Range(5, 15)  # Speed at which the vehicle approaches the intersection.
param OPT_ADV_DISTANCE = Range(0, 5)  # The critical distance to start the turn.
param OPT_STEER = Range(-1.0, 0.0)
# Identifying lane sections with a right lane moving in the same forward direction
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToRight is not None and laneSec._laneToRight.isForward == laneSec.isForward:
            laneSecsWithRightLane.append(laneSec)

# Selecting a random lane section from identified sections for the ego vehicle
egoLaneSec = Uniform(*laneSecsWithRightLane)
egoSpawnPt = OrientedPoint in egoLaneSec.centerline

# Ego vehicle setup
ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
param OPT_GEO_Y_DISTANCE = Range(10, 30)  # Frontal distance range

FrontSpawnPtOpp = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = None (There is no adversarial object in this scenario as described). at FrontSpawnPtOpp,
    with heading FrontSpawnPtOpp.heading + 180 deg,  # Opposite direction to the car
    with regionContainedIn None,
    with behavior AdvBehavior()