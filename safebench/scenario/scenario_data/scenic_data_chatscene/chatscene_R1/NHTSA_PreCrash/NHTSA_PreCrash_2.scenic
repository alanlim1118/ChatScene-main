'''Vehicle is going straight in a rural area, in daylight, under adverse weather conditions, with a posted speed limit of 55 mph or more, and then loses control due to wet or slippery roads and runs off the road'''
Town = 'Town03'
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
AdvAgent = Car at FrontSpawnPtOpp,
    with heading FrontSpawnPtOpp.heading + 180 deg,  # Opposite direction to the car
    with regionContainedIn None,
    with behavior AdvBehavior()