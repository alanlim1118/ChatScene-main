'''A target vehicle in an adjacent lane performs a full lateral displacement of 3.5 meters to merge into the path of the ego vehicle with a lateral deviation of no more than 0.2 meters'''
Town = 'Town03'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED) until (
        distance to self < globalParameters.OPT_ADV_DISTANCE and
        len(network.laneSectionAt(self).adjacentLanes) != 0 )
    # Identify the lane to switch into
    weaveLaneSec = network.laneSectionAt(self).adjacentLanes[0]
    # Execute the lane change
    do LaneChangeBehavior(laneSectionToSwitch=weaveLaneSec, target_speed=globalParameters.OPT_ADV_SPEED)

param OPT_ADV_SPEED = Range(5, 10)
param OPT_ADV_DISTANCE = Range(0, 20)
# Identifying lane sections with both left and right lanes moving in the same forward direction
laneSecsWithBothSides = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward == laneSec.isForward) and 
           (laneSec._laneToRight is not None and laneSec._laneToRight.isForward == laneSec.isForward):
            laneSecsWithBothSides.append(laneSec)

# Selecting a random lane section from identified sections for the ego vehicle
egoLaneSec = Uniform(*laneSecsWithBothSides)
egoSpawnPt = OrientedPoint in egoLaneSec.centerline

# Ego vehicle setup
ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
# Setup the leading vehicle's spawn point directly in front of the ego to simulate a slow-moving vehicle
param OPT_LEADING_DISTANCE = Range(0, 30)
param OPT_LEADING_SPEED = Range(1, 5)
LeadingSpawnPt = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEADING_DISTANCE
LeadingAgent = Car at LeadingSpawnPt,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_LEADING_SPEED)

# Identifying the adjacent lane for the Adversarial Agent and setting its spawn point further in front
param OPT_GEO_Y_DISTANCE = Range(0, 30)
advLane = network.laneSectionAt(ego)._laneToRight.lane
IntSpawnPt = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE
projectPt = Vector(*advLane.centerline.project(IntSpawnPt.position).coords[0])
advHeading = advLane.orientation[projectPt]

# Spawn the Adversarial Agent
AdvAgent = Car at projectPt,
    with heading advHeading,
    with regionContainedIn None,
    with behavior AdvBehavior()