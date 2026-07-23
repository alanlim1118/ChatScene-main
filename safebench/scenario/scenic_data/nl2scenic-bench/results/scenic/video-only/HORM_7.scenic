"""Scenario Description:

This top-down video captures a grey ego vehicle navigating a curved on-ramp to merge onto a straight, multi-lane road situated in a suburban area with houses to the left and a forest to the right. A red vehicle is traveling ahead in the left lane of the main road, positioned to the front-left of the ego vehicle. As the ego vehicle accelerates up the ramp, it merges into the right lane of the main road, positioning itself longitudinally behind the red car. The red car continues straight in its lane while the ego vehicle completes the merge, maintaining a safe following distance and proper lane alignment as it joins the traffic flow.

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
RED_CAR_MODEL = "vehicle.tesla.model3"

param OPT_EGO_SPEED = Range(8, 12)
param OPT_RED_CAR_SPEED = Range(10, 14)
param OPT_MERGE_DISTANCE = Range(30, 50)
param OPT_FOLLOWING_DISTANCE = Range(15, 25)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoMergeBehavior(target_speed, merge_distance):
    """Ego follows the ramp and merges into the right lane of the main road."""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when (distance from self to RedCar < merge_distance and 
                    relative heading of RedCar from self < 30 deg):
        # Attempt to merge into the right lane (which is to the left of ego on the ramp)
        if self.laneSection._laneToLeft is not None:
            do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToLeft, 
                                  is_oppositeTraffic=False, 
                                  target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)

behavior RedCarBehavior(target_speed):
    """Red car travels straight in the left lane at constant speed."""
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have a left neighbor (right lane of main road has left lane)
mainRoadRightLanes = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward and 
            laneSec._laneToLeft is not None and 
            laneSec._laneToLeft.isForward and
            len(laneSec.centerline.points) > 20):  # Ensure substantial lane section
            mainRoadRightLanes.append(laneSec)

require len(mainRoadRightLanes) > 0

# Select a right lane section on the main road
rightLaneSec = Uniform(*mainRoadRightLanes)
leftLaneSec = rightLaneSec._laneToLeft

# Place red car in the left lane ahead on the main road
redCarSpawnPt = new OrientedPoint on leftLaneSec.centerline
redCarPos = follow roadDirection from redCarSpawnPt for Range(50, 80)

# Place ego on the ramp/on-ramp leading to this merge point
# We look for a lane section that connects to or is near the right lane
# For simplicity, place ego behind where the red car will be after merge
egoSpawnOffset = Range(60, 90)
egoBasePt = new OrientedPoint on rightLaneSec.centerline
egoSpawnPt = follow roadDirection from egoBasePt for -egoSpawnOffset

# If there's a connecting ramp lane, prefer it; otherwise use right lane start
rampLanes = [ls for ls in network.laneSections 
             if ls._laneToRight is not None and ls._laneToRight == rightLaneSec]
if len(rampLanes) > 0:
    rampLane = Uniform(*rampLanes)
    egoSpawnPt = new OrientedPoint on rampLane.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    facing roadDirection,
    with blueprint EGO_MODEL,
    with color "grey",
    with behavior EgoMergeBehavior(globalParameters.OPT_EGO_SPEED, 
                                    globalParameters.OPT_MERGE_DISTANCE)

RedCar = new Car at redCarPos,
    facing roadDirection,
    with regionContainedIn leftLaneSec,
    with blueprint RED_CAR_MODEL,
    with color "red",
    with behavior RedCarBehavior(globalParameters.OPT_RED_CAR_SPEED)

# Ensure red car is ahead and to the front-left of ego initially
require distance from ego to RedCar > 30
require relative heading of RedCar from ego < 45 deg

# Terminate after successful merge and stable following
terminate when (distance from ego to RedCar < globalParameters.OPT_FOLLOWING_DISTANCE + 10 and
                ego.laneSection == rightLaneSec and
                simulation().currentTime > 15)