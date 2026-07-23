"""Scenario Description:

A green vehicle travels forward in the left lane of a straight road divided by a central dotted line, approaching a construction zone situated directly ahead. The obstruction begins with a series of red water-filled barriers arranged in a diagonal formation, tilted at a 30-degree angle in the direction of the vehicle's travel, which guide traffic away from the blocked area. Following these barriers is a long, rectangular iron sheet wall that spans across the lane and occupies a portion of the adjacent lane, creating a continuous barrier that necessitates a maneuver around the work zone.

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
BARRIER_MODEL = "static.prop.waterfilled_barrier"
WALL_MODEL = "static.prop.iron_sheet_wall"

param OPT_EGO_SPEED = Range(3, 6)
param OPT_BRAKE_DISTANCE = Range(8, 15)
param OPT_CONSTRUCTION_DISTANCE = Range(40, 60)
param OPT_NUM_BARRIERS = Range(3, 5)
param OPT_BARRIER_SPACING = Range(2.5, 4.0)
param OPT_WALL_LENGTH = Range(6, 10)

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

behavior ConstructionApproachBehavior(speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=brake_dist):
        take SetThrottleAction(0)
        take SetBrakeAction(1)
        do WaitBehavior() for 10 seconds
        terminate

#################################
# SPATIAL RELATIONS             #
#################################

# Select a straight road segment with multiple lanes
straightRoad = Uniform(*filter(lambda r: len(r.lanes) >= 2 and r.isStraight, network.roads))
leftLane = Uniform(*filter(lambda l: l.isForward, straightRoad.lanes))

egoSpawnPt = new OrientedPoint in leftLane.centerline

# Construction zone reference point ahead of ego
constructionStartPt = new OrientedPoint following leftLane.orientation from egoSpawnPt for globalParameters.OPT_CONSTRUCTION_DISTANCE

#################################
# SCENARIO SPECIFICATION        #
#################################

# Green ego vehicle in the left lane
ego = new Car at egoSpawnPt,
    with blueprint EGO_MODEL,
    with color Color.withBytes([0, 180, 0]),
    with regionContainedIn None,
    with behavior ConstructionApproachBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_BRAKE_DISTANCE)

# Red water-filled barriers in diagonal formation (30 degrees relative to travel direction)
barrierBasePt = constructionStartPt
for i in range(globalParameters.OPT_NUM_BARRIERS):
    offsetDist = i * globalParameters.OPT_BARRIER_SPACING
    barrierPt = new OrientedPoint following constructionStartPt.heading from barrierBasePt for offsetDist,
        with heading constructionStartPt.heading + 30 deg
    new Object at barrierPt,
        with blueprint BARRIER_MODEL,
        with color Color.withBytes([200, 0, 0]),
        with regionContainedIn None

# Iron sheet wall after the barriers, spanning across lanes
wallOffset = globalParameters.OPT_NUM_BARRIERS * globalParameters.OPT_BARRIER_SPACING + 2
wallCenterPt = new OrientedPoint following constructionStartPt.heading from constructionStartPt for wallOffset,
    with heading constructionStartPt.heading + 90 deg  # Perpendicular to road to span across lanes

new Object at wallCenterPt,
    with blueprint WALL_MODEL,
    with regionContainedIn None,
    with width globalParameters.OPT_WALL_LENGTH

require distance from ego to constructionStartPt >= 30
terminate when distance from ego to constructionStartPt > 80 or simulation().currentTime > 30