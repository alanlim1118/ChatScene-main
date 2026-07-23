description = "The ego is performing a right turn at an intersection when the crossing car suddenly speeds up, causing the ego to brake abruptly."
param map = localPath('../../maps/Town07.xodr')
param carla_map = 'Town07'
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"
# -----------------------------------------------------------
# 1. PYTHON PHASE: Use geometry to find crossing maneuvers
# -----------------------------------------------------------
validPairs = []
for i in network.intersections:
    if i.is4Way:
        for ego_m in i.maneuvers:
            if ego_m.type is ManeuverType.RIGHT_TURN:
                crossing_maneuvers = []
                for other_m in i.maneuvers:
                    if other_m.type is ManeuverType.STRAIGHT:
                        # Find a straight path that physically crosses the right turn!
                        if ego_m.connectingLane.intersects(other_m.connectingLane):
                            crossing_maneuvers.append(other_m)
                
                if len(crossing_maneuvers) > 0:
                    validPairs.append((ego_m, crossing_maneuvers))

chosenPair = Uniform(*validPairs)
egoManeuver = chosenPair[0]
advManeuver = Uniform(*chosenPair[1])

intersection = egoManeuver.intersection

# -----------------------------------------------------------
# 2. SPATIAL SETUP PHASE (Math handles the distances perfectly)
# -----------------------------------------------------------
egoInitLane = egoManeuver.startLane
egoTrajectory = [egoInitLane, egoManeuver.connectingLane, egoManeuver.endLane]

# Safely spawn Ego 15-20 meters behind the intersection
egoStopLinePt = new OrientedPoint at egoInitLane.centerline[-1], facing roadDirection
egoSpawnPt = new OrientedPoint behind egoStopLinePt by Range(15, 20)

advTrajectory = [advManeuver.startLane, advManeuver.connectingLane, advManeuver.endLane]

# Safely spawn Adv 20-30 meters behind the intersection
advStopLinePt = new OrientedPoint at advManeuver.startLane.centerline[-1], facing roadDirection
param OPT_GEO_Y_DISTANCE = Range(20, 30)
advSpawnPt = new OrientedPoint behind advStopLinePt by globalParameters.OPT_GEO_Y_DISTANCE

# -----------------------------------------------------------
# 3. BEHAVIOR PHASE
# -----------------------------------------------------------
param OPT_EGO_SPEED = Range(5, 7)
param OPT_EGO_YIELD_DIST = Range(8, 12)

behavior EgoBehavior():
    try:
        do FollowTrajectoryBehavior(trajectory=egoTrajectory, target_speed=globalParameters.OPT_EGO_SPEED)
    interrupt when (distance to AdvAgent) < globalParameters.OPT_EGO_YIELD_DIST:
        while True:
            take SetBrakeAction(1.0)

param OPT_ADV_SPEED = globalParameters.OPT_EGO_SPEED * Uniform(0.8, 0.9)
param OPT_ADV_DISTANCE = Range(20, 30)
param OPT_ADV_ACC_SPEED = Range(25, 30) 

behavior AdvBehavior():
    # Approach slowly...
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=advTrajectory) \
        until (distance to ego) < globalParameters.OPT_ADV_DISTANCE
        
    # FLOOR IT!
    do FollowTrajectoryBehavior(target_speed=globalParameters.OPT_ADV_ACC_SPEED, trajectory=advTrajectory)

# -----------------------------------------------------------
# 4. INSTANTIATION PHASE
# -----------------------------------------------------------
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior()

AdvAgent = new Car at advSpawnPt,
    with blueprint EGO_MODEL,
    with behavior AdvBehavior()

# -----------------------------------------------------------
# 5. CONSTRAINT PHASE (Impossible math deleted)
# -----------------------------------------------------------
terminate when (distance from ego to egoSpawnPt) > 60
terminate after 30 seconds
MODEL = 'vehicle.lincoln.mkz_2017'

param weather = 'CloudyNoon'
