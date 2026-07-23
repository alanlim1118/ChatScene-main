"""Scenario Description:

In a dark, nighttime environment, the ego vehicle navigates towards a multi-lane roundabout, following a lead vehicle with visible headlights. As the ego vehicle enters the junction to execute a right turn, the lead vehicle ahead abruptly halts within the roundabout. This sudden stop occurs because the lead vehicle must yield to three oncoming adversary vehicles entering the intersection from the upper-right lanes, their headlights becoming visible as they approach. Reacting to the lead vehicle's braking, the ego vehicle also comes to a complete stop directly behind it to prevent a collision, waiting for the traffic flow to clear.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

MODEL = 'vehicle.lincoln.mkz_2017'

param EGO_SPEED = VerifaiRange(5, 8)
param LEAD_SPEED = VerifaiRange(5, 8)
param ADV_SPEED = VerifaiRange(6, 9)

param EGO_BRAKE = VerifaiRange(0.7, 1.0)
param LEAD_BRAKE = VerifaiRange(0.8, 1.0)

EGO_FOLLOW_DIST = Range(12, 18)
LEAD_STOP_ZONE = Range(5, 15)
ADV_INIT_DIST = Range(20, 40)
CRASH_DIST = 4
TERM_TIME = 60

#################################
# AGENT BEHAVIORS               #
#################################

behavior LeadBehavior(trajectory, stopPoint):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.LEAD_SPEED, trajectory=trajectory)
    interrupt when (distance from self to stopPoint) <= globalParameters.LEAD_STOP_ZONE:
        take SetBrakeAction(globalParameters.LEAD_BRAKE)
        take SetThrottleAction(0)
        while True:
            wait

behavior EgoBehavior(trajectory, leadVehicle):
    try:
        do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)
    interrupt when (distance from self to leadVehicle) <= globalParameters.EGO_FOLLOW_DIST:
        take SetBrakeAction(globalParameters.EGO_BRAKE)
        take SetThrottleAction(0)
        while True:
            wait

behavior AdversaryBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.ADV_SPEED, trajectory=trajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select a roundabout intersection in the network
roundabout = Uniform(*filter(lambda i: i.isRoundabout, network.intersections))

# Ego and lead vehicle share the same incoming lane and right-turn maneuver
egoInitLane = Uniform(*roundabout.incomingLanes)
rightManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.RIGHT_TURN, egoInitLane.maneuvers))
sharedTrajectory = [egoInitLane, rightManeuver.connectingLane, rightManeuver.endLane]

# Spawn lead vehicle ahead on the approach lane
leadSpawnPt = new OrientedPoint in egoInitLane.centerline
# Spawn ego behind the lead vehicle
egoSpawnPt = new OrientedPoint behind leadSpawnPt by EGO_FOLLOW_DIST

# Define stop point inside the roundabout where lead vehicle halts
stopPoint = new OrientedPoint in rightManeuver.connectingLane.centerline

# Three adversary vehicles entering from upper-right lanes of the roundabout
# These are lanes that conflict with the right-turn maneuver
conflictingManeuvers = filter(lambda m: m.type is ManeuverType.STRAIGHT or m.type is ManeuverType.LEFT_TURN,
                              rightManeuver.conflictingManeuvers)
advManeuverList = list(conflictingManeuvers)

# If not enough conflicting maneuvers, reuse with offset spawning
advManeuvers = []
for i in range(3):
    if i < len(advManeuverList):
        advManeuvers.append(advManeuverList[i])
    else:
        advManeuvers.append(advManeuverList[i % len(advManeuverList)])

advTrajectories = []
advSpawnPts = []
for am in advManeuvers:
    traj = [am.startLane, am.connectingLane, am.endLane]
    advTrajectories.append(traj)
    pt = new OrientedPoint in am.startLane.centerline
    advSpawnPts.append(pt)

#################################
# SCENARIO SPECIFICATION        #
#################################

# Nighttime / dark environment
param timeOfDay = 0.0
param weather = Weather(precipitation=0, cloudiness=0.9, wetness=0.3, fog=0.1, sunAltitude=-10)

leadVehicle = new Car at leadSpawnPt,
    with blueprint MODEL,
    with behavior LeadBehavior(sharedTrajectory, stopPoint),
    with headlights True

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior(sharedTrajectory, leadVehicle),
    with headlights True

adversary1 = new Car at advSpawnPts[0],
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectories[0]),
    with headlights True

adversary2 = new Car at advSpawnPts[1],
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectories[1]),
    with headlights True

adversary3 = new Car at advSpawnPts[2],
    with blueprint MODEL,
    with behavior AdversaryBehavior(advTrajectories[2]),
    with headlights True

# Ensure adversaries are at appropriate distances when scenario starts
require all((ADV_INIT_DIST[0] <= (distance from advSpawnPts[i] to stopPoint) <= ADV_INIT_DIST[1]) for i in range(3))

# Termination conditions
terminate when simulation().currentTime > TERM_TIME
terminate when (distance from ego to leadVehicle) < CRASH_DIST