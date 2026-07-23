"""Scenario Description:

On a sunny day in a suburban area, the ego vehicle travels forward on a straight road lined with a row of vibrant yellow trees on the left, maintaining a speed of approximately 59 km/h. A blue and yellow taxi suddenly emerges from the left side near the tree line and cuts directly across the ego vehicle's path into the lane. This abrupt intrusion forces the ego vehicle to decelerate sharply, dropping speed rapidly from nearly 60 km/h down to a complete stop at 0 km/h to avoid a collision, while the taxi proceeds to drive ahead in the same lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town01'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town01'
model scenic.domains.driving.model

#################################
# CONSTANTS                     #
#################################

EGO_SPEED_KMH = 59                    # Ego target speed in km/h
EGO_SPEED_MS = EGO_SPEED_KMH / 3.6    # Convert to m/s for CARLA
TAXI_SPEED_MS = 8.0                   # Taxi cruising speed after cutting in
BRAKE_TRIGGER_DISTANCE = 25           # Distance at which ego begins emergency braking
TAXI_EMERGE_DISTANCE = 30             # Distance ahead of ego where taxi enters lane
TAXI_LATERAL_OFFSET = 4               # Lateral offset from left curb/tree line
TREE_SPACING = 8                      # Spacing between decorative trees
NUM_TREES = 12                        # Number of trees along the left side

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoDriveAndEmergencyBrake():
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED_MS)
    interrupt when withinDistanceToObjsInLane(self, thresholdDistance=BRAKE_TRIGGER_DISTANCE):
        take SetThrottleAction(0), SetBrakeAction(1)
        while True:
            wait

behavior TaxiCutInAndProceed():
    # Wait until ego is close enough, then cut into the lane
    while distance from self to ego > TAXI_EMERGE_DISTANCE:
        wait
    # Steer into the ego's lane (right relative to taxi's initial position on left)
    take SetSteeringAction(0.5), SetThrottleAction(0.6)
    do WaitBehavior() for 1.5 seconds
    # Straighten out and continue driving in the lane
    take SetSteeringAction(0), SetThrottleAction(0.5)
    do FollowLaneBehavior(target_speed=TAXI_SPEED_MS)

behavior WaitBehavior():
    while True:
        wait

#################################
# SCENARIO SPECIFICATION        #
#################################

# Place ego on a straight section of road
straightSegments = [seg for seg in network.segments if len(seg.maneuvers) > 0 
                    and any(m.type is ManeuverType.STRAIGHT for m in seg.maneuvers)]
require len(straightSegments) > 0
chosenSegment = Uniform(*straightSegments)
egoSpawnPt = new OrientedPoint in chosenSegment.centerline

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with behavior EgoDriveAndEmergencyBrake()

# Place taxi initially off to the left of the road, ahead of ego
taxiSpawnBase = new OrientedPoint following chosenSegment.orientation from egoSpawnPt for TAXI_EMERGE_DISTANCE + 10
taxiSpawn = new OrientedPoint left of taxiSpawnBase by TAXI_LATERAL_OFFSET,
    facing heading of taxiSpawnBase

taxi = new Car at taxiSpawn,
    with blueprint "vehicle.ford.mustang",
    with color "Blue",
    with regionContainedIn None,
    with behavior TaxiCutInAndProceed()

# Decorative yellow trees along the left side of the road
leftCurb = chosenSegment.leftEdge
for i in range(NUM_TREES):
    treeOffset = (i * TREE_SPACING) + 5
    treeSpot = new OrientedPoint following chosenSegment.orientation from egoSpawnPt for treeOffset
    treePos = new OrientedPoint left of treeSpot by (TAXI_LATERAL_OFFSET + 2)
    new Object at treePos,
        with shape BoxShape(width=1, length=1, height=4),
        with color "Yellow",
        with regionContainedIn None

# Ensure scenario runs long enough to observe the full interaction
terminate after 30 seconds