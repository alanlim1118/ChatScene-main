"""Scenario Description:

On a sunny day in a suburban area, the ego vehicle travels forward on a straight road lined with a row of vibrant yellow trees on the left, maintaining a speed of approximately 59 km/h. A blue and yellow taxi suddenly emerges from the left side near the tree line and cuts directly across the ego vehicle's path into the lane. This abrupt intrusion forces the ego vehicle to decelerate sharply, dropping speed rapidly from nearly 60 km/h down to a complete stop at 0 km/h to avoid a collision, while the taxi proceeds to drive ahead in the same lane.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
param weather = "ClearNoon"
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_SPEED = 16.4                    # 59 km/h in m/s
BRAKE_DISTANCE = 12                 # Distance at which ego begins hard braking
TAXI_TRIGGER_DISTANCE = 30          # Distance at which taxi pulls out
TAXI_SPEED = 12                     # Target speed for taxi after merging (approx 43 km/h)
EGO_TO_TAXI_MIN_DIST = 50           # Ensure ego starts far enough away

#################################
# AGENT BEHAVIORS               #
#################################

behavior WaitBehavior():
    while True:
        wait

# Ego follows lane and brakes hard when an object enters its lane
behavior EgoDriveAndBrakeBehavior():
    try:
        do FollowLaneBehavior(target_speed=EGO_SPEED)
    interrupt when withinDistanceToObjsInLane(self, BRAKE_DISTANCE):
        take SetThrottleAction(0), SetBrakeAction(1)
        do WaitBehavior() for 5 seconds
        terminate

# Taxi waits on the left shoulder and pulls out when ego approaches
behavior TaxiCutInBehavior(ego_vehicle, trigger_dist, target_speed):
    while distance from self to ego_vehicle > trigger_dist:
        wait
    take SetThrottleAction(1.0)
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

intersection = Uniform(*filter(lambda i: i.is4Way and not i.isSignalized, network.intersections))
egoInitLane = Uniform(*intersection.incomingLanes)
egoManeuver = Uniform(*filter(lambda m: m.type is ManeuverType.STRAIGHT, egoInitLane.maneuvers))
egoSpawnPt = new OrientedPoint in egoManeuver.startLane.centerline

# Place taxi on the left side near the tree line (shoulder)
taxiSpawnPt = new OrientedPoint left of egoSpawnPt by 4,
    facing egoSpawnPt.heading

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle traveling on a straight suburban road
ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with behavior EgoDriveAndBrakeBehavior()

# Blue and yellow taxi emerges from the left side near the tree line
taxi = new Car at taxiSpawnPt,
    with color [0.1, 0.1, 0.9],  # Blue taxi (yellow represented in description)
    with regionContainedIn None,
    with behavior TaxiCutInBehavior(ego, TAXI_TRIGGER_DISTANCE, TAXI_SPEED)

require distance from ego to taxi > EGO_TO_TAXI_MIN_DIST

terminate after 30 seconds