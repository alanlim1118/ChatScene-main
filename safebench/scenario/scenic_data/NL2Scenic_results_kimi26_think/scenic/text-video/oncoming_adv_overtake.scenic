"""Scenario Description:

The ego vehicle drives forward on a wet, two-lane mountain road surrounded by dense vegetation under overcast conditions, initially traveling at roughly 82 km/h. As the vehicle rounds a gentle left curve marked by roadside chevron signs, an oncoming red truck appears in the opposite lane, followed closely by a black SUV. The black SUV attempts to overtake the truck by crossing the center line, directly invading the ego vehicle's lane and creating an imminent head-on collision threat. In response, the ego vehicle executes a sudden emergency brake, rapidly reducing speed as the black SUV passes extremely close to the front left side of the car. The ego vehicle continues to decelerate sharply, coming to a complete stop on the right side of the road immediately after the dangerous encounter.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town07'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr') 
param carla_map = 'Town07'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

# Speeds in m/s
EGO_SPEED = 22.78          # ~82 km/h
TRUCK_SPEED = 16.67        # ~60 km/h
SUV_SPEED = 20.0           # ~72 km/h

# Scenario parameters
param OPT_ONCOMING_DIST = Range(80, 120)      # Distance ahead for oncoming traffic
param OPT_SUV_SPAWN_DIST = Range(20, 30)      # Distance behind truck for SUV
param OPT_BRAKE_DIST = Range(25, 35)          # Distance at which ego emergency brakes
param OPT_OVERTAKE_DIST = Range(10, 15)       # Distance to truck for SUV to start overtaking

OPT_EGO_BRAKE = 1.0
OPT_EGO_STEER = -0.35      # Steer right toward the road edge

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(ego_speed, brake_dist):
    try:
        do FollowLaneBehavior(target_speed=ego_speed)
    interrupt when (distance from self to suv < brake_dist):
        # Emergency brake and pull toward the right side of the road
        while self.speed > 0.5:
            take SetBrakeAction(OPT_EGO_BRAKE)
            take SetSteerAction(OPT_EGO_STEER)

behavior TruckBehavior(truck_speed):
    do FollowLaneBehavior(target_speed=truck_speed)

behavior InvadeBehavior(steer_dir):
    while True:
        take SetSteerAction(steer_dir)
        take SetThrottleAction(0.6)

behavior SUVBehavior(truck, suv_speed, overtake_dist, steer_dir):
    do FollowLaneBehavior(target_speed=suv_speed) until (distance from self to truck < overtake_dist)
    # Cross the center line into the ego lane
    do InvadeBehavior(steer_dir) for 2 seconds
    while True:
        take SetThrottleAction(0.6)

#################################
# SPATIAL RELATIONS             #
#################################

# Find a forward lane section with an adjacent opposite-direction lane
validPairs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward:
            if laneSec._laneToRight is not None and not laneSec._laneToRight.isForward:
                validPairs.append((laneSec, laneSec._laneToRight))
            elif laneSec._laneToLeft is not None and not laneSec._laneToLeft.isForward:
                validPairs.append((laneSec, laneSec._laneToLeft))

egoLaneSec, oncomingLaneSec = Uniform(*validPairs)

# Determine steering direction for the SUV to invade the ego lane
if egoLaneSec._laneToRight is oncomingLaneSec:
    # Oncoming lane is to the right; ego lane is to the left of it
    suvInvadeSteer = 0.4   # steer left (toward ego lane)
else:
    # Oncoming lane is to the left; ego lane is to the right of it
    suvInvadeSteer = -0.4  # steer right (toward ego lane)

egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Oncoming traffic placed ahead on the road, projected onto the opposite lane
aheadPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ONCOMING_DIST
truckSpawnPt = new OrientedPoint at oncomingLaneSec.centerline.project(aheadPt.position)

# Black SUV spawns behind the truck in the oncoming lane
suvSpawnPt = new OrientedPoint following roadDirection from truckSpawnPt for globalParameters.OPT_SUV_SPAWN_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle
ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(EGO_SPEED, globalParameters.OPT_BRAKE_DIST)

# Red truck in the opposite lane
truck = new Car at truckSpawnPt,
    with regionContainedIn oncomingLaneSec,
    with blueprint "vehicle.carlamotors.firetruck",
    with color (0.8, 0.1, 0.1),
    with behavior TruckBehavior(TRUCK_SPEED)

# Black SUV in the opposite lane, behind the truck
suv = new Car at suvSpawnPt,
    with regionContainedIn oncomingLaneSec,
    with blueprint "vehicle.jeep.wrangler_rubicon",
    with color (0.05, 0.05, 0.05),
    with behavior SUVBehavior(truck, SUV_SPEED, globalParameters.OPT_OVERTAKE_DIST, suvInvadeSteer)

# Wet and overcast weather parameters
param weather = {
    'cloudiness': 90,
    'precipitation': 15,
    'precipitation_deposits': 50,
    'wetness': 100,
    'sun_altitude_angle': 10,
    'fog_density': 25
}

require (distance to intersection) >= 100
terminate when ego.speed < 0.5 and (distance from ego to suv) < 30