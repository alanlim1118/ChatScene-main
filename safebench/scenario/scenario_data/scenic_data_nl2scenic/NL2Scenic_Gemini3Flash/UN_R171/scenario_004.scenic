"""Scenario Description:

While traveling on a straight section, the ego vehicle wants to perform a lane change to overtake 
a lead vehicle but remains in its current lane because it detects a motorcycle approaching 
rapidly from behind in the adjacent lane, ensuring the target space is clear before moving.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"
MOTOR_MODEL = "vehicle.yamaha.yzf"

param EGO_SPEED = Range(9, 11)
param LEAD_SPEED = Range(5, 6)
param MOTOR_SPEED = Range(15, 18)

param INITIAL_DISTANCE_LEAD = Range(25, 30)
param INITIAL_DISTANCE_MOTOR = Range(15, 20) # Distance behind ego

param OVERTAKE_THRESHOLD = 15
param SAFETY_DISTANCE_MOTOR = 20

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(target_speed, lead_car, motor, target_lane_sec):
    """
    Ego behavior: drive behind lead, wait for motorcycle to pass, then change lane.
    """
    try:
        # 1. Drive at target speed until getting close to the slow lead car
        do FollowLaneBehavior(target_speed=target_speed) until (distance from self to lead_car < OVERTAKE_THRESHOLD)
        
        # 2. Detected need to overtake, but check for motorcycle in adjacent lane
        # While the motorcycle is within a safety bubble, stay in lane and match speed
        while (distance from self to motor < SAFETY_DISTANCE_MOTOR):
            # If the motorcycle is behind us or passing, stay in lane
            # We match the lead car's speed to maintain safety
            do FollowLaneBehavior(target_speed=lead_car.speed)
            
            # Additional safety: brake if getting way too close to lead car
            if withinDistanceToObjsInLane(self, 6):
                take SetBrakeAction(0.6)
        
        # 3. Motorcycle has passed and the gap is safe
        do LaneChangeBehavior(laneSectionToSwitch=target_lane_sec, target_speed=target_speed)
        
        # 4. Continue in the new lane
        do FollowLaneBehavior(target_speed=target_speed)

    interrupt when withinDistanceToObjsInLane(self, 5):
        take SetBrakeAction(1.0)

behavior MotorBehavior(speed):
    """
    Motorcycle approaching rapidly in the adjacent lane.
    """
    do FollowLaneBehavior(target_speed=speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find all lane sections that have a lane to their left
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

assert len(laneSecsWithLeftLane) > 0, "No suitable multi-lane road found."

# Select a random starting lane section for the ego
egoLaneSec = Uniform(*laneSecsWithLeftLane)
adjLaneSec = egoLaneSec._laneToLeft

# Define spawn points
egoSpawnPt = new OrientedPoint on egoLaneSec.centerline

# Lead car is ahead of ego in the same lane
leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.INITIAL_DISTANCE_LEAD

# Motorcycle is behind ego in the adjacent (left) lane
adjLanePt = adjLaneSec.centerline.project(egoSpawnPt.position)
motorSpawnPt = new OrientedPoint following roadDirection from adjLanePt for -globalParameters.INITIAL_DISTANCE_MOTOR

#################################
# SCENARIO SPECIFICATION        #
#################################

# Spawn the lead vehicle
LeadAgent = new Car at leadSpawnPt,
    with behavior FollowLaneBehavior(target_speed=globalParameters.LEAD_SPEED)

# Spawn the motorcycle (the obstacle in the target lane)
MotorcycleAgent = new Motorcycle at motorSpawnPt,
    with blueprint MOTOR_MODEL,
    with behavior MotorBehavior(speed=globalParameters.MOTOR_SPEED)

# Spawn the ego vehicle
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(
        target_speed=globalParameters.EGO_SPEED,
        lead_car=LeadAgent,
        motor=MotorcycleAgent,
        target_lane_sec=adjLaneSec
    )

#################################
# REQUIREMENTS                  #
#################################

# Ensure we are on a straight enough section by avoiding intersections
require (distance to intersection) > 40
require (distance from LeadAgent to intersection) > 40

# Terminate after some time or distance
terminate when (distance from ego to egoSpawnPt) > 150