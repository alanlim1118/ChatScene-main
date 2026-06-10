"""Scenario Description:
The ego vehicle travels on a non-physically separated road shared with pedestrians and cyclists. 
It approaches a slower-moving lead vehicle and intends to overtake. However, an oncoming vehicle 
creates an unsafe boundary condition, forcing the ego to delay its lane change. 
Once a safe gap is detected after the oncoming vehicle passes, the ego executes the 
maneuver, performing a lateral displacement to the opposite lane to bypass the leader 
before returning to its original lane.
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
LEAD_MODEL = "vehicle.toyota.prius"
ADV_MODEL = "vehicle.audi.a2"

param OPT_EGO_SPEED = Range(8, 10)
param OPT_LEAD_SPEED = Range(4, 5)
param OPT_ADV_SPEED = Range(8, 10)

# Safety constants
param OPT_SAFE_GAP = Range(50, 60)      # Minimum distance to oncoming car to start maneuver
param OPT_OVERTAKE_TRIGGER = 15         # Distance to lead car to start considering overtake
param OPT_LEAD_START_DIST = Range(20, 30)
param OPT_ADV_START_DIST = Range(100, 120)

#################################
# AGENT BEHAVIORS               #
#################################

behavior OncomingBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior LeadBehavior(target_speed):
    do FollowLaneBehavior(target_speed=target_speed)

behavior EgoOvertakeBehavior(target_speed, leader, oncoming, safety_gap):
    # 1. Drive behind the slower vehicle
    try:
        do FollowLaneBehavior(target_speed=target_speed) until (distance from self to leader < globalParameters.OPT_OVERTAKE_TRIGGER)
    interrupt when (distance from self to leader < 5):
        take SetBrakeAction(1)
        wait for 1 seconds

    # 2. Wait for the unsafe boundary condition to clear (oncoming vehicle passing)
    # The ego matches speed with the leader while waiting for the gap
    while (distance from self to oncoming < safety_gap) and (relative heading of oncoming.heading from self.heading > 90 deg):
        take SetSpeedAction(leader.speed)
    
    # 3. Gap detected: Execute the lateral displacement (lane change to opposite lane)
    # 3.5m lateral displacement is represented by a standard lane change in CARLA
    target_lane = self.laneSection._laneToLeft
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane, is_oppositeTraffic=True, target_speed=target_speed)
    
    # 4. Bypass the leader
    do FollowLaneBehavior(target_speed=target_speed) until (distance from self to leader > 15) and (self ahead of leader)
    
    # 5. Return to original lane
    do LaneChangeBehavior(laneSectionToSwitchTo=target_lane._laneToRight, is_oppositeTraffic=False, target_speed=target_speed)
    
    # 6. Continue
    do FollowLaneBehavior(target_speed=target_speed)

#################################
# SPATIAL RELATIONS             #
#################################

# Find road sections that allow for overtaking (two-way roads with no physical divider)
laneSecsWithOppositeLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        # We look for a lane that has a lane to its left which is oriented in the opposite direction
        if laneSec._laneToLeft is not None:
            if laneSec._laneToLeft.isForward != laneSec.isForward:
                laneSecsWithOppositeLane.append(laneSec)

# Select a suitable lane section
egoLaneSec = Uniform(*laneSecsWithOppositeLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Spawn points for the other agents
LeadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_START_DIST

# Oncoming agent starts further down the road in the adjacent (left) lane
adjLaneSec = egoLaneSec._laneToLeft
AdvSpawnPt_temp = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_ADV_START_DIST
AdvSpawnPt = adjLaneSec.centerline.project(AdvSpawnPt_temp.position)

# Shared road elements: Cyclist and Pedestrian
CyclistSpawnPt = new OrientedPoint following roadDirection from LeadSpawnPt for 20
PedestrianSpawnPt = new OrientedPoint right of egoSpawnPt by 4

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle setup
ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint EGO_MODEL,
    with behavior EgoOvertakeBehavior(
        globalParameters.OPT_EGO_SPEED, 
        None, # Assigned after LeadingAgent is created
        None, # Assigned after OncomingAgent is created
        globalParameters.OPT_SAFE_GAP
    )

# Slower lead vehicle
LeadingAgent = new Car at LeadSpawnPt,
    with blueprint LEAD_MODEL,
    with behavior LeadBehavior(globalParameters.OPT_LEAD_SPEED)

# Oncoming vehicle creating the boundary condition
OncomingAgent = new Car at AdvSpawnPt,
    with blueprint ADV_MODEL,
    with heading egoSpawnPt.heading + 180 deg,
    with behavior OncomingBehavior(globalParameters.OPT_ADV_SPEED)

# Assign references to Ego
ego.behavior.leader = LeadingAgent
ego.behavior.oncoming = OncomingAgent

# Shared road context: A cyclist in the same lane further ahead
cyclist = new Bicycle at CyclistSpawnPt,
    with behavior FollowLaneBehavior(target_speed=3)

# Shared road context: A pedestrian on the side of the road
ped = new Pedestrian at PedestrianSpawnPt,
    with behavior WalkForwardBehavior(speed=0.5)

#################################
# REQUIREMENTS                  #
#################################

# Ensure we have a long enough straight road for the maneuver
require distance to intersection > 80
require (relative heading of OncomingAgent.heading from ego.heading) > 170 deg