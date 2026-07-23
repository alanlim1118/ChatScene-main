"""Scenario Description:

The ego vehicle follows a lead vehicle in a straight lane for over two seconds with a lateral offset of less than one meter, then ego vehicle successfully completes a full lane change maneuver involving a 3.5-meter lateral displacement into the adjacent lane.

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

EGO_SPEED = 10
LEAD_SPEED = 8
FOLLOW_DIST = 15
FOLLOW_DURATION = 3
LATERAL_OFFSET_THRESHOLD = 1.0
LANE_CHANGE_DISPLACEMENT = 3.5

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(targetLaneSec):
    # Phase 1: Follow lead vehicle maintaining small lateral offset
    do FollowLaneBehavior(target_speed=EGO_SPEED) for FOLLOW_DURATION seconds
    
    # Phase 2: Execute lane change into adjacent lane
    do LaneChangeBehavior(
        laneSectionToSwitch=targetLaneSec,
        target_speed=EGO_SPEED
    )
    
    # Continue in new lane to ensure completion
    do FollowLaneBehavior(target_speed=EGO_SPEED) for 2 seconds
    terminate

behavior LeadVehicleBehavior():
    do FollowLaneBehavior(target_speed=LEAD_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have an adjacent right lane for lane change
laneSecsWithRightLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToRight is not None:
            laneSecsWithRightLane.append(laneSec)

assert len(laneSecsWithRightLane) > 0, \
    'No lane sections with adjacent right lane found in network.'

initLaneSec = Uniform(*laneSecsWithRightLane)
rightLaneSec = initLaneSec._laneToRight

spawnPt = new OrientedPoint on initLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at spawnPt,
    with behavior EgoBehavior(rightLaneSec)

lead = new Car following roadDirection from ego for FOLLOW_DIST,
    with behavior LeadVehicleBehavior()

# Ensure sufficient distance from intersections for clean maneuver
require (distance from ego to intersection) > 30
require (distance from lead to intersection) > 30

# Require lateral offset constraint during following phase
require always (
    (simulation().currentTime < FOLLOW_DURATION) implies
    (abs(relative position of ego with respect to lead).y < LATERAL_OFFSET_THRESHOLD)
)

# Require that the lane change achieves approximately 3.5m lateral displacement
# by checking ego ends up in the target lane section
require eventually (ego.laneSection is rightLaneSec)