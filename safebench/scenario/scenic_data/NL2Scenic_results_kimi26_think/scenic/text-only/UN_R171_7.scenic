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
LEAD_SPEED = 10
LEAD_DIST = 20
FOLLOW_TIME = 2.5

#################################
# AGENT BEHAVIORS               #
#################################

# EGO BEHAVIOR: Follow lead in current lane, then change to adjacent lane
behavior EgoBehavior(adjacentLaneSec):
    # Follow in the current lane for over 2 seconds
    do FollowLaneBehavior(target_speed=EGO_SPEED) for FOLLOW_TIME seconds
    
    # Perform full lane change into adjacent lane (~3.5 m lateral displacement)
    do LaneChangeBehavior(laneSectionToSwitch=adjacentLaneSec, target_speed=EGO_SPEED)
    
    # Continue driving in the new lane
    do FollowLaneBehavior(target_speed=EGO_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

laneSecsWithAdjacent = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft != None:
            laneSecsWithAdjacent.append(laneSec)

assert len(laneSecsWithAdjacent) > 0, \
    'No lane sections with adjacent lane in network.'

initLaneSec = Uniform(*laneSecsWithAdjacent)
adjacentLane = initLaneSec._laneToLeft

spawnPt = new OrientedPoint on initLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at spawnPt,
    with behavior EgoBehavior(adjacentLane)

lead = new Car following roadDirection from ego for LEAD_DIST,
    with behavior FollowLaneBehavior(target_speed=LEAD_SPEED)

require (distance from ego to intersection) > 20
require (distance from lead to intersection) > 20