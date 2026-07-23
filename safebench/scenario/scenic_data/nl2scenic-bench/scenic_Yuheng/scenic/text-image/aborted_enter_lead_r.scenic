"""Scenario Description:

The image displays a top-down view of a multi-lane road marked by dashed white lines, where a blue ego vehicle is positioned in the center lane with a straight blue arrow indicating forward motion. Ahead and to the right of the ego vehicle, a pink car is shown with a wavy pink arrow trajectory that initially curves upward toward the center lane before veering back down toward the right lane.

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

EGO_COLOR = (0, 0, 1)       # Blue
NPC_COLOR = (1, 0.4, 0.7)   # Pink

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have both left and right neighbors (i.e., center lanes)
centerLaneSections = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward \
           and laneSec._laneToRight is not None and laneSec._laneToRight.isForward:
            centerLaneSections.append(laneSec)

require len(centerLaneSections) > 0

egoLaneSec = Uniform(*centerLaneSections)
rightLaneSec = egoLaneSec._laneToRight

#################################
# SCENARIO SPECIFICATION        #
#################################

# Ego vehicle in center lane facing forward
egoSpawn = new OrientedPoint on egoLaneSec.centerline
ego = new Car at egoSpawn,
    facing roadDirection,
    with blueprint 'vehicle.tesla.model3',
    with color EGO_COLOR

# NPC car ahead and to the right in the right lane
npcSpawnPt = follow roadDirection from egoSpawn for Range(20, 40)
npcSpawnPt = npcSpawnPt offset laterally by Range(-0.5, 0.5) relative to rightLaneSec.centerline

npc = new Car at npcSpawnPt,
    with regionContainedIn rightLaneSec,
    facing roadDirection,
    with blueprint 'vehicle.nissan.micra',
    with color NPC_COLOR

# Define wavy lane-change behavior: curve toward center lane then back to right lane
behavior WavyLaneChange(targetLane, sourceLane):
    # Phase 1: steer toward the target (center) lane
    do UTurn(maximumLateralSpeed=3, duration=Range(2, 4))
    # Phase 2: steer back toward the source (right) lane
    do UTurn(maximumLateralSpeed=3, duration=Range(2, 4))

npc.behavior = WavyLaneChange(egoLaneSec, rightLaneSec)