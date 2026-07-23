"""Scenario Description:

A top-down schematic view depicts a two-lane road separated by a dashed white center line, featuring a blue rectangular vehicle in the left lane moving towards the right and a pink rectangular vehicle in the right lane moving towards the left. Long arrows extending from each vehicle indicate their respective directions of travel, showing them moving in opposite lanes towards one another, which corresponds to the description of approaching an oncoming object.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model

#################################
# FUNCTIONS                     #
#################################

def findTwoLaneRoadSection():
    """Find a lane section that has exactly one adjacent lane to the left (opposing traffic)."""
    candidates = []
    for lane in network.lanes:
        if not lane.isForward:
            continue
        for laneSec in lane.sections:
            leftLane = laneSec._laneToLeft
            if leftLane is not None and not leftLane.isForward:
                candidates.append((laneSec, leftLane))
    return candidates

#################################
# SPATIAL RELATIONS             #
#################################

twoLaneSections = findTwoLaneRoadSection()
require len(twoLaneSections) > 0

selectedPair = resample(Uniform(*twoLaneSections))
egoLaneSec = selectedPair[0]
oncomingLaneSec = selectedPair[1]

# Define a reference point along the ego lane centerline
refPoint = new OrientedPoint on egoLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

# Blue vehicle in the left (ego) lane moving forward (towards the right in schematic)
ego = new Car at refPoint,
    with regionContainedIn egoLaneSec,
    facing roadDirection,
    with blueprint 'vehicle.tesla.model3',
    with color (0, 0, 255)

# Pink vehicle in the right (oncoming) lane moving towards the ego (opposite direction)
# Place it ahead of the ego along the road, but in the opposing lane facing backwards
oncomingPos = follow roadDirection from refPoint for Range(30, 60)
oncomingSpawn = new OrientedPoint at oncomingPos,
    facing roadDirection

oncomingCar = new Car at oncomingSpawn,
    with regionContainedIn oncomingLaneSec,
    facing -roadDirection,
    with blueprint 'vehicle.tesla.model3',
    with color (255, 192, 203)

# Ensure both vehicles are properly placed in their respective lanes
require egoLaneSec is not None
require oncomingLaneSec is not None