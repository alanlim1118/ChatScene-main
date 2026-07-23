"""Scenario Description:

A top-down schematic view depicts a two-lane road separated by a dashed white center line, featuring a blue rectangular vehicle in the left lane moving towards the right and a pink rectangular vehicle in the right lane moving towards the left. Long arrows extending from each vehicle indicate their respective directions of travel, showing them moving in opposite lanes towards one another, which corresponds to the description of approaching an oncoming object.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town03'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town03'
model scenic.simulators.carla.model

#################################
# SCENARIO SPECIFICATION        #
#################################

# Collect all pairs of adjacent lane sections to locate two-lane road segments
adjacentPairs = []
for lane in network.lanes:
    for ls in lane.sections:
        if ls._laneToLeft is not None:
            adjacentPairs.append((ls, ls._laneToLeft))
        if ls._laneToRight is not None:
            adjacentPairs.append((ls, ls._laneToRight))

# Sample a pair of adjacent lane sections (one for ego, one for the oncoming vehicle)
egoLaneSec, oppLaneSec = resample(Uniform(*adjacentPairs))

# Place ego in the first lane section
ego = new Car on egoLaneSec.centerline,
    facing roadDirection

# Place the oncoming vehicle in the adjacent lane section
oncoming = new Car on oppLaneSec.centerline,
    facing roadDirection

# Require the vehicles are facing opposite directions (head-on)
require abs(relative heading of oncoming from ego) > 160 deg

# Require the oncoming vehicle is ahead of ego and approaching
# (apparent heading near 180 deg indicates a head-on approach)
require abs(apparent heading of oncoming from ego) > 150 deg

# Keep the encounter at a reasonable distance
require (distance from ego to oncoming) > 20
require (distance from ego to oncoming) < 100