"""Scenario Description:

A blue ego vehicle is depicted on a multi-lane road marked by dashed white lines, initially positioned in the rightmost lane. A curved blue arrow illustrates the vehicle's trajectory as it executes a continuous maneuver to change lanes to the left. The path shows the car crossing the dashed white lane markings, moving from the bottom lane upwards across the adjacent lanes, effectively shifting from the right side of the road towards the left lanes.

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

EGO_MODEL = "vehicle.lincoln.mkz_2017"
EGO_COLOR = (0, 0, 255)  # Blue color for ego vehicle

param OPT_EGO_SPEED = Range(6, 9)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoLaneChangeBehavior(target_speed):
    """Ego vehicle performs a continuous lane change to the left."""
    try:
        do FollowLaneBehavior(target_speed=target_speed) until (distance along road from self to end of laneSection > 30)
        if self.laneSection._laneToLeft is not None and self.laneSection._laneToLeft.isForward:
            do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToLeft, target_speed=target_speed)
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when False:
        pass

#################################
# SPATIAL RELATIONS             #
#################################

# Find rightmost forward lane sections that have at least one lane to the left
rightmostLaneSecs = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if (laneSec.isForward 
            and laneSec._laneToLeft is not None 
            and laneSec._laneToLeft.isForward
            and laneSec._laneToRight is None):
            rightmostLaneSecs.append(laneSec)

require len(rightmostLaneSecs) > 0

egoLaneSec = Uniform(*rightmostLaneSecs)
egoSpawnPt = new OrientedPoint on egoLaneSec.centerline

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    facing roadDirection,
    with blueprint EGO_MODEL,
    with color EGO_COLOR,
    with behavior EgoLaneChangeBehavior(globalParameters.OPT_EGO_SPEED)

require distance to intersection >= 80