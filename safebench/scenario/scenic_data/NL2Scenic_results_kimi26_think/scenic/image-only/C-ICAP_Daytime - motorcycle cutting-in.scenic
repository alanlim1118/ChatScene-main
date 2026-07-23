"""Scenario Description:

The scenario takes place on a long, straight road with two lanes separated by a dotted line, where a green vehicle travels in the upper lane and a motorcycle travels in the lower lane. Both vehicles are moving in the same direction, with the motorcycle positioned ahead of the green vehicle in the adjacent lane. As the green vehicle approaches the motorcycle, the motorcycle initiates a lane change maneuver, curving from the lower lane into the upper lane, thereby cutting into the lane occupied by the green vehicle.

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

param OPT_EGO_SPEED = Range(10, 15)
param OPT_MOTORCYCLE_SPEED = Range(6, 9)
param OPT_TRIGGER_DIST = Range(15, 25)
param OPT_CUTIN_LENGTH = Range(25, 40)
param OPT_LANE_WIDTH = 3.5

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior CutInBehavior(speed, trigger_dist, ego_vehicle):
    # Follow the lower lane until the green vehicle is close behind
    do FollowLaneBehavior(target_speed=speed) until (distance from ego_vehicle to self) < trigger_dist
    
    # Define a curved polyline trajectory that shifts into the upper lane
    p1 = new OrientedPoint at self.position
    p2 = new OrientedPoint at p1 offset by (15 @ OPT_LANE_WIDTH)
    p3 = new OrientedPoint at p1 offset by (30 @ OPT_LANE_WIDTH)
    
    cutInTrajectory = PolylineRegion([p1.position, p2.position, p3.position])
    do FollowTrajectoryBehavior(speed, cutInTrajectory)

#################################
# SPATIAL RELATIONS             #
#################################

# Select the upper lane and its adjacent lower lane (to the right)
egoLane = Uniform(*filter(lambda l: l._laneToRight is not None, network.lanes))
advLane = egoLane._laneToRight

# Spawn the green vehicle in the upper lane
egoSpawnPt = new OrientedPoint in egoLane.centerline

# Spawn the motorcycle ahead in the lower lane
aheadPt = new OrientedPoint ahead of egoSpawnPt by globalParameters.OPT_CUTIN_LENGTH
motorcycleSpawnPt = new OrientedPoint at advLane.centerline.project(aheadPt.position)
motorcycleHeading = advLane.orientation[motorcycleSpawnPt.position]

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn None,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED)

motorcycle = new Motorcycle at motorcycleSpawnPt,
    with heading motorcycleHeading,
    with regionContainedIn None,
    with behavior CutInBehavior(globalParameters.OPT_MOTORCYCLE_SPEED, globalParameters.OPT_TRIGGER_DIST, ego)

# Ensure there is enough road ahead for the maneuver
require (distance from egoSpawnPt to egoLane.centerline.end) > 100
require (distance from motorcycleSpawnPt to advLane.centerline.end) > 80