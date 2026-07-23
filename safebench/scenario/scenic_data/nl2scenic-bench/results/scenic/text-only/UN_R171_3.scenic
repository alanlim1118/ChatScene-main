"""Scenario Description:

The ego vehicle initiates a driver-requested lane change on a multi-lane road, but must detect and yield to a high-speed vehicle approaching from the rear in the target lane, delaying the maneuver until the overtaking vehicle has safely passed.

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

MODEL = 'vehicle.lincoln.mkz_2017'

param EGO_SPEED = Range(8, 12)
param ADV_SPEED = Range(18, 25)  # High-speed approaching vehicle
param ADV_INIT_DIST = Range(60, 100)  # Initial distance behind ego in target lane
param SAFE_GAP = Range(25, 35)  # Minimum safe gap before initiating lane change
param TERM_TIME = 10  # Time to continue after successful lane change

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior():
    """Ego attempts lane change but waits until the approaching vehicle has passed."""
    targetLaneSec = self.laneSection._laneToLeft
    if targetLaneSec is None:
        targetLaneSec = self.laneSection._laneToRight
    
    # Wait in current lane until the approaching vehicle is far enough behind
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) \
        until (distance from self to adversary) > globalParameters.SAFE_GAP or \
              (relative heading of adversary from self) > 90 deg
    
    # Once safe, execute lane change
    do LaneChangeBehavior(
            laneSectionToSwitch=targetLaneSec,
            target_speed=globalParameters.EGO_SPEED)
    
    # Continue driving in new lane
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) for globalParameters.TERM_TIME seconds
    terminate

behavior ApproachingVehicleBehavior():
    """High-speed vehicle that maintains its speed in the target lane."""
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

#################################
# SPATIAL RELATIONS             #
#################################

# Find lane sections that have an adjacent lane for lane changing
validLaneSections = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and (laneSec._laneToLeft is not None or laneSec._laneToRight is not None):
            validLaneSections.append(laneSec)

egoLaneSec = Uniform(*validLaneSections)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Place adversary behind ego in the target lane (approaching from rear)
targetLaneSec = egoLaneSec._laneToLeft
if targetLaneSec is None:
    targetLaneSec = egoLaneSec._laneToRight

# Project ego position onto target lane centerline to get reference point
targetRefPt = targetLaneSec.centerline.project(egoSpawnPt.position)

# Adversary starts behind ego in the target lane (negative direction = behind)
advSpawnPt = new OrientedPoint following roadDirection from targetRefPt for -globalParameters.ADV_INIT_DIST

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with blueprint MODEL,
    with behavior EgoBehavior()

adversary = new Car at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn targetLaneSec,
    with blueprint MODEL,
    with behavior ApproachingVehicleBehavior()

# Ensure sufficient distance from intersections for clean scenario execution
require (distance to intersection) > 80
require (distance from adversary to intersection) > 80

# Ensure the target lane exists throughout the scenario
require always (targetLaneSec is not None)