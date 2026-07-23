"""Scenario Description:

In this traffic scenario, a blue ego car travels straight along the lower lane of a two-lane road, moving towards the right as indicated by a blue directional arrow. It approaches a pink adversarial object, depicted as a circle located below the road, which is laterally moving upwards and crossing into the ego-traffic area from the right side of the vehicle's path, indicated by a pink arrow pointing perpendicular to the flow of traffic.

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

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(1, 3)
param OPT_ADV_DISTANCE = Range(15, 25)
param OPT_ADV_OFFSET = Range(3, 6)

#################################
# AGENT BEHAVIORS               #
#################################

behavior EgoBehavior(speed):
    do FollowLaneBehavior(target_speed=speed)

behavior CrossBehavior(speed):
    take SetWalkingDirectionAction(self.heading)
    take SetWalkingSpeedAction(speed)
    while True:
        wait

#################################
# SPATIAL RELATIONS             #
#################################

# Identify forward lane sections that have a lane to the left (ego is in the lower/right lane)
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

# Point ahead on the lane centerline where the adversary will cross
advRefPt = new OrientedPoint following egoLaneSec.centerline from egoSpawnPt for globalParameters.OPT_ADV_DISTANCE

# Adversarial object spawn point: off-road to the right (below the road), heading perpendicular across traffic
advSpawnPt = new OrientedPoint right of advRefPt by globalParameters.OPT_ADV_OFFSET,
    with heading egoSpawnPt.heading + 90 deg

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED)

AdvAgent = new Pedestrian at advSpawnPt,
    with heading advSpawnPt.heading,
    with regionContainedIn None,
    with behavior CrossBehavior(globalParameters.OPT_ADV_SPEED)

require (distance to intersection) >= 50
terminate when (distance from ego to AdvAgent) > 60