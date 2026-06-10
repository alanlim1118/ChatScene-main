'''The driver was alert and driving along a surface street. Suddenly something appeared in the driver's path (e.g., pedestrian)'''
Town = 'Town03'
param map = localPath(f'../../maps/{Town}.xodr') 
param carla_map = Town
model scenic.simulators.carla.model
EGO_MODEL = "vehicle.lincoln.mkz_2017"

behavior AdvBehavior():
    do CrossingBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE) until (distance from self to egoTrajectory) < globalParameters.OPT_STOP_DISTANCE
    while True:
        take SetWalkingSpeedAction(0)

param OPT_ADV_SPEED = Range(0, 5)
param OPT_ADV_DISTANCE = Range(0, 15)
param OPT_STOP_DISTANCE = Range(0, 1)
# Identifying lane sections with no adjacent lanes
laneSecsWithNoAdjacent = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec._laneToLeft is None and laneSec._laneToRight is None:
            laneSecsWithNoAdjacent.append(laneSec)

# Selecting a random lane section from identified sections for the ego vehicle
egoLaneSec = Uniform(*laneSecsWithNoAdjacent)
egoSpawnPt = OrientedPoint in egoLaneSec.centerline

# Ego vehicle setup
ego = Car at egoSpawnPt,
    with rolename 'hero',
    with regionContainedIn None,
    with blueprint EGO_MODEL
param OPT_GEO_Y_DISTANCE = Range(10, 30)  # Frontal distance range

FrontSpawnPtOpp = OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_GEO_Y_DISTANCE
AdvAgent = Pedestrian at FrontSpawnPtOpp,
    with heading FrontSpawnPtOpp.heading + 180 deg,  # Opposite direction to the car
    with regionContainedIn None,
    with behavior AdvBehavior()