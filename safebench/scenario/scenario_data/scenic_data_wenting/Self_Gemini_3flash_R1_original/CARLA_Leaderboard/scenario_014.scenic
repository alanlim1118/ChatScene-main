description = "Ego vehicle maneuvers to avoid a parked car opening its door into the lane."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

forwardSections = []
for lane in network.lanes:
    for section in lane.sections:
        if section.isForward:
            forwardSections.append(section)

egoSection = Uniform(*forwardSections)
egoSpawnPt = new OrientedPoint on egoSection.centerline

egoEdgePos = egoSection.rightEdge.project(egoSpawnPt.position)
advSpawnPt = new OrientedPoint following egoSection.orientation from egoEdgePos for Range(30, 50)

param EGO_SPEED = Range(6, 10)
param BRAKE_DISTANCE = 5
param MANEUVER_DISTANCE = 25

behavior EgoBehavior(speed, maneuver_dist, brake_dist, obstacle_pos):
    try:
        do FollowLaneBehavior(target_speed=speed) until (distance from self to obstacle_pos) < maneuver_dist
        if self.laneSection._laneToLeft:
            do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToLeft, target_speed=speed)
        do FollowLaneBehavior(target_speed=speed)
    interrupt when withinDistanceToAnyObjs(self, brake_dist):
        take SetBrakeAction(1.0)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(
        globalParameters.EGO_SPEED, 
        globalParameters.MANEUVER_DISTANCE, 
        globalParameters.BRAKE_DISTANCE,
        advSpawnPt
    )

param ADV_TRIGGER_DIST = Range(15, 20)

behavior AdversarialBehavior(trigger_dist):
    wait until (distance from self to ego) < trigger_dist
    while True:
        take SetHandBrakeAction(True)

adversarial = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversarialBehavior(globalParameters.ADV_TRIGGER_DIST)

require 30 <= (distance from egoSpawnPt to advSpawnPt) <= 50
terminate when (distance from ego to advSpawnPt) > 70
terminate after 50 seconds