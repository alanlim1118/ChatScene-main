description = "Ego vehicle follows a lead car which swerves to reveal a stationary obstacle, requiring the ego to brake to a full stop."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param LEAD_DISTANCE = Range(10, 15)
param OBSTACLE_DISTANCE_FROM_LEAD = Range(15, 25)

initLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on initLane.centerline

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.LEAD_DISTANCE
obstacleSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.OBSTACLE_DISTANCE_FROM_LEAD

param OPT_EGO_SPEED = Range(10, 15)
param OPT_EGO_SAFETY_DISTANCE = Range(7, 9)

behavior EgoBehavior(speed, safety_dist):
    do FollowLaneBehavior(target_speed=speed)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(globalParameters.OPT_EGO_SPEED, globalParameters.OPT_EGO_SAFETY_DISTANCE)

param ADV_SPEED = Range(10, 15)
param SWERVE_DISTANCE = Range(5, 10)

behavior LeadCarBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.SWERVE_DISTANCE):
        if self.laneSection._laneToLeft:
            do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToLeft, target_speed=globalParameters.ADV_SPEED)
        elif self.laneSection._laneToRight:
            do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToRight, target_speed=globalParameters.ADV_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)


adversary = new Car at leadSpawnPt,
    with blueprint MODEL,
    with behavior LeadCarBehavior()

Blocker = new Car at obstacleSpawnPt,
    with heading obstacleSpawnPt.heading,
    with regionContainedIn None

require distance to intersection >= 100
terminate when ego.speed < 0.1 and (distance from ego to Blocker) < 15 and not (ego intersects Blocker) and (distance from ego to egoSpawnPt) > 10