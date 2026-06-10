description = "Ego vehicle follows a lead car which swerves to reveal a stationary obstacle, requiring the ego to brake to a full stop."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_LEAD_DISTANCE = Range(10, 15)
param OPT_OBSTACLE_DISTANCE_FROM_LEAD = Range(15, 25)

EgoSpawnPt = globalParameters.spawnPt
yaw = globalParameters.yaw
egoSpawnPt = new OrientedPoint at EgoSpawnPt, facing yaw

leadSpawnPt = new OrientedPoint following roadDirection from egoSpawnPt for globalParameters.OPT_LEAD_DISTANCE
obstacleSpawnPt = new OrientedPoint following roadDirection from leadSpawnPt for globalParameters.OPT_OBSTACLE_DISTANCE_FROM_LEAD

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL

param OPT_ADV_SPEED = Range(10, 15)
param OPT_SWERVE_DISTANCE = Range(5, 10)

behavior LeadCarBehavior():
    try:
        do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)
    interrupt when withinDistanceToAnyObjs(self, globalParameters.OPT_SWERVE_DISTANCE):
        if self.laneSection._laneToLeft:
            do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToLeft, target_speed=globalParameters.OPT_ADV_SPEED)
        elif self.laneSection._laneToRight:
            do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToRight, target_speed=globalParameters.OPT_ADV_SPEED)
        do FollowLaneBehavior(target_speed=globalParameters.OPT_ADV_SPEED)

adversary = new Car at leadSpawnPt,
    with blueprint MODEL,
    with behavior LeadCarBehavior()

Blocker = new Car at obstacleSpawnPt,
    with heading obstacleSpawnPt.heading,
    with regionContainedIn None

require distance to intersection >= 100
terminate when ego.speed < 0.1 and (distance from ego to Blocker) < 15 and not (ego intersects Blocker) and (distance from ego to egoSpawnPt) > 10
