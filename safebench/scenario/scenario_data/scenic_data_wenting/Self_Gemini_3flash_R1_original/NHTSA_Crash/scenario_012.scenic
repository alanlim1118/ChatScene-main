description = "Vehicle A performs an unsafe lane change into Vehicle B's lane, resulting in a collision."
param map = localPath('../../maps/Town04.xodr')
param carla_map = 'Town04'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

road = Uniform(*filter(lambda r: r is not None and len(r.lanes) >= 4, network.roads))
egoLane = Uniform(*filter(lambda l: l is not None and any(s._laneToLeft or s._laneToRight for s in l.sections), road.lanes))
egoSection = Uniform(*filter(lambda s: s._laneToLeft or s._laneToRight, egoLane.sections))
egoSpawnPt = new OrientedPoint on egoSection.centerline
targetPt = new OrientedPoint following egoSection.orientation from egoSpawnPt for Range(10, 20)
sideIsLeft = egoSection._laneToLeft is not None
advSpawnPt = new OrientedPoint left of targetPt by 3.5 if sideIsLeft else new OrientedPoint right of targetPt by 3.5
advLane = egoSection._laneToLeft.lane if sideIsLeft else egoSection._laneToRight.lane
egoTrajectory = [egoLane]
advTrajectory = [advLane]

param EGO_SPEED = Range(10, 15)

behavior EgoBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED) for 5 seconds
    do LaneChangeBehavior(laneSectionToSwitch=self.laneSection._laneToLeft, target_speed=globalParameters.EGO_SPEED)
    do FollowLaneBehavior(target_speed=globalParameters.EGO_SPEED)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior()

param ADV_SPEED = Range(8, 12)

behavior AdversaryBehavior():
    do FollowLaneBehavior(target_speed=globalParameters.ADV_SPEED)

adversary = new Car at advSpawnPt,
    with blueprint MODEL,
    with behavior AdversaryBehavior()

require sideIsLeft
require 10 <= (distance from egoSpawnPt to advSpawnPt) <= 25
terminate when ego intersects adversary
terminate after 20 seconds