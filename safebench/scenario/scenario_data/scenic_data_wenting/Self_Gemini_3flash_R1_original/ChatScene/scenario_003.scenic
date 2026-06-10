description = "Ego drives straight; pedestrian hidden behind vending machine on right front suddenly dashes out and stops in ego's path."
param map = localPath('../../maps/Town05.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
MODEL = 'vehicle.lincoln.mkz_2017'
param weather = 'ClearNoon'

param OPT_DIST_TO_BLOCKER = Range(20, 30)
param OPT_LATERAL_OFFSET = Range(3.5, 5.0)

egoLane = Uniform(*network.lanes)
egoSpawnPt = new OrientedPoint on egoLane.centerline

pedestrianGoalPt = new OrientedPoint following egoLane.orientation from egoSpawnPt for globalParameters.OPT_DIST_TO_BLOCKER

vendingMachineSpawnPt = new OrientedPoint right of pedestrianGoalPt by globalParameters.OPT_LATERAL_OFFSET
pedestrianSpawnPt = new OrientedPoint right of vendingMachineSpawnPt by 1.5, facing toward pedestrianGoalPt

egoTrajectory = [egoLane]

param EGO_SPEED = Range(7, 10)

behavior EgoBehavior(trajectory):
    do FollowTrajectoryBehavior(target_speed=globalParameters.EGO_SPEED, trajectory=trajectory)

ego = new Car at egoSpawnPt,
    with rolename 'hero',
    with blueprint MODEL,
    with behavior EgoBehavior(egoTrajectory)

param OPT_ADV_SPEED = Range(1, 5)
param OPT_ADV_DISTANCE = Range(15, 20)
param OPT_STOP_DISTANCE = 0.8

behavior CrossAndStopBehavior(actor_reference, adv_speed, adv_distance, stop_reference, stop_distance):
    do CrossingBehavior(actor_reference, min_speed=adv_speed, threshold=adv_distance) until (distance from self to stop_reference <= stop_distance)
    take SetWalkingSpeedAction(0)

pedestrian = new Pedestrian at pedestrianSpawnPt,
    with behavior CrossAndStopBehavior(ego, globalParameters.OPT_ADV_SPEED, globalParameters.OPT_ADV_DISTANCE, pedestrianGoalPt, globalParameters.OPT_STOP_DISTANCE)

vendingMachine = new VendingMachine at vendingMachineSpawnPt,
    with heading vendingMachineSpawnPt.heading,
    with regionContainedIn None

require not (ego can see pedestrian)
terminate when (distance from ego to egoSpawnPt) > (globalParameters.OPT_DIST_TO_BLOCKER + 10)