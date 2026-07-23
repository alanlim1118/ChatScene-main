"""Scenario Description:

The ego car travels straight forward within its lane. It follows a adversarial object driving ahead that is overlapping its lane to the left.

"""

#################################
# MAP AND MODEL                 #
#################################

Town = 'Town05'
param map = localPath(f'../../assets/maps/CARLA/{Town}.xodr')
param carla_map = 'Town05'
model scenic.simulators.carla.model
from scenic.domains.driving.controllers import *

#################################
# CONSTANTS                     #
#################################

EGO_MODEL = "vehicle.lincoln.mkz_2017"

param OPT_EGO_SPEED = Range(5, 10)
param OPT_ADV_SPEED = Range(2, 5)

#################################
# AGENT BEHAVIORS               #
#################################

behavior FollowRightEdgeBehavior(target_speed = 10, trajectory = None):
    """
    Follows the right edge of the given trajectory.
    """
    assert trajectory is not None
    assert isinstance(trajectory, list)

    distanceToEndpoint = 5

    has_right = all(hasattr(traj, 'rightEdge') and traj.rightEdge is not None for traj in trajectory)
    if not has_right:
        raise Exception("RightEdge does not exist for the given trajectory.")

    traj_edge = [traj.rightEdge for traj in trajectory]
    trajectory_edge = concatenateCenterlines(traj_edge)

    _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)
    past_steer_angle = 0
    end_point = trajectory_edge[-1]

    while True:
        if (distance from self to end_point) < distanceToEndpoint:
            break

        current_speed = self.speed if self.speed is not None else 0
        cte = trajectory_edge.signedDistanceTo(self.position)
        speed_error = target_speed - current_speed

        throttle = _lon_controller.run_step(speed_error)
        current_steer_angle = _lat_controller.run_step(cte)

        take RegulatedControlAction(throttle, current_steer_angle, past_steer_angle)
        past_steer_angle = current_steer_angle

#################################
# SPATIAL RELATIONS             #
#################################

# Identify lane sections with a left lane
laneSecsWithLeftLane = []
for lane in network.lanes:
    for laneSec in lane.sections:
        if laneSec.isForward and laneSec._laneToLeft is not None and laneSec._laneToLeft.isForward:
            laneSecsWithLeftLane.append(laneSec)

egoLaneSec = Uniform(*laneSecsWithLeftLane)
egoSpawnPt = new OrientedPoint in egoLaneSec.centerline

advLaneSec = egoLaneSec._laneToLeft

# Place adversarial ahead on the left lane but offset rightward so it overlaps the ego lane
aheadPt = new OrientedPoint following egoSpawnPt.heading from egoSpawnPt for Range(10, 30)
advCenter = advLaneSec.centerline.project(aheadPt.position)
advCenterPt = new OrientedPoint at advCenter, facing egoSpawnPt.heading
advPos = advCenterPt offset by (0, -1.0)

#################################
# SCENARIO SPECIFICATION        #
#################################

ego = new Car at egoSpawnPt,
    with regionContainedIn egoLaneSec,
    with blueprint EGO_MODEL,
    with behavior FollowLaneBehavior(target_speed=globalParameters.OPT_EGO_SPEED)

AdvAgent = new Car at advPos,
    with heading egoSpawnPt.heading,
    with regionContainedIn advLaneSec,
    with behavior FollowRightEdgeBehavior(target_speed=globalParameters.OPT_ADV_SPEED, trajectory=[advLaneSec])

require distance to intersection >= 100
require 10 < (distance from ego to AdvAgent) < 35
terminate when distance from ego to AdvAgent > 50