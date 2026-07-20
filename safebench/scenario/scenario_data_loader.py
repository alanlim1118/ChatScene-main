''' 
Date: 2023-01-31 22:23:17
LastEditTime: 2023-03-07 01:28:53
Description: 
    Copyright (c) 2022-2023 Safebench Team

    This work is licensed under the terms of the MIT license.
    For a copy, see <https://opensource.org/licenses/MIT>
'''

import numpy as np
import carla
from copy import deepcopy
import random
from safebench.scenario.tools.route_manipulation import interpolate_trajectory


def calculate_interpolate_trajectory(config, world):
    # get route
    origin_waypoints_loc = []
    for loc in config.trajectory:
        origin_waypoints_loc.append(loc)
    route = interpolate_trajectory(world, origin_waypoints_loc, 5.0)

    # get [x, y] along the route
    waypoint_xy = []
    for transform_tuple in route:
        waypoint_xy.append([transform_tuple[0].location.x, transform_tuple[0].location.y])

    return waypoint_xy


def check_route_overlap(current_routes, route, distance_threshold=10):
    overlap = False
    for current_route in current_routes:
        for current_waypoint in current_route:
            for waypoint in route:
                distance = np.linalg.norm([current_waypoint[0] - waypoint[0], current_waypoint[1] - waypoint[1]])
                if distance < distance_threshold:
                    overlap = True
                    return overlap

    return overlap

class ScenarioDataLoader:
    def __init__(self, config_lists, num_scenario, town, world):
        self.num_scenario = num_scenario
        self.config_lists = config_lists
        self.town = town.lower()
        self.world = world
        self.routes = []

        # If using CARLA maps, manually check overlaps
        if 'safebench' not in self.town:
            for config in config_lists:
                self.routes.append(calculate_interpolate_trajectory(config, world))

        self.num_total_scenario = len(config_lists)
        self.reset_idx_counter()

    def reset_idx_counter(self):
        self.scenario_idx = list(range(self.num_total_scenario))

    def _select_non_overlap_idx_safebench(self, remaining_ids, sample_num):
        selected_idx = []
        current_regions = []
        for s_i in remaining_ids:
            if self.config_lists[s_i].route_region not in current_regions:
                selected_idx.append(s_i)
                if self.config_lists[s_i].route_region != "random":
                    current_regions.append(self.config_lists[s_i].route_region)
            if len(selected_idx) >= sample_num:
                break
        return selected_idx

    def _select_non_overlap_idx_carla(self, remaining_ids, sample_num):
        selected_idx = []
        selected_routes = []
        for s_i in remaining_ids:
            if not check_route_overlap(selected_routes, self.routes[s_i]):
                selected_idx.append(s_i)
                selected_routes.append(self.routes[s_i])
            if len(selected_idx) >= sample_num:
                break
        return selected_idx

    def _select_non_overlap_idx(self, remaining_ids, sample_num):
        if 'safebench' in self.town:
            # If using SafeBench map, check overlap based on regions
            return self._select_non_overlap_idx_safebench(remaining_ids, sample_num)
        else:
            # If using CARLA maps, manually check overlaps
            return self._select_non_overlap_idx_carla(remaining_ids, sample_num)

    def __len__(self):
        return len(self.scenario_idx)

    def sampler(self):
        # sometimes the length of list is smaller than num_scenario
        sample_num = np.min([self.num_scenario, len(self.scenario_idx)])
        # select scenarios
#         selected_idx = np.random.choice(self.scenario_idx, size=sample_num, replace=False)
        selected_idx = self._select_non_overlap_idx(self.scenario_idx, sample_num)
        selected_scenario = []
        for s_i in selected_idx:
            selected_scenario.append(self.config_lists[s_i])
            self.scenario_idx.remove(s_i)

        assert len(selected_scenario) <= self.num_scenario, f"number of scenarios is larger than {self.num_scenario}"
        return selected_scenario, len(selected_scenario)

class ScenicDataLoader:
    def __init__(self, scenic, config, num_scenario):
        self.num_scenario = num_scenario
        self.config = config
        self.behavior = config.behavior
        self.select_num = config.select_num
        self.sample_num = config.sample_num
        self.opt_params = config.opt_params
        self.mode = config.mode
        self.route_id = config.route_id
        self.opt_step = config.opt_step
        self.use_opt_json = getattr(config, 'use_opt_json', True)
        self.scenic = scenic
        self.scene = []
        
        if self.mode in ['eval', 'train_agent'] and self.use_opt_json:
            self.select_id = self.opt_params['select_id']
            for j in range(self.sample_num//self.opt_step):
                self.generate_scene(j, self.opt_step, config.opt_params[f'opt_time_{j}'])
            if len(self.scene) < self.sample_num:
                self.generate_scene(j+1, self.sample_num - len(self.scene), config.opt_params[f'opt_time_{j+1}'])
        else:
            self.train_scene()
        self.reset_idx_counter()
        
    def generate_scene(self, opt_time, sample_num, params = None):
        if params is not None:
            self.scenic.load_params(params)
        random.seed(opt_time)
        scenes = []
        while len(scenes) < sample_num:
            scene, _ = self.scenic.generateScene()
            if self.scenic.setSimulation(
                scene, max_steps=1, name=f'validate_{opt_time}_{len(scenes)}'
            ):
                scenes.append(scene)
                self.scenic.endSimulation()
        self.scene.extend(scenes)
            
    def reset_idx_counter(self):
        if self.mode in ['eval', 'train_agent'] and self.use_opt_json:
            self.num_total_scenario = len(self.select_id)
            self.scenario_idx = self.select_id
        else:
            self.num_total_scenario = self.sample_num
            self.scenario_idx = list(range(self.num_total_scenario))

    def train_scene(self, opt_time = 0):
        if (opt_time + 1) * self.opt_step <= self.sample_num:
            self.generate_scene(opt_time, self.opt_step)
        else:
            self.generate_scene(opt_time, self.sample_num - len(self.scene))
        
    def sampler(self):
        ## no need to be random for scenic loading file ###
        selected_scenario = []
        idx = self.scenario_idx.pop(0)
        new_config = deepcopy(self.config)
        new_config.scene = self.scene[idx]
        new_config.data_id = idx
        route_format = getattr(new_config, 'route_format', None) or {}
        traj_format = route_format.get('trajectory', 'scenic')
        waypoint_format = route_format.get('waypoints', 'scenic')
        if len(new_config.trajectory) != 0:
            new_config.trajectory = self.toCarlaLocations(new_config.trajectory, traj_format)
        elif 'egoTrajectoryPts' in new_config.scene.params:
            new_config.trajectory = self.toCarlaLocations(new_config.scene.params['egoTrajectoryPts'], 'scenic')
        elif 'waypoints' in new_config.scene.params:
            new_config.trajectory = self.toCarlaLocations(new_config.scene.params['waypoints'], waypoint_format)
        else:
            ego_traj = new_config.scene.params.get('egoTrajectory')
            if ego_traj is not None and hasattr(ego_traj, 'points'):
                new_config.trajectory = self.toCarlaLocations(ego_traj.points, 'scenic')
            else:
                new_config.trajectory = []
        selected_scenario.append(new_config)
        assert len(selected_scenario) <= self.num_scenario, f"number of scenarios is larger than {self.num_scenario}"
        return selected_scenario, len(selected_scenario)

    def toCarlaLocations(self, points, coord_format='scenic'):
        """Convert route points to CARLA locations using the pickle route_format."""
        waypoints = []
        for point in points:
            if isinstance(point, carla.Location):
                waypoints.append(point)
                continue
            if len(point) == 3:
                x, y, z = float(point[0]), float(point[1]), float(point[2])
            else:
                x, y, z = float(point[0]), float(point[1]), 0.0
            if coord_format == 'carla':
                waypoints.append(carla.Location(x, y, z))
            else:
                waypoints.append(carla.Location(x, -y, z))
        return waypoints

    def scenicToCarlaLocation(self, points):
        return self.toCarlaLocations(points, 'scenic')
    
    def __len__(self):
        return len(self.scenario_idx)