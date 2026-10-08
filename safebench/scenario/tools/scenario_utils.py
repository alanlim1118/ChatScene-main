''' 
Date: 2023-01-31 22:23:17
LastEditTime: 2023-04-03 19:26:20
Description: 
    Copyright (c) 2022-2023 Safebench Team

    This file is modified from <https://github.com/carla-simulator/scenario_runner/tree/master/srunner/tools>
    Copyright (c) 2018-2020 Intel Corporation

    This work is licensed under the terms of the MIT license.
    For a copy, see <https://opensource.org/licenses/MIT>
'''

import os
import os.path as osp
import math
import json
import random
import pickle
from copy import deepcopy

import carla
import xml.etree.ElementTree as ET

from safebench.scenario.tools.route_parser import RouteParser, TRIGGER_THRESHOLD, TRIGGER_ANGLE_THRESHOLD
from safebench.scenario.scenario_manager.scenario_config import ScenarioConfig
from safebench.util import scenario_id_manifest
from safebench.scenario.tools.scenario_dir_parse import scenario_dir_parse


def calculate_distance_transforms(transform_1, transform_2):
    distance_x = (transform_1.location.x - transform_2.location.x) ** 2
    distance_y = (transform_1.location.y - transform_2.location.y) ** 2
    return math.sqrt(distance_x + distance_y)


def calculate_distance_locations(location_1, location_2):
    distance_x = (location_1.x - location_2.x) ** 2
    distance_y = (location_1.y - location_2.y) ** 2
    return math.sqrt(distance_x + distance_y)


def _scenic_base_extra_params(config):
    """CARLA/Scenic 3.x globals merged into each ScenarioConfig.extra_params."""
    return {
        'port': config['port'],
        'traffic_manager_port': config['tm_port'],
        'use2DMap': True,
        'render': 0,
        'address': '127.0.0.1',
        'timestep': config.get('fixed_delta_seconds', 0.1),
    }


def scenario_parse(config, logger):
    """
        Data file should also come from args
    """
    ROOT_DIR = config['ROOT_DIR']
    logger.log(">> Parsing scenario route and data")
    list_of_scenario_config = osp.join(ROOT_DIR, config['scenario_type_dir'], config['scenario_type'])
    route_file_formatter = osp.join(ROOT_DIR, config['route_dir'], 'scenario_%02d_routes/scenario_%02d_route_%02d.xml')
    scenario_file_formatter = osp.join(ROOT_DIR, config['route_dir'], 'scenarios/scenario_%02d.json')

    # scenario_id, method, route_id, risk_level
    with open(list_of_scenario_config, 'r') as f:
        data_full = json.loads(f.read())
        # filter the list if any parameter is specified
        if config['scenario_id'] is not None:
            logger.log('>> Selecting scenario_id: ' + str(config['scenario_id']))
            data_full = [item for item in data_full if item["scenario_id"] == config['scenario_id']]
        if config['route_id'] is not None:
            logger.log('>> Selecting route_id: ' + str(config['route_id']))
            data_full = [item for item in data_full if item["route_id"] == config['route_id']]
    
    ### different mapping between v1 and v2 ###
    if config['mode'] == 'train_agent':
        data_full = [item for item in data_full if  item["route_id"] >= 4 and item["route_id"] < 12]
    
    logger.log(f'>> Loading {len(data_full)} data')
    data_full = [item for item in data_full if item["data_id"] not in logger.eval_records.keys()]
    logger.log(f'>> Parsing {len(data_full)} unfinished data')

    config_by_map = {}
    for item in data_full:
        route_file = route_file_formatter % (item['scenario_id'], item['scenario_id'], item['route_id'])
        scenario_file = scenario_file_formatter % item['scenario_id']
        parsed_configs = RouteParser.parse_routes_file(route_file, scenario_file)
        
        # assume one file only has one route
        assert len(parsed_configs) == 1, 'More than one route in one file'

        parsed_config = parsed_configs[0]
        parsed_config.auto_ego = config['auto_ego']
        parsed_config.num_scenario = config['num_scenario']
        parsed_config.data_id = item['data_id']
        parsed_config.scenario_folder = item["scenario_folder"]
        parsed_config.scenario_id = item['scenario_id']
        parsed_config.route_id = item['route_id']
        parsed_config.risk_level = item['risk_level']
        parsed_config.parameters = item['parameters']
        # parse the template directory from .yaml config of scenarios
        if 'texture_dir' in config.keys():
            parsed_config.texture_dir = config['texture_dir']
        
        # cluster config according to the town
        if parsed_config.town not in config_by_map:
            config_by_map[parsed_config.town] = [parsed_config]
        else:
            config_by_map[parsed_config.town].append(parsed_config)

    return config_by_map

def scenic_parse(config, logger):
    """
        Parse scenic config, especially for loading the scenic files.
    """
    if config.get('scenario_dir'):
        return scenario_dir_parse(config, logger, _scenic_base_extra_params(config))

    mode = config['mode']
    scenic_dir = config['scenic_dir']

    route_file_formatter = osp.join(config['route_dir'], 'scenic_route.pickle')
    with open(route_file_formatter, 'rb') as f:
        data_full = pickle.load(f)

    # Optional index for non-numeric route keys, e.g. "scenario_id_ChatScene__1_route_id_0"
    # Maps "{bench_id}:{scenario_id}:{route_id}" -> {"pickle_key": "...", ...}
    index_path = osp.join(config['route_dir'], 'scenic_route_index.json')
    route_index = None
    if osp.exists(index_path):
        try:
            with open(index_path, 'r') as f:
                route_index = json.load(f)
        except Exception:
            route_index = None
        
    scenic_rel_listdir = []
    scenic_abs_listdir = []
    if config['scenario_id'] is None:
        search_scenarios = [i for i in range(1, 9)]
    else:
        search_scenarios = [config['scenario_id']]
        
    scenario_ids = []
    for j in search_scenarios:
        current_scenic_dir = osp.join(scenic_dir, f'scenario_{j}')
        new_files = []
        # Primary layout: scenic_dir/scenario_{id}/*.scenic
        if osp.isdir(current_scenic_dir):
            new_files = sorted([path for path in os.listdir(current_scenic_dir) if path.split('.')[1] == 'scenic'])
            scenic_rel_listdir.extend(new_files)
            scenic_abs_listdir.extend(osp.join(current_scenic_dir, path) for path in new_files)
            scenario_ids.extend([j] * len(new_files))
        # Bench layout: scenic_dir/{bench_id}/*.scenic
        if (not new_files) and config.get('bench_id') is not None:
            bench_dir = osp.join(scenic_dir, str(config['bench_id']))
            if osp.isdir(bench_dir):
                bench_files = sorted([path for path in os.listdir(bench_dir) if path.endswith('.scenic')])
                # If scenario_id is specified, only load the matching Scenic file (e.g. scenario_002.scenic).
                if config.get('scenario_id') is not None:
                    expected = f"scenario_{int(j):03d}.scenic"
                    if expected in bench_files:
                        bench_files = [expected]
                    else:
                        # Fallback: accept any file with matching numeric suffix.
                        matched = [p for p in bench_files if p.startswith("scenario_") and p.split(".")[0].endswith(f"{int(j):03d}")]
                        if not matched:
                            # Fallback: bench dirs with non-scenario_NNN.scenic
                            # filenames resolve scenario_id via a manifest
                            # (see scripts/build_scenario_id_manifest.py).
                            manifest_file = scenario_id_manifest.file_for_id(bench_dir, int(j))
                            if manifest_file is not None and manifest_file in bench_files:
                                matched = [manifest_file]
                        bench_files = matched
                scenic_rel_listdir.extend(bench_files)
                scenic_abs_listdir.extend(osp.join(bench_dir, path) for path in bench_files)
                scenario_ids.extend([j] * len(bench_files))

    behaviors = [path.split('.')[0] for path in scenic_rel_listdir]
    assert len(scenic_rel_listdir) > 0, 'no scenic file in this dir'
    
    def _load_opt_params_for_scenario(scenario_id: int):
        """Load OPT selection JSON for eval/train_agent if present.

        Supports both:
        - scenic_dir/scenario_{id}/scenario_{id}.json (legacy layout)
        - scenic_dir/<bench_id>/scenario_{id}.json (bench layout when config['bench_id'] provided)
        """
        bench_id = config.get('bench_id')
        candidates = []
        if bench_id:
            candidates.append(osp.join(scenic_dir, str(bench_id), f"scenario_{scenario_id}.json"))
        candidates.append(osp.join(scenic_dir, f"scenario_{scenario_id}", f"scenario_{scenario_id}.json"))
        for p in candidates:
            try:
                with open(p, 'r') as f:
                    return json.load(f)
            except Exception:
                continue
        return {}

    try:
        params = _load_opt_params_for_scenario(config['scenario_id'])
    except:
        params = {}

    config_list = []
    for i, scenic_file in enumerate(scenic_abs_listdir):
        parsed_config = ScenarioConfig()
        parsed_config.auto_ego = config['auto_ego']
        parsed_config.num_scenario = config['num_scenario']
        parsed_config.data_id = i
        parsed_config.scenic_file = scenic_file
        parsed_config.behavior = behaviors[i]
        parsed_config.scenario_generation_method = config['method']
        parsed_config.scenario_id = scenario_ids[i]
        
        current_scenic_dir = osp.join(scenic_dir, f'scenario_{scenario_ids[i]}')
        try:
            params = _load_opt_params_for_scenario(scenario_ids[i])
        except:
            params = {}
        
        parsed_config.sample_num = config['sample_num']
        parsed_config.trajectory = []
        parsed_config.select_num = config['select_num']
        parsed_config.mode = config['mode']
        parsed_config.opt_step =  config['opt_step']
        parsed_config.scenic_mode2d = config.get('scenic_mode2d', True)
        parsed_config.use_opt_json = config.get('use_opt_json', True)
        parsed_config.extra_params = _scenic_base_extra_params(config)
        
        route = config['route_id']
        if route is None:
            parsed_config.route_id = None
            if mode in ['eval', 'train_agent'] and config.get('use_opt_json', True):
                parsed_config.opt_params = params[f'OPT_{behaviors[i]}']
            else:
                parsed_config.opt_params = None
            config_list.append(parsed_config)
        else:
            for j in route:
                updated_config = deepcopy(parsed_config)
                updated_config.route_id = j
                # Backward-compatible key resolution:
                # 1) numeric-only key
                # 2) index-resolved key based on bench_id (config override or parent directory name)
                data_key = f'scenario_id_{updated_config.scenario_id}_route_id_{j}'
                if data_key not in data_full and route_index is not None:
                    bench_id = config.get('bench_id')
                    if bench_id is None:
                        bench_id = osp.basename(osp.dirname(updated_config.scenic_file))
                    idx_key = f'{bench_id}:{updated_config.scenario_id}:{j}'
                    idx_entry = route_index.get(idx_key)
                    if isinstance(idx_entry, dict):
                        maybe_key = idx_entry.get('pickle_key')
                        if isinstance(maybe_key, str):
                            data_key = maybe_key
                data = data_full[data_key]
                updated_config.trajectory = data['trajectory']
                updated_config.extra_params['town'] = data['town']
                updated_config.extra_params['weather'] = data['weather']
                updated_config.extra_params['waypoints'] = data['waypoints']
                updated_config.extra_params['lanePts'] = data['lanePts']
                spawnPt = data['spawnPt']
                updated_config.extra_params['spawnPt'] = (spawnPt['x'], spawnPt['y'])
                updated_config.extra_params['z'] = spawnPt['z']
                updated_config.extra_params['yaw'] = spawnPt['yaw']
        
                if mode in ['eval', 'train_agent'] and config.get('use_opt_json', True):
                    try:
                        updated_config.opt_params = params[f'OPT_{behaviors[i]}_ROUTE-{j}']
                    except:
                        continue
                else:
                    updated_config.opt_params = None
                config_list.append(updated_config)
    
#     if mode == 'train_agent':
        ### sorted by town ###
    
    config_by_map = {}
    for config in config_list:
        if config.extra_params['town'] not in config_by_map:
            config_by_map[config.extra_params['town']] = []
        config_by_map[config.extra_params['town']].append(config)
    new_lists = list(config_by_map.values())
    config_list = [item for sublist in new_lists for item in sublist]
    return config_list

def dynamic_scenic_parse(config, logger):
    """
        Parse dynamic scenic config, especially for loading the scenic files.
    """
    mode = config['mode']
    scenic_dir = config['scenic_dir']

    scenic_rel_listdir = []
    scenic_abs_listdir = []

    new_files = sorted([path for path in os.listdir(scenic_dir) if path.split('.')[1] == 'scenic'])
    scenic_rel_listdir = new_files
    scenic_abs_listdir = [osp.join(scenic_dir, path) for path in new_files]

    behaviors = [path.split('.')[0] for path in scenic_rel_listdir]
    assert len(scenic_rel_listdir) > 0, 'no scenic file in this dir'
    
    try:
        params_dir = open(osp.join(scenic_dir, "dynamic_scenario.json"))
        params = json.load(params_dir)
    except:
        params = {}

    config_list = []
    for i, scenic_file in enumerate(scenic_abs_listdir):
        parsed_config = ScenarioConfig()
        parsed_config.auto_ego = config['auto_ego']
        parsed_config.num_scenario = config['num_scenario']
        parsed_config.data_id = i
        parsed_config.scenic_file = scenic_file
        parsed_config.behavior = behaviors[i]
        parsed_config.trajectory = []
        parsed_config.scenario_generation_method = config['method']
        parsed_config.scenario_id = None
        
        parsed_config.sample_num = config['sample_num']
        parsed_config.select_num = config['select_num']
        parsed_config.mode = config['mode']
        parsed_config.opt_step =  config['opt_step']
        parsed_config.scenic_mode2d = config.get('scenic_mode2d', True)
        parsed_config.extra_params = _scenic_base_extra_params(config)
        
        parsed_config.route_id = None
        if mode in ['eval', 'train_agent']:
            parsed_config.opt_params = params[f'OPT_{behaviors[i]}']
        else:
            parsed_config.opt_params = None
        config_list.append(parsed_config)
    
    return config_list

def get_valid_spawn_points(world):
    vehicle_spawn_points = list(world.get_map().get_spawn_points())
    random.shuffle(vehicle_spawn_points)
    actor_location_list = get_current_location_list(world)
    vehicle_spawn_points = filter_valid_spawn_points(vehicle_spawn_points, actor_location_list)
    return vehicle_spawn_points


def filter_valid_spawn_points(spawn_points, current_locations):
    dis_threshold = 8
    valid_spawn_points = []
    for spawn_point in spawn_points:
        valid = True
        for location in current_locations:
            if spawn_point.location.distance(location) < dis_threshold:
                valid = False
                break
        if valid:
            valid_spawn_points.append(spawn_point)
    return valid_spawn_points


def get_current_location_list(world):
    locations = []
    for actor in world.get_actors().filter('vehicle.*'):
        locations.append(actor.get_transform().location)
    return locations


def compare_scenarios(scenario_choice, existent_scenario):
    """
        Compare function for scenarios based on distance of the scenario start position
    """

    def transform_to_pos_vec(scenario):
        # Convert left/right/front to a meaningful CARLA position
        position_vec = [scenario['trigger_position']]
        if scenario['other_actors'] is not None:
            if 'left' in scenario['other_actors']:
                position_vec += scenario['other_actors']['left']
            if 'front' in scenario['other_actors']:
                position_vec += scenario['other_actors']['front']
            if 'right' in scenario['other_actors']:
                position_vec += scenario['other_actors']['right']
        return position_vec

    # put the positions of the scenario choice into a vec of positions to be able to compare
    choice_vec = transform_to_pos_vec(scenario_choice)
    existent_vec = transform_to_pos_vec(existent_scenario)
    for pos_choice in choice_vec:
        for pos_existent in existent_vec:

            dx = float(pos_choice['x']) - float(pos_existent['x'])
            dy = float(pos_choice['y']) - float(pos_existent['y'])
            dz = float(pos_choice['z']) - float(pos_existent['z'])
            dist_position = math.sqrt(dx * dx + dy * dy + dz * dz)
            dyaw = float(pos_choice['yaw']) - float(pos_choice['yaw'])
            dist_angle = math.sqrt(dyaw * dyaw)
            if dist_position < TRIGGER_THRESHOLD and dist_angle < TRIGGER_ANGLE_THRESHOLD:
                return True
    return False


def convert_json_to_transform(actor_dict):
    """
        Convert a JSON string to a CARLA transform
    """
    return carla.Transform(
        location=carla.Location(x=float(actor_dict['x']), y=float(actor_dict['y']), z=float(actor_dict['z'])),
        rotation=carla.Rotation(roll=0.0, pitch=0.0, yaw=float(actor_dict['yaw']))
    )


class ActorConfigurationData(object):
    """
        This is a configuration base class to hold model and transform attributes
    """
    def __init__(self, model, transform, rolename='other', speed=0, autopilot=False, random=False, color=None, category="car", args=None):
        self.model = model
        self.rolename = rolename
        self.transform = transform
        self.speed = speed
        self.autopilot = autopilot
        self.random_location = random
        self.color = color
        self.category = category
        self.args = args

    @staticmethod
    def parse_from_node(node, rolename):
        model = node.attrib.get('model', 'vehicle.*')

        pos_x = float(node.attrib.get('x', 0))
        pos_y = float(node.attrib.get('y', 0))
        pos_z = float(node.attrib.get('z', 0))
        yaw = float(node.attrib.get('yaw', 0))

        transform = carla.Transform(carla.Location(x=pos_x, y=pos_y, z=pos_z), carla.Rotation(yaw=yaw))
        rolename = node.attrib.get('rolename', rolename)
        speed = node.attrib.get('speed', 0)

        autopilot = False
        if 'autopilot' in node.keys():
            autopilot = True

        random_location = False
        if 'random_location' in node.keys():
            random_location = True

        color = node.attrib.get('color', None)

        return ActorConfigurationData(model, transform, rolename, speed, autopilot, random_location, color)


def convert_json_to_actor(actor_dict):
    """
        Convert a JSON string to an ActorConfigurationData dictionary
    """
    node = ET.Element('waypoint')
    node.set('x', actor_dict['x'])
    node.set('y', actor_dict['y'])
    node.set('z', actor_dict['z'])
    node.set('yaw', actor_dict['yaw'])

    return ActorConfigurationData.parse_from_node(node, 'simulation')


def convert_transform_to_location(transform_vec):
    """
        Convert a vector of transforms to a vector of locations
    """
    location_vec = []
    for transform_tuple in transform_vec:
        location_vec.append((transform_tuple[0].location, transform_tuple[1]))

    return location_vec
