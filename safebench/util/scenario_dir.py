'''
Self-contained per-scenario directory layout for the Scenic pipeline.

Each scenario lives in its own directory, named by its scenario id, and every
artifact for that scenario is kept next to the Scenic code:

    <dataset>/
      assets/maps/CARLA/<Town>.xodr      # optional, referenced by the .scenic files
      <scenario_id>/
        <scenario_id>.scenic              # original Scenic code (ego scripted)
        <scenario_id>_route_driven.scenic # optional route-driven variant
        route.pickle                      # written by generate_scenic_route_pickle.py
        route_error.txt                   # last route-generation failure, if any
        video/<scenario_id>_fpv.mp4       # recorded during route generation
        video/<scenario_id>_bev.mp4
        opt_params.json                   # written by train_scenario, read by eval

No numeric ids, route index or manifest are involved: a scenario is identified
by its directory, and its route is whatever `route.pickle` sits in it.
'''

import os
import os.path as osp
import pickle
import re
import tempfile
from typing import Dict, List, Optional, Tuple

ROUTE_FILE = 'route.pickle'
ROUTE_ERROR_FILE = 'route_error.txt'
OPT_PARAMS_FILE = 'opt_params.json'
VIDEO_DIR = 'video'
ROUTE_DRIVEN_SUFFIX = '_route_driven'

ROUTE_KEYS = ('town', 'weather', 'spawnPt', 'trajectory', 'waypoints', 'lanePts')

_CARLA_MAP_LITERAL_RE = re.compile(r"^\s*param\s+carla_map\s*=\s*['\"]([^'\"]+)['\"]\s*$", re.MULTILINE)
_CARLA_MAP_IDENT_RE = re.compile(r"^\s*param\s+carla_map\s*=\s*([A-Za-z_]\w*)\s*$", re.MULTILINE)


def scenario_id(scenario_dir: str) -> str:
    return osp.basename(osp.normpath(scenario_dir))


def source_scenic_file(scenario_dir: str) -> str:
    """The original Scenic file (`<id>/<id>.scenic`), used for route generation."""
    return osp.join(scenario_dir, f'{scenario_id(scenario_dir)}.scenic')


def route_driven_scenic_file(scenario_dir: str) -> str:
    return osp.join(scenario_dir, f'{scenario_id(scenario_dir)}{ROUTE_DRIVEN_SUFFIX}.scenic')


def eval_scenic_file(scenario_dir: str) -> str:
    """Scenic file for train_scenario/eval: the route-driven variant if present,
    otherwise the original file."""
    route_driven = route_driven_scenic_file(scenario_dir)
    return route_driven if osp.isfile(route_driven) else source_scenic_file(scenario_dir)


def is_scenario_dir(path: str) -> bool:
    return osp.isdir(path) and osp.isfile(source_scenic_file(path))


def resolve(path: str) -> Tuple[str, Optional[str]]:
    """Resolve a scenario directory or a .scenic file inside one.

    Returns (scenario_dir, scenic_file); scenic_file is None when a directory
    was given, meaning "use the default file for the task".
    """
    path = osp.abspath(path)
    if osp.isfile(path) and path.endswith('.scenic'):
        return osp.dirname(path), path
    if is_scenario_dir(path):
        return path, None
    raise FileNotFoundError(
        f'{path} is neither a .scenic file nor a scenario directory '
        f'(expected <dir>/<dir_name>.scenic)'
    )


def discover(root: str) -> List[str]:
    """Scenario directories at `root`: root itself if it is one, otherwise its
    immediate subdirectories that are."""
    root = osp.abspath(root)
    if is_scenario_dir(root):
        return [root]
    if not osp.isdir(root):
        raise FileNotFoundError(f'Not a directory: {root}')
    return sorted(
        osp.join(root, name) for name in os.listdir(root)
        if is_scenario_dir(osp.join(root, name))
    )


def dataset_name(scenario_dir: str) -> str:
    """Name of the dataset directory holding the scenario (used for log paths)."""
    return osp.basename(osp.dirname(osp.normpath(scenario_dir)))


def route_path(scenario_dir: str) -> str:
    return osp.join(scenario_dir, ROUTE_FILE)


def route_error_path(scenario_dir: str) -> str:
    return osp.join(scenario_dir, ROUTE_ERROR_FILE)


def opt_params_path(scenario_dir: str) -> str:
    return osp.join(scenario_dir, OPT_PARAMS_FILE)


def video_dir(scenario_dir: str) -> str:
    return osp.join(scenario_dir, VIDEO_DIR)


def has_route(scenario_dir: str) -> bool:
    return osp.isfile(route_path(scenario_dir))


def load_route(scenario_dir: str) -> Dict:
    path = route_path(scenario_dir)
    if not osp.isfile(path):
        raise FileNotFoundError(
            f'No route for scenario {scenario_id(scenario_dir)}: {path} does not exist. '
            f'Generate it with: python scripts/generate_scenic_route_pickle.py --scenario-dir {scenario_dir} '
            f'(Scenic 3) or scripts/generate_scenic_route_pickle_v2.py (Scenic 2)'
        )
    with open(path, 'rb') as f:
        route = pickle.load(f)
    missing = [k for k in ROUTE_KEYS if k not in route]
    if missing:
        raise ValueError(f'{path} is missing keys: {missing}')
    return route


def save_route(scenario_dir: str, route: Dict) -> str:
    """Atomically write route.pickle (a crash mid-write never leaves a
    truncated file that would later be mistaken for a finished route)."""
    path = route_path(scenario_dir)
    fd, tmp = tempfile.mkstemp(dir=scenario_dir, prefix='.route.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'wb') as f:
            pickle.dump(route, f, protocol=pickle.HIGHEST_PROTOCOL)
        # mkstemp creates 0600; give the file the same permissions a plain open() would.
        umask = os.umask(0)
        os.umask(umask)
        os.chmod(tmp, 0o666 & ~umask)
        os.replace(tmp, path)
    except BaseException:
        if osp.exists(tmp):
            os.remove(tmp)
        raise
    return path


def is_empty_scenic(scenic_file: str) -> bool:
    with open(scenic_file, 'r', encoding='utf-8') as f:
        return not f.read().strip()


def parse_carla_map(scenic_text: str) -> Optional[str]:
    """Town from `param carla_map = 'Town05'`, or from the indirect form

        Town = 'Town05'
        param carla_map = Town
    """
    m = _CARLA_MAP_LITERAL_RE.search(scenic_text)
    if m:
        return m.group(1)
    m = _CARLA_MAP_IDENT_RE.search(scenic_text)
    if m:
        ident = re.escape(m.group(1))
        assigns = re.findall(rf"^\s*{ident}\s*=\s*['\"]([^'\"]+)['\"]\s*$", scenic_text, re.MULTILINE)
        if len(set(assigns)) == 1:
            return assigns[0]
    return None
