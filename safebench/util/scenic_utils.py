
### Top-level functionality of the scenic package as a script:
### load a scenario and generate scenes in an infinite loop.
### modified from https://github.com/BerkeleyLearnVerify/Scenic/blob/main/src/scenic/__main__.py
### & https://github.com/BerkeleyLearnVerify/Scenic/blob/main/src/scenic/core/simulators.py

import os 
import random
import numpy as np 
import torch

import sys
import time
import argparse
import pygame
from collections import OrderedDict, defaultdict

if sys.version_info >= (3, 8):
    from importlib import metadata
else:
    import importlib_metadata as metadata

from scenic import scenarioFromFile
import scenic.core.errors as errors
import scenic.syntax.veneer as veneer
from scenic.core.simulators import SimulationCreationError, SimulationResult, TerminationType
from scenic.core.object_types import disableDynamicProxyFor
from scenic.core.distributions import RejectionException
from scenic.core.dynamics import GuardViolation, RejectSimulationException
from scenic.core.dynamics.actions import _EndSimulationAction
from scenic.core.requirements import RequirementType
from scenic.simulators.carla.simulator import CarlaSimulation


def _is_opt_range(param):
    """True if param is a Scenic Range (or similar) with optimizable bounds."""
    return hasattr(param, 'low') and hasattr(param, 'high')


def merge_scenic_global_params(user_params, fixed_delta_seconds=0.1):
    """Defaults required for Scenic 3.x + CARLA OpenDRIVE maps."""
    merged = dict(user_params)
    merged.setdefault('use2DMap', True)
    merged.setdefault('render', 0)
    merged.setdefault('address', '127.0.0.1')
    merged.setdefault('timestep', fixed_delta_seconds)
    return merged

def get_parser(scenicFile):
    parser = argparse.ArgumentParser(prog='scenic', add_help=False,
                                     usage='scenic [-h | --help] [options] FILE [options]',
                                     description='Sample from a Scenic scenario, optionally '
                                                 'running dynamic simulations.')

    mainOptions = parser.add_argument_group('main options')
    mainOptions.add_argument('-S', '--simulate', default=True,
                             help='run dynamic simulations from scenes '
                                  'instead of simply showing diagrams of scenes')
    mainOptions.add_argument('-s', '--seed', help='random seed', default=0, type=int)
    mainOptions.add_argument('-v', '--verbosity', help='verbosity level (default 1)',
                             type=int, choices=(0, 1, 2, 3), default=1)
    mainOptions.add_argument('-p', '--param', help='override a global parameter',
                             nargs=2, default=[], action='append', metavar=('PARAM', 'VALUE'))
    mainOptions.add_argument('-m', '--model', help='specify a Scenic world model', default='scenic.simulators.carla.model')
    mainOptions.add_argument('--scenario', default=None,
                             help='name of scenario to run (if file contains multiple)')

    # Simulation options
    simOpts = parser.add_argument_group('dynamic simulation options')
    simOpts.add_argument('--time', help='time bound for simulations (default none)',
                         type=int, default=10000)
    simOpts.add_argument('--count', help='number of successful simulations to run (default infinity)',
                         type=int, default=0)
    simOpts.add_argument('--max-sims-per-scene', type=int, default=1, metavar='N',
                         help='max # of rejected simulations before sampling a new scene (default 1)')

    # Interactive rendering options
    intOptions = parser.add_argument_group('static scene diagramming options')
    intOptions.add_argument('-d', '--delay', type=float,
                            help='loop automatically with this delay (in seconds) '
                                 'instead of waiting for the user to close the diagram')
    intOptions.add_argument('-z', '--zoom', type=float, default=1,
                            help='zoom expansion factor, or 0 to show the whole workspace (default 1)')

    # Debugging options
    debugOpts = parser.add_argument_group('debugging options')
    debugOpts.add_argument('--show-params', help='show values of global parameters',
                           action='store_true')
    debugOpts.add_argument('--show-records', help='show values of recorded expressions',
                           action='store_true')
    debugOpts.add_argument('-b', '--full-backtrace', help='show full internal backtraces',
                           action='store_true')
    debugOpts.add_argument('--pdb', action='store_true',
                           help='enter interactive debugger on errors (implies "-b")')
    debugOpts.add_argument('--pdb-on-reject', action='store_true',
                           help='enter interactive debugger on rejections (implies "-b")')
    ver = metadata.version('scenic')
    debugOpts.add_argument('--version', action='version', version=f'Scenic {ver}',
                           help='print Scenic version information and exit')
    debugOpts.add_argument('--dump-initial-python', help='dump initial translated Python',
                           action='store_true')
    debugOpts.add_argument('--dump-ast', help='dump final AST', action='store_true')
    debugOpts.add_argument('--dump-python', help='dump Python equivalent of final AST',
                           action='store_true')
    debugOpts.add_argument('--no-pruning', help='disable pruning', action='store_true')
    debugOpts.add_argument('--gather-stats', type=int, metavar='N',
                           help='collect timing statistics over this many scenes')
    
    parser.add_argument('--scenicFile', help='a Scenic file to run', default = scenicFile, metavar='FILE')

    parser.add_argument('-h', '--help', action='help', default=argparse.SUPPRESS,
                        help=argparse.SUPPRESS)

    # Parse arguments and set up configuration
    args = parser.parse_args(args=[])
    return args


class SafebenchCarlaSimulation(CarlaSimulation):
    """Scenic 3 CARLA simulation that spawns actors but does not auto-run the loop.

    Scenic 3's ``Simulation.__init__`` runs the full simulation and destroys actors.
    Safebench needs setup + manual stepping (``runSimulation``) with ``world.tick()``.
    """

    def __init__(self, scene, client, tm, render, record, scenario_number, **kwargs):
        self.client = client
        self.world = self.client.get_world()
        self.current_frame = None
        self.map = self.world.get_map()
        self.blueprintLib = self.world.get_blueprint_library()
        self.tm = tm
        self.render = render
        self.record = record
        self.scenario_number = scenario_number
        self.cameraManager = None

        maxSteps = kwargs.pop('maxSteps', 10000)
        name = kwargs.pop('name', 'safebench')
        timestep = kwargs.pop('timestep', None)
        verbosity = kwargs.pop('verbosity', 0)
        kwargs.pop('replay', None)
        kwargs.pop('enableReplay', None)
        kwargs.pop('allowPickle', None)
        kwargs.pop('enableDivergenceCheck', None)
        kwargs.pop('divergenceTolerance', None)
        kwargs.pop('continueAfterDivergence', None)

        self._safebench_max_steps = maxSteps
        self.screen = None
        self.result = None
        self.scene = scene
        self.objects = []
        self.trajectory = []
        self.records = defaultdict(list)
        self.currentTime = 0
        self.timestep = 1 if timestep is None else float(timestep)
        self.verbosity = verbosity
        self.name = name
        self.worker_num = 0
        self.actionSequence = []
        self.initializeReplay(None, True, False, False)

        veneer.beginSimulation(self)
        dynamicScenario = self.scene.dynamicScenario
        try:
            self.setup()
            dynamicScenario._start()
            self.updateObjects()
        except (RejectSimulationException, RejectionException, GuardViolation) as e:
            if hasattr(e, 'simulation'):
                e.simulation = self
            self._safebench_teardown()
            raise
        except SimulationCreationError:
            self._safebench_teardown()
            raise
        except Exception:
            # Any other failure during setup()/_start()/updateObjects() (e.g. a
            # scenario bug unrelated to Scenic's own rejection-sampling) still
            # left veneer.currentSimulation set, since only the two exception
            # types above triggered teardown. Every subsequent retry attempt
            # in the same process then failed `assert currentSimulation is
            # None` in veneer.beginSimulation() before doing anything real -
            # so callers need this reset regardless of exception type.
            self._safebench_teardown()
            raise

    def _safebench_teardown(self):
        try:
            self.destroy()
        except Exception:
            pass
        for obj in self.objects:
            disableDynamicProxyFor(obj)
        for agent in getattr(self, 'agents', []):
            if agent.behavior and agent.behavior._isRunning:
                agent.behavior._stop()
        for scenario in tuple(reversed(veneer.runningScenarios)):
            scenario._stop('exception', quiet=True)
        veneer.endSimulation(self)


class ScenicSimulator:
    def __init__(self, scenicFile, params, mode2D=True, fixed_delta_seconds=0.1):
        self.args = get_parser(scenicFile)
        self.mode2D = mode2D
        self.fixed_delta_seconds = fixed_delta_seconds
        errors.showInternalBacktrace = self.args.full_backtrace
        if self.args.pdb:
            errors.postMortemDebugging = True
            errors.showInternalBacktrace = True
        if self.args.pdb_on_reject:
            errors.postMortemRejections = True
            errors.showInternalBacktrace = True
        errors.verbosityLevel = self.args.verbosity
        merged_params = merge_scenic_global_params(params, fixed_delta_seconds)
        # Load scenario from file (Scenic 3.x API)
        if self.args.verbosity >= 1:
            print('Beginning scenario construction...')
        startTime = time.time()
        self.scenario = errors.callBeginningScenicTrace(
            lambda: scenarioFromFile(
                self.args.scenicFile,
                params=merged_params,
                model=self.args.model,
                scenario=self.args.scenario,
                mode2D=self.mode2D,
            )
        )
        self.opt_params, self.opt_record = self.get_params()

        totalTime = time.time() - startTime
        if self.args.verbosity >= 1:
            print(f'Scenario constructed in {totalTime:.2f} seconds.')
        self.simulator = errors.callBeginningScenicTrace(self.scenario.getSimulator)
        if hasattr(self.simulator, 'render'):
            self.simulator.render = False
        
    def get_params(self):
        all_params = self.scenario.params
        opt_record = {}
        opt_params = {}
        for param in all_params.keys():
            if not param.startswith('OPT'):
                continue
            value = all_params[param]
            if not _is_opt_range(value):
                continue
            opt_record[param] = []
            opt_params[param] = value
            opt_params[param].min = opt_params[param].low
            opt_params[param].max = opt_params[param].high
        return opt_params, opt_record
    
    def record_params(self):
        all_params = self.scene.params
        for param in self.opt_record.keys():
            self.opt_record[param].append(all_params[param])
        print("Recording params...")
              
    def update_params(self):
        print("Updating params...")
        for param in self.opt_params.keys():
            if not _is_opt_range(self.opt_params[param]):
                continue
            if len(self.opt_record[param]) > 1:
                mean = np.mean(self.opt_record[param])
                std = np.std(self.opt_record[param])
                self.opt_params[param].low = round(max(mean - std, self.opt_params[param].min), 2)
                self.opt_params[param].high = round(min(mean + std, self.opt_params[param].max), 2)
        self.scenario.params.update(self.opt_params)
        print(self.save_params())

    def load_params(self, params):
        print("Loading params...")
        for param, bounds in params.items():
            if param not in self.opt_params or not _is_opt_range(self.opt_params[param]):
                continue
            if 'low' not in bounds or 'high' not in bounds:
                continue
            self.opt_params[param].low = bounds['low']
            self.opt_params[param].high = bounds['high']
        self.scenario.params.update(self.opt_params)

    def save_params(self):
        print("Saving params...")
        save_params = {}
        for param, cur_param in self.opt_params.items():
            if not _is_opt_range(cur_param):
                continue
            save_params[param] = {'low': cur_param.low, 'high': cur_param.high}
        return save_params
        
    def generateScene(self):
        scene, iterations = errors.callBeginningScenicTrace(
            lambda: self.scenario.generate(verbosity=self.args.verbosity)
        )
        return scene, iterations
    
    def setSimulation(self, scene, max_steps=None, name=None):
        if self.args.verbosity >= 1:
            print(f'  Beginning simulation of {scene.dynamicScenario}...')
        try:
            self.scene = scene
            timestep = getattr(self.simulator, 'timestep', self.fixed_delta_seconds)
            if max_steps is None:
                max_steps = self.args.time
            self.simulator.scenario_number += 1
            scenario_number = self.simulator.scenario_number
            if name is None:
                name = f'safebench_{scenario_number}'
            self.simulation = SafebenchCarlaSimulation(
                scene,
                self.simulator.client,
                self.simulator.tm,
                self.simulator.render,
                self.simulator.record,
                scenario_number,
                maxSteps=max_steps,
                name=name,
                timestep=timestep,
                verbosity=self.args.verbosity,
            )
        except (SimulationCreationError, RejectSimulationException, RejectionException, GuardViolation) as e:
            if self.args.verbosity >= 1:
                print(f'  Failed to create simulation: {e}')
            return False
        return True

    def _ego_object(self):
        simulation = self.simulation
        ego = getattr(simulation, 'ego', None)
        if ego is not None:
            return ego
        return simulation.scene.egoObject

    def runSimulation(self):
        """Step Scenic behaviors each tick; Safebench calls world.tick() separately."""
        maxSteps = getattr(self.simulation, '_safebench_max_steps', self.args.time)
        if self.simulation.currentTime > 0:
            raise RuntimeError('tried to run a Simulation which has already run')

        dynamicScenario = self.simulation.scene.dynamicScenario
        ego_object = self._ego_object()
        terminationReason = None
        terminationType = None

        while True:
            yield self.simulation.currentTime
            if self.simulation.verbosity >= 3:
                print(f'    Time step {self.simulation.currentTime}:')

            terminationReason = dynamicScenario._step()
            terminationType = TerminationType.scenarioComplete
            self.simulation.recordCurrentState()

            newReason = dynamicScenario._runMonitors()
            if newReason is not None:
                terminationReason = newReason
                terminationType = TerminationType.terminatedByMonitor

            if terminationReason is not None:
                break
            terminationReason = dynamicScenario._checkSimulationTerminationConditions()
            if terminationReason is not None:
                terminationType = TerminationType.simulationTerminationCondition
                break
            if maxSteps and self.simulation.currentTime >= maxSteps:
                terminationReason = f'reached time limit ({maxSteps} steps)'
                terminationType = TerminationType.timeLimit
                break

            allActions = OrderedDict()
            for agent in self.simulation.scheduleForAgents():
                if ego_object is not None and agent is ego_object:
                    continue
                if not agent.behavior:
                    continue
                actions = agent.behavior._step()
                if isinstance(actions, _EndSimulationAction):
                    terminationReason = str(actions)
                    terminationType = TerminationType.terminatedByBehavior
                    break
                if isinstance(actions, tuple) and len(actions) == 1 and isinstance(actions[0], (list, tuple)):
                    actions = tuple(actions[0])
                allActions[agent] = actions

            if terminationReason is not None:
                break

            if self.simulation.verbosity >= 3:
                for agent, actions in allActions.items():
                    print(f'      Agent {agent} takes action(s) {actions}')
            self.simulation.actionSequence.append(allActions)
            self.simulation.executeActions(allActions)
            self.simulation.updateObjects()
            self.simulation.currentTime += 1

            self.simulation.result = SimulationResult(
                self.simulation.trajectory,
                self.simulation.actionSequence,
                terminationType,
                terminationReason,
                self.simulation.records,
            )

    def endSimulation(self):
        if not hasattr(self, 'simulation'):
            return

        for scenario in tuple(veneer.runningScenarios):
            try:
                scenario._stop('simulation terminated')
            except (RejectSimulationException, RejectionException, GuardViolation):
                # Scenario._stop(reason, quiet=False) re-checks temporal
                # `require`s and raises if one was left unsatisfied - which
                # can happen even on an otherwise-normal termination (e.g. a
                # dynamic requirement was briefly violated right as the
                # scenario ended for an unrelated reason). We're already
                # tearing this simulation down; a rejection here has nowhere
                # to reject into, so don't let it abort cleanup of the
                # remaining running scenarios or the rest of endSimulation().
                pass
        for scenario in tuple(veneer.runningScenarios):
            scenario._stop('exception', quiet=True)

        dynamicScenario = self.simulation.scene.dynamicScenario
        values = dynamicScenario._evaluateRecordedExprs(
            RequirementType.recordFinal, self.simulation.currentTime
        )
        for name, val in values.items():
            self.simulation.records[name] = val

        self.simulation.destroy()
        for obj in self.simulation.scene.objects:
            disableDynamicProxyFor(obj)
        for agent in self.simulation.agents:
            if agent.behavior and agent.behavior._isRunning:
                agent.behavior._stop()
        for monitor in getattr(self.simulation.scene, 'monitors', []):
            if monitor._isRunning:
                monitor._stop()
        veneer.endSimulation(self.simulation)
        
    def destroy(self):
        self.simulator.destroy()
