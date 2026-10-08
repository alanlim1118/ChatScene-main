
### Scenic 2.x counterpart of `safebench.util.scenic_utils`.
### Works with the bundled `Scenic/` checkout (2.1.0b4) used by `scenic2-venv`.
### Do NOT import this module in the Scenic 3 pipeline; use `scenic_utils` there.
###
### Key differences vs the Scenic 3 version:
###   - `scenarioFromFile` has no `mode2D` kwarg (Scenic 2 is inherently 2D).
###   - `EndSimulationAction` lives in `scenic.core.simulators` (no underscore).
###   - `Simulation.__init__` does not auto-run the loop, so no
###     `SafebenchCarlaSimulation` workaround is needed: `createSimulation()`
###     spawns actors and we replicate the `Simulation.run()` preamble manually.

import sys
import time
import argparse

import numpy as np

if sys.version_info >= (3, 8):
    from importlib import metadata
else:
    import importlib_metadata as metadata

from scenic import scenarioFromFile
import scenic.core.errors as errors
import scenic.syntax.veneer as veneer
from scenic.core.simulators import (
    SimulationCreationError,
    RejectSimulationException,
    EndSimulationAction,
    TerminationType,
)
from scenic.core.object_types import disableDynamicProxyFor
from scenic.core.distributions import RejectionException
from scenic.core.dynamics import GuardViolation
from scenic.core.requirements import RequirementType


def _is_opt_range(param):
    """True if param is a Scenic Range (or similar) with optimizable bounds."""
    return hasattr(param, 'low') and hasattr(param, 'high')


def merge_scenic_global_params(user_params, fixed_delta_seconds=0.1):
    """Defaults required for Scenic 2.x + CARLA maps."""
    merged = dict(user_params)
    merged.pop('use2DMap', None)  # Scenic 3-only concept
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

    parser.add_argument('--scenicFile', help='a Scenic file to run', default=scenicFile, metavar='FILE')

    parser.add_argument('-h', '--help', action='help', default=argparse.SUPPRESS,
                        help=argparse.SUPPRESS)

    args = parser.parse_args(args=[])
    return args


def _release_synchronous_mode(params):
    """Put the CARLA server back in asynchronous mode before Scenic 2 connects.

    Scenic 2's CarlaSimulator calls client.load_world() unconditionally. If a previous
    run (Scenic 3 route generation, an earlier eval, ...) left the server in synchronous
    mode, load_world waits for a client tick that never comes and the server hangs.
    CarlaSimulator re-enables synchronous mode right after the map is loaded, so the
    simulation itself still runs synchronously.
    """
    import carla
    client = carla.Client(params.get('address', '127.0.0.1'), int(params.get('port', 2000)))
    client.set_timeout(float(params.get('timeout', 10.0)))
    world = client.get_world()
    settings = world.get_settings()
    if settings.synchronous_mode:
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)


class ScenicSimulator:
    """Scenic 2.x version: compile scenario, sample scenes, set up CARLA simulations.

    Public interface mirrors `safebench.util.scenic_utils.ScenicSimulator`
    (minus `mode2D`, which does not exist in Scenic 2).
    """

    def __init__(self, scenicFile, params, fixed_delta_seconds=0.1):
        self.args = get_parser(scenicFile)
        self.fixed_delta_seconds = fixed_delta_seconds
        self.simulation = None
        errors.showInternalBacktrace = self.args.full_backtrace
        if self.args.pdb:
            errors.postMortemDebugging = True
            errors.showInternalBacktrace = True
        if self.args.pdb_on_reject:
            errors.postMortemRejections = True
            errors.showInternalBacktrace = True
        errors.verbosityLevel = self.args.verbosity
        merged_params = merge_scenic_global_params(params, fixed_delta_seconds)
        if self.args.verbosity >= 1:
            print('Beginning scenario construction...')
        startTime = time.time()
        self.scenario = errors.callBeginningScenicTrace(
            lambda: scenarioFromFile(
                self.args.scenicFile,
                params=merged_params,
                model=self.args.model,
                scenario=self.args.scenario,
            )
        )
        self.opt_params, self.opt_record = self.get_params()

        totalTime = time.time() - startTime
        if self.args.verbosity >= 1:
            print(f'Scenario constructed in {totalTime:.2f} seconds.')
        _release_synchronous_mode(merged_params)
        # CarlaSimulator.__init__ loads the map, then switches the world and TM back to
        # synchronous mode with fixed_delta_seconds = timestep.
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
        """Create a CARLA simulation (spawning actors) and run the Scenic 2
        `Simulation.run()` preamble, without entering the stepping loop."""
        if self.args.verbosity >= 1:
            print(f'  Beginning simulation of {scene.dynamicScenario}...')
        veneer_begun = False
        try:
            self.scene = scene
            if max_steps is None:
                max_steps = self.args.time
            self._max_steps = max_steps
            # Spawns all actors (raises SimulationCreationError on spawn failure).
            self.simulation = self.simulator.createSimulation(scene, verbosity=self.args.verbosity)
            sim = self.simulation

            # Mirror the top of Scenic 2's Simulation.run().
            veneer.beginSimulation(sim)
            veneer_begun = True
            dynamicScenario = scene.dynamicScenario
            dynamicScenario._start()
            for obj in sim.objects:
                obj.startDynamicSimulation()
            sim.updateObjects()
            sim._safebench_max_steps = max_steps
        except (SimulationCreationError, RejectSimulationException, RejectionException, GuardViolation) as e:
            if self.args.verbosity >= 1:
                print(f'  Failed to create simulation: {e}')
            self._teardown(veneer_begun)
            return False
        return True

    def _teardown(self, veneer_begun=True):
        """Best-effort cleanup after a failed or finished simulation."""
        sim = self.simulation
        if sim is None:
            return
        try:
            sim.destroy()
        except Exception:
            pass
        for obj in sim.scene.objects:
            try:
                disableDynamicProxyFor(obj)
            except Exception:
                pass
        for agent in getattr(sim, 'agents', []):
            try:
                if agent.behavior and agent.behavior._isRunning:
                    agent.behavior._stop()
            except Exception:
                pass
        for monitor in getattr(sim.scene, 'monitors', []):
            try:
                if monitor._isRunning:
                    monitor._stop()
            except Exception:
                pass
        for scenario in tuple(veneer.runningScenarios):
            try:
                scenario._stop('exception', quiet=True)
            except Exception:
                pass
        if veneer_begun:
            try:
                veneer.endSimulation(sim)
            except Exception:
                pass
        self.simulation = None

    def _ego_object(self):
        simulation = self.simulation
        ego = getattr(simulation, 'ego', None)
        if ego is not None:
            return ego
        return simulation.scene.egoObject

    def runSimulation(self):
        """Step Scenic behaviors each tick; Safebench calls world.tick() separately."""
        sim = self.simulation
        maxSteps = getattr(sim, '_safebench_max_steps', self._max_steps)
        if sim.currentTime > 0:
            raise RuntimeError('tried to run a Simulation which has already run')

        dynamicScenario = sim.scene.dynamicScenario
        ego_object = self._ego_object()
        terminationReason = None
        terminationType = None
        actionSequence = []

        while True:
            yield sim.currentTime
            if sim.verbosity >= 3:
                print(f'    Time step {sim.currentTime}:')

            terminationReason = dynamicScenario._step()
            terminationType = TerminationType.scenarioComplete
            sim.recordCurrentState()

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
            if maxSteps and sim.currentTime >= maxSteps:
                terminationReason = f'reached time limit ({maxSteps} steps)'
                terminationType = TerminationType.timeLimit
                break

            allActions = {}
            for agent in sim.scheduleForAgents():
                if ego_object is not None and agent is ego_object:
                    continue
                behavior = getattr(agent, 'behavior', None)
                if not behavior:
                    continue
                if not behavior._runningIterator:
                    behavior._start(agent)
                try:
                    actions = behavior._step()
                except (RejectionException, GuardViolation) as e:
                    # Agent temporarily off-road or in invalid state; skip its
                    # action this tick so the simulation can continue.
                    print(f'[ScenicSimulator] behavior step skipped for {agent}: {e}')
                    actions = ()
                except RejectSimulationException:
                    raise
                if isinstance(actions, EndSimulationAction):
                    terminationReason = str(actions)
                    terminationType = TerminationType.terminatedByBehavior
                    break
                if isinstance(actions, tuple) and len(actions) == 1 and isinstance(actions[0], (list, tuple)):
                    actions = tuple(actions[0])
                allActions[agent] = actions

            if terminationReason is not None:
                break

            if sim.verbosity >= 3:
                for agent, actions in allActions.items():
                    print(f'      Agent {agent} takes action(s) {actions}')
            actionSequence.append(allActions)
            sim.executeActions(allActions)
            sim.updateObjects()
            sim.currentTime += 1

    def endSimulation(self):
        """Mirror the epilogue + finally block of Scenic 2's Simulation.run()."""
        sim = self.simulation
        if sim is None:
            return
        try:
            for scenario in tuple(veneer.runningScenarios):
                scenario._stop('simulation terminated')
            values = sim.scene.dynamicScenario._evaluateRecordedExprs(RequirementType.recordFinal)
            for name, val in values.items():
                sim.records[name] = val
        finally:
            self._teardown(veneer_begun=True)

    def destroy(self):
        self.simulator.destroy()
