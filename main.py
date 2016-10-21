#!/usr/bin/env python

import IPython
import fcntl
import openravepy
import pickle
import sys
import time

from numpy import pi
from robot import Robot
from rrt import RRT
from trajectory import Trajectory


dof_limits = [-pi], [pi]
goal_state = [pi, 0.]
torque_limits = [10.]
velocity_limits = [10.]

bezier_rrt = None
soc_rrt = None
max_rrt_iter = 100000
steer_to_goal_every = 100
all_runs = []


def run_rrts():
    global bezier_rrt
    global soc_rrt
    bezier_rrt = RRT(
        pendulum, init_state, goal_state, Trajectory.bezier_interpolate,
        steer_to_goal_every)
    soc_rrt = RRT(
        pendulum, init_state, goal_state, Trajectory.soc_interpolate,
        steer_to_goal_every)
    samples = []
    for itnum in xrange(max_rrt_iter):
        sampled_state = pendulum.sample_state()
        samples.append(sampled_state)
        sys.stderr.write("Bezier: ")
        bezier_rrt.step(sampled_state)
        sys.stderr.write("   SOC: ")
        soc_rrt.step(sampled_state)
    for rrt in [bezier_rrt, soc_rrt]:
        if not rrt.solution:
            rrt.plot_data.append((rrt.nb_iter, rrt.dist_to_goal))
    return {
        'dof_limits': dof_limits,
        'init_state': init_state,
        'goal_state': goal_state,
        'max_rrt_iter': max_rrt_iter,
        'plots': {
            'Bezier': bezier_rrt.plot_data,
            'SOC': soc_rrt.plot_data
        },
        'steer_to_goal_every': steer_to_goal_every,
        'torque_limits': torque_limits,
        'samples': samples,
        'velocity_limits': velocity_limits
    }


def log_computation_time(dt):
    h = int(dt / 3600)
    m = int(dt - 3600 * h) / 60
    s = dt - 3600 * h - 60 * m
    sys.stderr.write("------------------------------------------\n\n")
    sys.stderr.write("Computation time:  %d h %d min %.2f s\n\n" % (h, m, s))


def save_run(new_run, fname):
    with open('benchmark/%s.pkl' % fname, 'w') as f:
        fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # file is unlocked automatically at end of with statement
        pickle.dump(new_run, f)


if __name__ == "__main__":
    interactive_mode = (len(sys.argv) < 2)
    if interactive_mode:
        print "Running in interactive mode as no run ID is provided.\n"
        print "Benchmark usage: %s <run_id>\n" % sys.argv[0]
    env = openravepy.Environment()
    env.Load('pendulum.xml')
    rave_robot = env.GetRobot("Pendulum")
    # env.SetViewer('qtcoin')
    # raw_input()
    pendulum = Robot(rave_robot, dof_limits, velocity_limits, torque_limits)
    init_state = 0.5 * pendulum.sample_state()  # dof limits: [-pi/2, +pi/2]
    t0 = time.time()
    new_run = run_rrts()
    log_computation_time(time.time() - t0)
    if interactive_mode:
        print "RRTs are stored in `bezier_rrt` and `soc_rrt`. You can try:\n"
        print "\tbezier_rrt.plot_roadmap()"
        print "\tbezier_rrt.show_solution()"
        print "\tsoc_rrt.plot_roadmap()"
        print "\tsoc_rrt.show_solution()"
        print ""
        IPython.embed()
    else:
        save_run(new_run, sys.argv[1])
        print "Done."
