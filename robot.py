#!/usr/bin/env python

import pylab
import time

from numpy import all, array, cos, hstack, linspace, sqrt


class Robot(object):

    def __init__(self, rave, dof_limits, velocity_limits, torque_limits):
        rave.SetDOFLimits(dof_limits[0], dof_limits[1])
        rave.SetDOFVelocityLimits(velocity_limits)
        self.env = rave.GetEnv()
        self.n = rave.GetDOF()
        self.qd_max = array(velocity_limits)
        self.rave = rave
        self.torque_limits = array(torque_limits)

    def check_torque_limits(self, q, qd, qdd):
        tau = self.compute_inverse_dynamics(q, qd, qdd)
        return all(abs(tau) <= self.torque_limits)

    def compute_inverse_dynamics(self, q, qd, qdd):
        with self.rave:
            self.rave.SetDOFValues(q)
            self.rave.SetDOFVelocities(qd)
            return self.rave.ComputeInverseDynamics(qdd)

    def play_trajectory_list(self, traj_list, dt=1e-3):
        for traj in traj_list:
            for t in linspace(0, traj.duration, traj.duration / dt):
                self.rave.SetDOFValues(traj.q(t))
                time.sleep(dt)

    def sample_state(self):
        q_min, q_max = self.rave.GetDOFLimits()
        q = q_min + pylab.random(self.n) * (q_max - q_min)
        qd = (2 * pylab.random(self.n) - 1) * self.qd_max
        return hstack([q, qd])

    @property
    def state(self):
        q = self.rave.GetDOFValues()
        qd = self.rave.GetDOFVelocities()
        return hstack([q, qd])

    def state_dist(self, state0, state1):
        # assert self.n == 1 and len(state0) == len(state1) == 2
        d1 = sqrt((1. - cos(state1[0] - state0[0])) / 2.) / 2.
        d2 = abs(state1[1] - state0[1]) / (4. * self.qd_max)
        return d1 + d2
