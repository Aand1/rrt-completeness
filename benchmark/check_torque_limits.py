#!/usr/bin/env python

"""
Small script to check that all torque limits in the current directory's pickled
files are the same.
"""

import glob
import pickle

all_ok = True
tau_max = None

for fname in glob.glob('*.pkl'):
    with open(fname, 'r') as f:
        d = pickle.load(f)
        assert len(d['torque_limits']) == 1
        tau = d['torque_limits'][0]
        if tau_max is None:
            tau_max = tau
        if abs(tau - tau_max) > 1e-10:
            print "tau_max for '%s' is %.2f N.m != %.2f N.m" % (
                fname, tau, tau_max)
            all_ok = False

if all_ok:
    print "All torque limits =", tau_max, "N.m"
