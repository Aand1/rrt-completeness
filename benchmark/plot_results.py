#!/usr/bin/env python

import bisect
import IPython
import glob
import pickle
import pylab

logscale = True

pylab.ion()
pylab.clf()


class Curve(object):

    def __init__(self, xvals, yvals):
        assert len(xvals) == len(yvals)
        self.xvals = xvals
        self.yvals = yvals

    def value(self, x):
        i = bisect.bisect(self.xvals, x)
        if i >= len(self.xvals):
            return self.yvals[-1]
        return self.yvals[i - 1]

    def plot(self, color, alpha=1., lw=1):
        xvals, yvals = [], []
        for i, x in enumerate(self.xvals):
            if i > 0:
                xvals.append(x)
                yvals.append(self.yvals[i - 1])
            xvals.append(x)
            yvals.append(self.yvals[i])
        pylab.plot(xvals, yvals, color, alpha=alpha, lw=lw)


curves = {'Bezier': [], 'SOC': []}
avg_xvals = {'Bezier': [], 'SOC': []}

jj = 0
for fname in glob.glob('*.pkl'):
    jj += 1
    # if jj > 2: break
    with open(fname, 'r') as f:
        run = pickle.load(f)
    for label, plot_data in run['plots'].iteritems():
        xvals, yvals = zip(*plot_data)
        if logscale:
            xvals = list(xvals)
            xvals[0] = 1   # 0 does not appear on log scale
        curve = Curve(xvals, yvals)
        curves[label].append(curve)
        avg_xvals[label].extend(xvals)
        color = 'g-' if 'Bezier' in label else 'r-'
        curve.plot(color, alpha=0.75)
        if xvals[-1] < 100000:
            ymax = 0.999 * pylab.ylim()[1]
            c = 'g' if 'Bezier' in label else 'r'
            pylab.plot([xvals[-1], xvals[-1]], [0., ymax], '%c-' % c, lw=2)
            pylab.plot([xvals[-1]], [0.], '%co' % c, ms=10)
        if 'SOC' in label:
            print fname, xvals[-1], float(yvals[-1])


def combine_curves(xvals, curves, fun):
    yvals = []
    # xvals = sorted(xvals)  # should be done beforehand
    for x in xvals:
        yvals.append(fun([curve.value(x) for curve in curves]))
    return Curve(xvals, yvals)


def average_curve(xvals, curves):
    n = len(curves)
    return combine_curves(xvals, curves, lambda l: sum(l) / n)


def max_curve(xvals, curves):
    return combine_curves(xvals, curves, lambda l: max(l))


def min_curve(xvals, curves):
    return combine_curves(xvals, curves, lambda l: min(l))


bezier_xvals = sorted(avg_xvals['Bezier'])
soc_xvals = sorted(avg_xvals['SOC'])
avg_bezier = average_curve(bezier_xvals, curves['Bezier'])
avg_soc = average_curve(soc_xvals, curves['SOC'])
min_bezier = min_curve(bezier_xvals, curves['Bezier'])
min_soc = min_curve(soc_xvals, curves['SOC'])
max_bezier = max_curve(bezier_xvals, curves['Bezier'])
max_soc = max_curve(soc_xvals, curves['SOC'])

pylab.fill_between(
    soc_xvals,
    [float(y1) for y1 in min_soc.yvals],
    [float(y2) for y2 in max_soc.yvals], color='r', alpha=0.3)

pylab.fill_between(
    bezier_xvals,
    [float(y1) for y1 in min_bezier.yvals],
    [float(y2) for y2 in max_bezier.yvals], color='g', alpha=0.3)

avg_bezier.plot('g-', lw=8)
avg_soc.plot('r-', lw=8)
# min_bezier.plot('g-', lw=2)
# min_soc.plot('r-', lw=2)
# max_bezier.plot('g-', lw=2)
# max_soc.plot('r-', lw=2)

pylab.xlabel("Number of extensions")
pylab.ylabel("Distance to goal")
pylab.grid(True)
if logscale:
    pylab.xscale('log')

IPython.embed()
