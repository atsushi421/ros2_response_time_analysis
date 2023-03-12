from functools import lru_cache

import pycpa.model as model
import ros
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
from fractions import Fraction
import math
import logging

from units import mseconds, Hz
from move_base import models, EventDrivenMoveBase


def isInteger(x):
    """Returns whether x is an integral number.

    This is independent of the type of x. 2.0 is considered an integral number"""
    return int(x) == x


def minimalP(alpha, min_budget=mseconds(1)):
    """Returns the smallest reservation period that allows a budget of alpha with min_budget
     granularity.

    Effectively, this computes the smallest P such that alpha*P is divisible by min_budget."""

    if alpha == 0:
        return min_budget

    alpha = Fraction(alpha).limit_denominator(100)

    # Multiply both sides by ceil(budget/numerator) to get a numerator
    # that is larger then min_budget.

    factor = min_budget / alpha.numerator
    if not isInteger(factor):
        factor = math.ceil(factor)
        assert isInteger(factor)
    return alpha.denominator * int(factor)


def make2res_alpha(alpha, model, P):
    """Creates a CPA model with two reservations, with relative budget alpha.

    model is a function that, given a reservation count, returns a model with that many
    reservations.
    The function then configures the budget such that reservation 0 receives alpha of the
    total budget,
    and reservation 1 receives 75% of budget.
    P can be the reservation period length or None, which selects the minimum
    feasible period length."""
    s = model(2)

    if P is None:
        P = minimalP(alpha)
        logging.info('Computed P for alpha={} as {}'.format(alpha, P))

    alpha = Fraction(alpha).limit_denominator(P)
    assert isInteger(alpha * P)
    s.reservations[0].period = P
    s.reservations[0].budget = int(alpha * P)
    s.reservations[1].period = 4
    s.reservations[1].budget = 3

    # Sanity check
    for r in s.reservations:
        assert isInteger(r.period) and isInteger(r.budget)
        assert 0 <= r.budget <= r.period

    return s


@lru_cache(maxsize=None)
def e2e_per_alpha_local(alpha, model, P=None, disable_chain=False):
    """Sets up a 2-reservation model with the given budget distribution, and computes
     the e2e latency of the /odom -> /cmd_vel chain"""
    # The chain analysis decision is passed in from the outside as a parameter,
    # and the global variable modified here. This allows memoization of the function,
    # which cannot account for global variables otherwise
    with ros.chainAnalysis(disable_chain):
        return make2res_alpha(alpha, model, P).e2e_local_chain()


def convertTimes(times, unit_tick):
    """Takes a list of timestamps, and normalizes them to the given unit.

    For convenience, None is normalized to None. This is important to allow undefined
     points in the list."""
    return [None if x is None else x / unit_tick
            for x in times]


# The pyplot plotting style for the different models.
model_graphspec = {'time-driven': 'bo-',
                   'event-driven': 'gD-'}


def plot_e2e_per_alpha(P=None, **kwargs):
    """Plots the reservation budget assignment vs. the /odom->/cmd_vel end-to-end latencies.

    P is the reservation period, any other kwargs are passed to plt.plot"""

    # Step through the x axis in steps of 5.
    xs_pct = np.linspace(0, 100, num=21)
    plt.xlabel('Budget of the local reservation (percent)')
    plt.ylabel('End-to-end latency (ms)')

    for mname, m in models.items():
        if P is not None:
            mname += '(P={})'.format(P / mseconds(1))

        latencies = [e2e_per_alpha_local(x / 100, P=P, model=m, disable_chain=False)
                     for x in xs_pct]
        latencies_nochain = [e2e_per_alpha_local(x / 100, P=P, model=m, disable_chain=True)
                             for x in xs_pct]
        plt.plot(xs_pct, convertTimes(latencies, mseconds(1)),
                 model_graphspec[mname], label=mname, **kwargs)
        if mname == 'event-driven':
            plt.plot(xs_pct, convertTimes(latencies_nochain, mseconds(1)),
                     model_graphspec[mname], linestyle=':', label=mname + ' (no chains)', **kwargs)
    plt.ylim(bottom=0)
    plt.legend(loc='upper left')

# We need to memoize this function in addition to e2e_per_alpha_local, since the memoization
# doesn't seem to be able to handle ad-hoc computed models.


@lru_cache(maxsize=None)
def jittered_lats(movebase, js, disable_chain=False):
    """Computes e2e_per_alpha_local at reservation alpha of 45% for each jitter in js."""

    def jittered_models(J):
        return {'/odom': model.PJdEventModel(P=Hz(12.5), J=J)}

    return [e2e_per_alpha_local(0.45,
                                lambda n: movebase(n, override_model=jittered_models(J)),
                                disable_chain=disable_chain)
            for J in js]


def plot_e2e_per_jitter(add_disabled_chain=False):
    """Plots jitter values vs. jittered_lats."""
    js = range(0, mseconds(200), mseconds(15))

    plt.xlabel('Jitter on the input sensors (ms)')
    plt.ylabel('End-to-End latency (ms)')
    for mname, m in models.items():
        plt.plot(convertTimes(js, mseconds(1)),
                 convertTimes(jittered_lats(m, js), mseconds(1)),
                 model_graphspec[mname], label=mname)

    if add_disabled_chain:
        plt.plot(
            convertTimes(js, mseconds(1)),
            convertTimes(
                jittered_lats(EventDrivenMoveBase, js, disable_chain=True),
                mseconds(1)),
            model_graphspec[mname],
            linestyle=':', label='event-driven (no chains)')

    plt.ylim(bottom=0)
    plt.legend()


def configure_mpl_for_tex():
    "Configures matplotlib for LaTeX embedding"
    # Inspired by https://jwalton.info/Embed-Publication-Matplotlib-Latex/

    # Width of the LIPICS a4paper column, determined by \the\columnwidth
    width_pts = 398.33858
    golden_ratio = (5**.5 - 1) / 2
    # Width of the figure relative to the page
    rel_width = .48

    inches_per_pt = 1 / 72.27

    # Figure width in inches
    width_in = width_pts * inches_per_pt * rel_width
    # Figure height in inches
    height_in = width_in * golden_ratio

    settings = {
        "figure.figsize": (width_in, height_in),
        "text.usetex": True,
        "font.family": "serif",
        "font.size": 6,
        "font.serif": ["computer modern roman"],
        "axes.labelsize": 6,
        "legend.fontsize": 6,
        "xtick.labelsize": 6,
        "ytick.labelsize": 6,
        "lines.markersize": 3.5
    }
    mpl.rcParams.update(settings)


def generate_charts_for_paper(dir='.'):
    """Generate the charts for the ECRTS paper."""
    plt.close()
    configure_mpl_for_tex()
    plot_e2e_per_alpha()
    lat_per_budget_path = dir + '/latency_per_budget.pdf'
    plt.savefig(lat_per_budget_path, bbox_inches='tight')

    plt.cla()
    plot_e2e_per_jitter()
    lat_per_jitter_path = dir + '/latency_per_jitter.pdf'
    plt.savefig(lat_per_jitter_path, bbox_inches='tight')

    print('Generated charts from the ECRTS paper in',
          lat_per_budget_path, 'and', lat_per_jitter_path)
