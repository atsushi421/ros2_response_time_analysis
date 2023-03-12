# Copyright (c) 2019 Robert Bosch GmbH
# All rights reserved.
#
# This source code is licensed under the BSD-3-Clause license found in the
# LICENSE file in the root directory of this source tree.

from models import PBurstModel
from units import mseconds, useconds, Hz
from pycpa.options import set_opt

import pycpa
import pycpa.model as model
from pycpa.junctions import ORJoin
from pycpa.analysis import NotSchedulableException


import ros
from ros import Callback, CBType, CBSReservation, DefaultScheduler
from units import seconds, Hz


def analyze_system(s):
    """Use names as keys instead of the callback objects, since the latter have equality issues"""
    r = pycpa.analysis.analyze_system(s)
    return {task.name: result for task, result in r.items()}


# The time unit is 100 microseconds. However, this is an implementation detail.
# Use these functions when calculating times instead.


set_opt("max_wcrt", seconds(1))

# Minimum worst-case jitter for input sensors.
# This models interrupt latency+ROS overhead, underestimated


class MoveBase(model.System):
    """The common parts of the move_base system model"""

    # Global dictionary of callback names -> callbacks
    callbacks = dict()
    reservations = []

    def __init__(self, num_reservations, *args, override_model={}, **kwargs):
        super().__init__(*args, **kwargs)
        for name, events in input_topics:
            if name in override_model:
                events = override_model[name]
            self.add_event_source(name, events)

        out = Callback("/cmd_vel", CBType.SUBSCRIBER, 0, wcet=0)
        res = CBSReservation(
            "Res[cmd_vel]", scheduler=DefaultScheduler(), budget=0, period=1
        )
        res.bind_task(out)
        self.bind_resource(res)
        self.register_callbacks((out,))

        for i in range(num_reservations):
            res = CBSReservation(
                "R{}".format(i),
                scheduler=DefaultScheduler(),
                budget=1,
                period=num_reservations,
            )
            self.bind_resource(res)
            self.reservations.insert(i, res)

    def add_event_source(self, name, event_model):
        """Adds an event source with the given name and event model."""
        src = Callback(name, CBType.EVENT_SOURCE, 0, wcet=0)
        src.in_event_model = event_model
        res = CBSReservation(
            "Res[{}]".format(name), scheduler=DefaultScheduler(), budget=0, period=1
        )
        res.bind_task(src)
        self.bind_resource(res)
        self.register_callbacks((src,))

    def register_callbacks(self, cbs):
        """Registers cbs, a list of callbacks, with the model.

        This allows referring to them by name in the other methods."""
        for cb in cbs:
            self.callbacks[cb.name] = cb

    def res_bind(self, resource, cbs):
        """Binds cbs (a list of callbacks) to resource.

        Callbacks might be callback objects or names (which must have
        been registered using register_callbacks before)."""
        cbs = [(self.callbacks[cb] if isinstance(cb, str) else cb) for cb in cbs]
        for cb in cbs:
            resource.bind_task(cb)

    def link(system, srcs, dsts, junctionStrategy=ORJoin):
        """Links multiple callbacks to each of multiple destinations.

        Each of srcs is linked to a junction, which is linked to each of dsts.
        junctionStrategy contains the constructor of the junction.
        srcs and dsts are sequences of callback objects or strings.
        For convenience, a single string or callback object is interpreted
        as a sequence of length 1."""

        # Wrap strings and non-sequences into a sequence of length 1
        if isinstance(srcs, str) or not hasattr(srcs, "__iter__"):
            srcs = (srcs,)
        if isinstance(dsts, str) or not hasattr(dsts, "__iter__"):
            dsts = (dsts,)

        # Look up callbacks by name, if necessary
        srcs = [system.callbacks[s] if isinstance(s, str) else s for s in srcs]
        dsts = [system.callbacks[d] if isinstance(d, str) else d for d in dsts]

        # Build a junction node to combine multiple sources, if necessary
        if len(srcs) == 1:
            junction = srcs[0]
        else:
            junction = model.Junction(
                "Junction<{}>".format(" ".join([s.name for s in srcs])),
                junctionStrategy(),
            )
            for s in srcs:
                s.link_dependent_task(junction)

        # Finally, link the destinations to the junction
        for d in dsts:
            junction.link_dependent_task(d)

    def e2e_local_chain(self):
        """Returns the end-to-end latency from /odom to /cmd_vel"""
        # Must be overridden by the child classes
        raise NotImplementedError


class EventDrivenMoveBase(MoveBase):
    """A model of move_base, where the components are triggered by ROS topics."""

    def __init__(self, num_reservations, **kwargs):
        super().__init__(
            num_reservations,
            name="move_base.driver-event.{}".format(num_reservations),
            **kwargs,
        )
        callbacks_ = [
            Callback("sensor2mem", CBType.SUBSCRIBER, 1, wcet=wcets["sensor2mem"]),
            Callback(
                "local_costmap", CBType.SUBSCRIBER, 3, wcet=wcets["local_costmap"]
            ),
            Callback(
                "global_costmap", CBType.SUBSCRIBER, 1, wcet=wcets["global_costmap"]
            ),
            Callback(
                "local_planner", CBType.SUBSCRIBER, 4, wcet=wcets["local_planner"]
            ),
            Callback(
                "global_planner_goalset",
                CBType.SUBSCRIBER,
                3,
                wcet=wcets["global_planner"],
            ),
            Callback(
                "global_planner_timed", CBType.TIMER, 2, wcet=wcets["global_planner"]
            ),
            Callback(
                "pose_estimator", CBType.SUBSCRIBER, 2, wcet=wcets["pose_estimator"]
            ),
        ]

        self.register_callbacks(callbacks_)

        self.callbacks["global_planner_timed"].in_event_model = model.PJdEventModel(
            P=Hz(1), J=0
        )

        link = self.link
        link(["/scan", "/tF"], "sensor2mem")
        link("pose_estimator", ["local_costmap", "global_costmap"])
        link("local_costmap", "local_planner")
        link("local_planner", "/cmd_vel")
        link("/odom", "pose_estimator")
        link("/goal", "global_planner_goalset")

        if num_reservations == 1:
            self.res_bind(self.reservations[0], callbacks_)
        elif num_reservations == 2:
            self.res_bind(
                self.reservations[0],
                ["local_costmap", "local_planner", "pose_estimator", "sensor2mem"],
            )
            self.res_bind(
                self.reservations[1],
                ["global_costmap", "global_planner_timed", "global_planner_goalset"],
            )
        else:
            raise ValueError("Unsupported reservation count: %s", num_reservations)

        assert all([cb.resource for cb in self.callbacks.values()])

    def e2e_local_chain(self):
        try:
            task_results = analyze_system(self)
        except NotSchedulableException as e:
            print(f"Not schedulable ({e})")
            return None
        return ros.e2e_latency(
            ros.find_path(self.callbacks["/odom"], self.callbacks["/cmd_vel"]),
            task_results,
        )


class TimeDrivenMoveBase(MoveBase):
    """A model of move_base, where all components are triggered periodically."""

    def __init__(self, num_reservations, period=Hz(12.5), **kwargs):
        super().__init__(
            num_reservations,
            name="move_base.time-driven{}".format(num_reservations),
            **kwargs,
        )

        # Prioritize the timers from source to destination. We want the source tasks to run first,
        # so the data can flow through the entire path in a single period.
        # We prioritize the local planner higher than the global planner components
        callbacks_ = [
            Callback("sensor2mem", CBType.SUBSCRIBER, 0, wcet=wcets["sensor2mem"]),
            Callback("goal2mem", CBType.SUBSCRIBER, 0, wcet=wcets["sensor2mem"]),
            Callback("local_costmap", CBType.TIMER, 4, wcet=wcets["local_costmap"]),
            Callback("global_costmap", CBType.TIMER, 1, wcet=wcets["global_costmap"]),
            Callback("local_planner", CBType.TIMER, 3, wcet=wcets["local_planner"]),
            Callback("global_planner", CBType.TIMER, 5, wcet=wcets["global_planner"]),
            Callback("pose_estimator", CBType.TIMER, 2, wcet=wcets["pose_estimator"]),
        ]

        self.register_callbacks(callbacks_)

        self.link(["/scan", "/tF", "/odom"], "sensor2mem")
        self.link("/goal", "goal2mem")
        self.link("local_planner", "/cmd_vel")
        self.callbacks["local_costmap"].in_event_model = model.PJdEventModel(
            P=period, J=0
        )
        self.callbacks["local_planner"].in_event_model = model.PJdEventModel(
            P=period, J=0
        )
        self.callbacks["pose_estimator"].in_event_model = model.PJdEventModel(
            P=period, J=0
        )
        self.callbacks["global_costmap"].in_event_model = model.PJdEventModel(
            P=period, J=0
        )
        self.callbacks["global_planner"].in_event_model = model.PJdEventModel(
            P=Hz(1), J=0
        )

        if num_reservations == 1:
            self.res_bind(self.reservations[0], callbacks_)
        elif num_reservations == 2:
            self.res_bind(
                self.reservations[0],
                ["local_costmap", "local_planner", "pose_estimator", "sensor2mem"],
            )
            self.res_bind(
                self.reservations[1], ["global_costmap", "global_planner", "goal2mem"]
            )
        else:
            raise ValueError("Unsupported reservation count: %s", num_reservations)

        assert all([cb.resource for cb in self.callbacks.values()])

    def e2e_local_chain(self):
        # In the time-driven system, the E2E delay corresponds to the WCET of
        # the local_planner task.
        # This is, because the predecessors of local_planner are prioritized
        #  higher than it and all trigger at
        # the same time.
        # However, the WCRT of the local_planner task is the E2E delay
        #  *counting from the task release*.
        # We therefore add the maximum delay between event arrival and task release
        # (one period) to get
        # the maximum E2E latency from event arrival.
        try:
            task_results = analyze_system(self)
        except NotSchedulableException:
            return None

        sampled_model = self.callbacks["/odom"].in_event_model
        sampler_period = self.callbacks["pose_estimator"].in_event_model.P
        sampling_delay = sampler_period if sampled_model.J > 0 else 0
        return sampling_delay + task_results["local_planner"].wcrt


models = {"time-driven": TimeDrivenMoveBase, "event-driven": EventDrivenMoveBase}


min_jitter = useconds(200)

wcets = {
    "local_costmap": mseconds(2),
    "global_costmap": mseconds(10),
    "local_planner": mseconds(18),
    "global_planner": mseconds(200),
    "goal_setter": min_jitter,
    "pose_estimator": useconds(200),
    "sensor2mem": min_jitter,
}

input_topics = [
    ("/scan", model.PJdEventModel(P=Hz(12.5), J=min_jitter)),
    ("/tF", model.PJdEventModel(P=Hz(12.5), J=min_jitter)),
    ("/odom", model.PJdEventModel(P=Hz(12.5), J=min_jitter)),
    ("/goal", PBurstModel(P=Hz(0.1), burstlen=0, dmin=mseconds(100))),
]
