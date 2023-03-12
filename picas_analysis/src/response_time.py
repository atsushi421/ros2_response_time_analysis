from typing import List, Tuple
from .entities import Chain, CPU, Callback, Executor
import sys
import math


class ResponseTime:
    def __init__(self, chains: List[Chain], cpus: List[CPU]) -> None:
        self._chains = chains
        self._cpus = cpus

    def response_time_callbacks(self) -> List[int]:
        callbacks: List[Callback] = []
        executors: List[Executor] = []

        # reshape callbacks and distinguish segment tasks for each chain/CPU
        cb_id = 0
        for cpu in self._cpus:
            chain_seg_p = [sys.maxsize] * len(self._chains)
            chain_seg_lowest_p_cb_id = [0] * len(self._chains)
            chain_seg_C = [0] * len(self._chains)
            for exe in cpu.executors:
                for cb in exe.callbacks:
                    cur_chain_id = cb.chain_id
                    if cb.priority < chain_seg_p[cur_chain_id]:
                        # Update segment priority
                        chain_seg_p[cur_chain_id] = cb.priority
                        chain_seg_lowest_p_cb_id[cur_chain_id] = cb_id
                    chain_seg_C[cur_chain_id] += cb.C
                    cb_id += 1
                    callbacks.append(cb)
                executors.append(exe)

            # set segment callbacks
            for s in range(len(self._chains)):
                if chain_seg_C[s] != 0:
                    callbacks[chain_seg_lowest_p_cb_id[s]].segment_flag = True
                    callbacks[chain_seg_lowest_p_cb_id[s]].segment_C = chain_seg_C[s]

        executors.sort(key=lambda x: x.id)
        callbacks.sort(key=lambda x: x.id)

        # compute the WCRT of individual callbacks
        for target_cb in callbacks:
            # blocking time by lower priority tasks within executor
            B = 0
            for cb in executors[target_cb.executor_id].callbacks:
                if (
                    cb.chain_id != target_cb.chain_id
                    and cb.priority < target_cb.priority
                    and cb.C > B
                ):
                    B = cb.C

            # initial R
            if target_cb.segment_flag:
                R = target_cb.segment_C + B
            else:
                R = target_cb.C + B

            R_prev = R
            while True:
                W = 0
                for cb in callbacks:

                    # only consider callback on higher- or same priority executor
                    if (
                        cb.id != target_cb.id
                        and executors[target_cb.executor_id].priority
                        <= executors[cb.executor_id].priority
                    ):
                        # check current chain is on a single cpu
                        if (
                            (
                                target_cb.chain_on_cpu
                                and cb.chain_id != target_cb.chain_id
                            )
                            and (cb.cpu_id == target_cb.cpu_id)
                        ) or (
                            not target_cb.chain_on_cpu and cb.cpu_id == target_cb.cpu_id
                        ):
                            timer_priority, timer_T, timer_cpu_id = self.find_timer_cb(
                                executors, cb.chain_id
                            )
                            if cb.chain_on_cpu:
                                P = max(cb.chain_C, timer_T)
                            else:
                                P = timer_T

                            if (
                                executors[target_cb.executor_id].priority
                                < executors[cb.executor_id].priority
                            ) or (target_cb.priority < cb.priority):
                                if (
                                    timer_priority >= target_cb.priority
                                    or timer_cpu_id != target_cb.cpu_id
                                ):
                                    W += math.ceil(R / P) * cb.C
                                else:
                                    W += cb.C

                if target_cb.segment_flag:
                    R = W + target_cb.segment_C + B
                else:
                    R = W + target_cb.C + B

                if R <= R_prev:
                    target_cb.wcrt = R
                    break

                R_prev = R

        # Theorem 1
        # capture WCRT with considering time delay by prior chain instance
        chain_latency: List[int] = [0] * len(self._chains)
        for i in range(len(self._chains)):
            chain = self._chains[i]
            if chain.timer_cb and chain.timer_cb.segment_flag:
                chain_latency[i] += chain.timer_cb.wcrt
            for rcb in chain.regular_cbs:
                if rcb.segment_flag:
                    chain_latency[i] += rcb.wcrt

            if chain.timer_cb and chain_latency[i] > chain.timer_cb.T:
                chain_latency[i] += chain.timer_cb.T

        return chain_latency

    @staticmethod
    def find_timer_cb(executors: List[Executor], chain_id: int) -> Tuple[int, int, int]:
        for exe in executors:
            for cb in exe.callbacks:
                if cb.chain_id == chain_id and cb.type == "timer":
                    return cb.priority, cb.T, cb.cpu_id

        raise NotImplementedError("Bug")
