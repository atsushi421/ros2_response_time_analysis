from typing import List, Optional
from .callback import Callback


class Chain:
    def __init__(self, id: int, sem_priority: int) -> None:
        self.id: int = id
        self.sem_priority: int = sem_priority

        self.timer_cb: Optional[Callback] = None
        self.regular_cbs: List[Callback] = []

        self.C: int = 0
        self.T: int = 0

    @property
    def num_callbacks(self) -> int:
        return 1 if self.timer_cb else 0 + len(self.regular_cbs)

    def add_callback(self, callback: Callback) -> None:
        if callback.type == "timer":
            self.timer_cb = callback
            self.T = callback.T
        else:
            self.regular_cbs.append(callback)

        # Update chain_C
        self.C += callback.C
        if self.timer_cb:
            self.timer_cb.chain_C = self.C

        for cb in self.regular_cbs:
            cb.chain_C = self.C
