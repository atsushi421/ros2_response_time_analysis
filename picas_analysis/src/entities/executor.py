from typing import List
from .callback import Callback


class Executor:
    def __init__(self, id: int, priority: int) -> None:
        self.id: int = id
        self.priority: int = priority

        self.callbacks: List[Callback] = []
        self.cpu: int = 0
        self.U: float = 0

    def add_callbacks(self, callbacks: List[Callback]) -> None:
        for cb in callbacks:
            self.U += cb.C / cb.chain_T
            cb.executor_id = self.id
            self.callbacks.append(cb)
