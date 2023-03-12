from typing import List
from .executor import Executor


class CPU:
    def __init__(self, id: int) -> None:
        self.id: int = id
        self.U: float = 0
        self.executors: List[Executor] = []

    # For analysis use
    def assign_executor(self, exe: Executor) -> None:
        exe.cpu = self.id
        for cb in exe.callbacks:
            cb.cpu_id = self.id

        self.executors.append(exe)
        self.U += exe.U
