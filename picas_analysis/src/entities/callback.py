from typing import Optional


class Callback:
    def __init__(
        self,
        id: int,
        period: int,
        execution_time: float,
        chain_id: int,
    ) -> None:
        self.id: int = id
        self.cpu_id: int = 0
        self.executor_id: int = 0

        self.type: str = "regular" if period == 0 else "timer"
        self.T: int = period
        self.C: float = execution_time
        self.priority: int = 0

        self.chain_id: int = chain_id
        self.chain_T: int
        self.chain_C: float = 0

        # For analysis purpose
        self.wcrt: int = 0
        self.segment_flag: bool = False
        self.segment_C: float = 0
        self.chain_on_cpu: bool = False
