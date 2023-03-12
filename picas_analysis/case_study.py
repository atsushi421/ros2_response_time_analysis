# Case study (Section VII-B)

# Period(only for timer callback), Execution time, Deadline(only for timer callback), Chain, Order in chain
# Case study III (overloeaded scenario)

from typing import List
import pandas as pd

from src import Callback, Executor, Chain, CPU, ResponseTime

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--input")

args = parser.parse_args()

import yaml

with open(args.input, "r") as f:
    input_data = yaml.safe_load(f)

NUM_EXECUTOERS = input_data["num_executors"]
NUM_CPUS = input_data["num_cpus"]

# Initialize chains
num_chains = list(input_data["callbacks"].values())[-1]["chain_id"] + 1
sem_priority = num_chains
chains: List[Chain] = []
for chain_id in range(num_chains):
    chains.append(Chain(chain_id, sem_priority))
    sem_priority -= 1

# Initialize callbacks
callbacks: List[Callback] = []
for cb_name, cb_dict in input_data["callbacks"].items():
    cb = Callback(
        int(cb_name.replace("cb", "")),
        int(cb_dict["period"]),
        float(cb_dict["exec"]),
        int(cb_dict["chain_id"]),
    )
    callbacks.append(cb)
    chains[cb.chain_id].add_callback(cb)

# Initialize executors
priority = NUM_EXECUTOERS
executors: List[Executor] = []
for exe_id in range(NUM_EXECUTOERS):
    executors.append(Executor(exe_id, priority))
    priority -= 1

# Initialize cpus
cpus: List[CPU] = [CPU(cpu_id) for cpu_id in range(NUM_CPUS)]

# Assign callback priority
callback_priority = len(input_data["callbacks"])
for chain in chains:
    for r_cb in reversed(chain.regular_cbs):
        r_cb.priority = callback_priority
        callback_priority -= 1
        r_cb.chain_T = chain.T

    if not chain.timer_cb:
        print(chain.id)
        raise NotImplementedError("BUG")
    chain.timer_cb.priority = callback_priority
    callback_priority -= 1
    chain.timer_cb.chain_T = chain.T

callbacks[0].priority = callbacks[
    2
].priority  # Since callback(1) and callback(3) are the same, i.e., a mutual callback

# If all callbacks of a chain exist on the same CPU core, set "chain_on_cpu" True
same_cpu_chain_cb_id = [0, 1, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18]
for i in same_cpu_chain_cb_id:
    callbacks[i].chain_on_cpu = True

# Allocate callbacks to executors manually
# RT chains
executors[0].add_callbacks(callbacks[0:3])
executors[1].add_callbacks(callbacks[3:6])
executors[2].add_callbacks(callbacks[6:10])
executors[3].add_callbacks(callbacks[10:13])
executors[4].add_callbacks(callbacks[13:17])
executors[5].add_callbacks(callbacks[17:19])

# BE chains
executors[6].add_callbacks([callbacks[19]])
executors[7].add_callbacks([callbacks[20]])
executors[8].add_callbacks([callbacks[21]])
executors[9].add_callbacks([callbacks[22]])
executors[10].add_callbacks([callbacks[23]])
executors[11].add_callbacks([callbacks[24]])
executors[12].add_callbacks([callbacks[25]])
executors[13].add_callbacks([callbacks[26]])
executors[14].add_callbacks([callbacks[27]])
executors[15].add_callbacks([callbacks[28]])
executors[16].add_callbacks([callbacks[29]])
executors[17].add_callbacks([callbacks[30]])

# Allocate executors to CPUs
cpus[0].assign_executor(executors[0])
cpus[0].assign_executor(executors[4])
cpus[0].assign_executor(executors[10])
cpus[0].assign_executor(executors[13])
cpus[0].assign_executor(executors[14])

cpus[1].assign_executor(executors[1])
cpus[1].assign_executor(executors[7])
cpus[1].assign_executor(executors[8])
cpus[1].assign_executor(executors[15])
cpus[1].assign_executor(executors[16])

cpus[2].assign_executor(executors[2])
cpus[2].assign_executor(executors[6])
cpus[2].assign_executor(executors[11])
cpus[2].assign_executor(executors[17])

cpus[3].assign_executor(executors[3])
cpus[3].assign_executor(executors[5])
cpus[3].assign_executor(executors[12])
cpus[3].assign_executor(executors[9])

# Compute response time of callbacks
response_time = ResponseTime(chains, cpus)
latency = response_time.response_time_callbacks()

# Output
for chain, l in zip(chains, latency):
    cbs = [f"t{chain.timer_cb.id}"] if chain.timer_cb else []
    for rcb in chain.regular_cbs:
        cbs.append(f"t{rcb.id}")
    print(f"|Chain {chain.id} = {cbs}| End-to-end latency: {l}")
