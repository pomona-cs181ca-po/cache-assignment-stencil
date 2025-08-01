# Copyright (c) 2021 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: ST, Pomona College (2025)

"""
This gem5 configuation script creates a simple board to run an X86
random access binary.

This is setup is the close to the simplest setup possible using the gem5
library. It does not contain any kind of caching, IO, or any non-essential
components.

Usage
-----

```
scons build/X86/gem5.opt
./build/X86/gem5.opt configs/example/hw2/memtest_fullcache.py --l2_size=<n>KiB --l2_assoc=<n>
```
"""

import argparse

from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.private_l1_private_l2_fullcache_cache_hierarchy import PrivateL1PrivateL2FullCacheHierarchy
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import BinaryResource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

# This check ensures the gem5 binary is compiled to the ARM ISA target. If not,
# an exception will be thrown.
requires(isa_required=ISA.X86)

parser = argparse.ArgumentParser(description="Configuration script for memtest.")

parser.add_argument(
    "--l2_size",
    type=str,
    default="128KiB",
    help="L2 cache size.",
)

parser.add_argument(
    "--l2_assoc",
    type=int,
    default=2,
    help="L2 cache associativity.",
)

args = parser.parse_args()

# In this setup we don't have a cache. `NoCache` can be used for such setups.
cache_hierarchy = PrivateL1PrivateL2FullCacheHierarchy(
    l1d_size="32KiB",
    l1i_size="16KiB",
    l2_size=args.l2_size,
    l2_assoc=args.l2_assoc,
)

# We use a single channel DDR3_1600 memory system
memory = SingleChannelDDR3_1600(size="32MiB")

# We use a simple Timing processor with one core.
processor = SimpleProcessor(cpu_type=CPUTypes.TIMING, isa=ISA.X86, num_cores=1)

# The gem5 library simple board which can be used to run simple SE-mode
# simulations.
board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Here we set the workload. In this case we want to run a simple "Hello World!"
# program compiled to the ARM ISA. The `Resource` class will automatically
# download the binary from the gem5 Resources cloud bucket if it's not already
# present.
board.set_se_binary_workload(
    binary=BinaryResource(local_path="./hw2-tests/bin/memtest")
)

# Lastly we run the simulation.
simulator = Simulator(board=board)
simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)
