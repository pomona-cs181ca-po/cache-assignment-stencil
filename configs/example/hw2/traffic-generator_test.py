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
This gem5 configuation script creates a simple board to run a traffic
generating tester CPU.

This is setup is the close to the simplest setup possible using the gem5
library. It does not contain any kind of caching, IO, or any non-essential
components (or even a real CPU!).

Usage
-----

```
scons build/X86/gem5.opt
./build/X86/gem5.opt configs/example/hw2/traffic-generator_test.py --impl=<micro,full> --l2_size=<n>KiB --l2_assoc=<n> --generator_class=<LinearGenerator,RandomGenerator> --blk_size=<n>
```
"""

import argparse

from m5.objects import MemorySize

from gem5.components.boards.test_board import TestBoard
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.linear_generator import LinearGenerator
from gem5.components.processors.random_generator import RandomGenerator
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

parser = argparse.ArgumentParser()

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

parser.add_argument(
    "--impl",
    type=str,
    default='micro',
    help="Use the micro cache or the full cache?",
    choices=['micro', 'full', ]
)

parser.add_argument(
    "--generator_class",
    type=str,
    default="RandomGenerator",
    help="The class of generator to use.",
    choices=["LinearGenerator", "RandomGenerator", ],
)

parser.add_argument(
    "--read_pct",
    type=int,
    default=80,
    help="Percentage of read requests in generated traffic.",
)

parser.add_argument(
    "--blk_size",
    type=int,
    default=64,
    help="Percentage of read requests in generated traffic.",
)

args = parser.parse_args()

def generator_factory(
    generator_class: str, rd_pct: int, mem_size: MemorySize
):
    if rd_pct > 100 or rd_pct < 0:
        raise ValueError(
            "Read percentage has to be an integer number between 0 and 100."
        )
    if generator_class == "LinearGenerator":
        return LinearGenerator(
            duration="1ms", rate="1GiB/s", max_addr=mem_size, rd_perc=rd_pct, block_size=args.blk_size,
        )
    elif generator_class == "RandomGenerator":
        return RandomGenerator(
            duration="1ms", rate="1GiB/s", max_addr=mem_size, rd_perc=rd_pct
        )
    else:
        raise ValueError(f"Unknown generator class {generator_class}")

if args.impl == 'full':
    from gem5.components.cachehierarchies.classic.private_l1_private_l2_fullcache_cache_hierarchy import PrivateL1PrivateL2FullCacheHierarchy
    
    cache_hierarchy = PrivateL1PrivateL2FullCacheHierarchy(
        l1d_size="32KiB",
        l1i_size="16KiB",
        l2_size=args.l2_size,
        l2_assoc=args.l2_assoc,
    )
else:
    from gem5.components.cachehierarchies.classic.private_l1_private_l2_microcache_cache_hierarchy import PrivateL1PrivateL2MicroCacheHierarchy

    cache_hierarchy = PrivateL1PrivateL2MicroCacheHierarchy(
        l1d_size="32KiB",
        l1i_size="16KiB",
    )

# We use a single channel DDR3_1600 memory system
memory = SingleChannelDDR3_1600(size="32MiB")

# We use a simple Timing processor with one core.
generator = generator_factory(args.generator_class, args.read_pct, memory.get_size())

# The gem5 library simple board which can be used to run simple SE-mode
# simulations.
board = TestBoard(
    clk_freq="3GHz",
    generator=generator,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Lastly we run the simulation.
simulator = Simulator(board=board)
simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)
