"""
"""

import gc
import os
import pprint
import psutil
import sys
import weakref

def get_working_set_mib():
    return psutil.Process(os.getpid()).memory_info().rss / 1024 ** 2


from pydrake.all import AddMultibodyPlantSceneGraph, DiagramBuilder, Parser
from pydrake.common import FindResourceOrThrow

URDF = FindResourceOrThrow("drake/examples/acrobot/Acrobot.urdf")

def iterate(iters):
    for _ in range(iters):
        def do_iter():
            builder = DiagramBuilder()
            plant, scene_graph = AddMultibodyPlantSceneGraph(builder,
                                                             time_step=0.001)
            plant.Finalize()
            diagram = builder.Build()
            return [weakref.finalize(v, lambda: None)
                    for k, v in locals().items()]
        spies = do_iter()
        gc.collect()
        # assert all(not x.alive for x in spies)
        # print("garbage count:", len(gc.garbage))
        # print("garbage:", pprint.pformat(gc.garbage))
        print('.', end='', file=sys.stderr, flush=True)

        if get_working_set_mib() > 220:
            raise RuntimeError("Working set threshold exceeded.")

def main():
    iters = 1000
    try:
        iters = int(sys.argv[1])
    except Exception:
        pass
    print("runs:", iters, file=sys.stderr)
    iterate(iters)
    print("\ndone.\n", iters, file=sys.stderr)

if __name__ == "__main__":
    main()
