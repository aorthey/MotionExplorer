#!/usr/bin/env python3
import glob
from PlotTable import *

from ParseBenchmarkFile import *
from collections import defaultdict
import subprocess
import glob
import re

def PlotTable(fnames):

    # fnames = sorted(fnames)

    d = defaultdict(dict)
    d_status = defaultdict(dict)

    planner_names_tmp = []
    env_names_tmp = []

    Nruns = 0
    for f in fnames:
        print(f)
        benchmark = BenchmarkAnalytica(f)
        env = benchmark.benchmark_name

        for p in benchmark.planners:
            pname = p.name
            V = p.GetNodesPerRun()
            print(V)
            print(p.run_number_of_modes)
            vn = np.mean(V)
            print(f,"time = ", p.AverageTime(), " stddev=", p.TimeVariance(), " modes=",p.AverageModes(), " avg nodes=", vn)
            print(f,"$t = ", np.round(p.AverageTime(),2), "\\pm",
            np.round(p.TimeVariance(),2), "$, $m=",p.AverageModes(), "$, $n=", vn,"$")


if __name__ == '__main__':
    fnames = []
    fnames += glob.glob("../../data/benchmarks/21-RSS/May30th/*.xml")
    print(fnames)

    PlotTable(fnames)
