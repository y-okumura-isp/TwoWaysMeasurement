#!/usr/bin/python

import os
import argparse
import glob
import matplotlib.pyplot as plt
import numpy as np

from common import get_round_ns_and_results

def parse_arguments():
    parser = argparse.ArgumentParser(description='plot 1e1n comparison graphs')
    ## normal
    parser.add_argument("result_dir",
                        help="filename should must contains 1e1n")
    parser.add_argument("graph_title")
    return parser.parse_args()

def plot_target_1e(files, target, pos):
    '''
    files: target files
    target: metrics name such as diff_wakeup, ping_wake, ...
    pos: graph position
    label: data label such as 1e1n
    '''
    for fpath in files:
        round_ns, data_1e1n, ys= get_round_ns_and_results(fpath, target)

        label_name = os.path.basename(fpath)
        plt.subplot(pos)
        plt.title(target)
        plt.plot(ys, data_1e1n, label=label_name)
        plt.yscale("log")
        plt.ylabel("frequency(log)")
        plt.xlabel("{} ns".format(round_ns))
        plt.legend()

def main(args):
    tgtdir = args.result_dir
    title = args.graph_title

    fname = "*.log"

    # glob files
    files = glob.glob(os.path.join(tgtdir, "*.log"))
    files.sort()

    plt.figure(figsize=(16, 18))
    plt.suptitle("1 executor. 1-2 nodes. {}".format(title))

    plot_target_1e(files, "diff_wakeup", 331)
    plot_target_1e(files, "ping_wakeup", 332)
    plot_target_1e(files, "ping_sub", 334)
    plot_target_1e(files, "pong_sub", 335)
    plot_target_1e(files, "ping_pong", 336)
    plot_target_1e(files, "timer_callback", 337)
    plot_target_1e(files, "ping_callback", 338)
    plot_target_1e(files, "pong_callback", 339)

    plt.savefig(os.path.join(tgtdir, "{}.png".format(title)))
    plt.show()
    plt.clf()

if __name__ == "__main__":
    args = parse_arguments()
    main(args)

