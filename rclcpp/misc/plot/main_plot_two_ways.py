#!/usr/bin/python
# Plot create 2 image files
#   1e: 1e1n vs 1e2n
#   2e: 2e1c vs 2e2c
# Each image containes
#   diff_wake
#   ping_wakeup
#   ping_sub
#   pong_sub
#   ping_pong
#   timer_callback
#   ping_callback
#   pong_callback

import os
import argparse
import glob
import re
import numpy as np
import matplotlib.pyplot as plt

def get_round_ns_and_results(fname, target):
    '''
    fname: target log filename
    target: result type such as wake-diff ping-pong
    '''
    with open(fname) as fp:
	lines = fp.readlines()
    lines = map(lambda l: l.rstrip(), lines)

    # find target
    line_target = -1
    for i, line in enumerate(lines):
	if target in line:
	    print("{} found at {}".format(target, i))
	    line_target = i

    if line_target < 0:
	raise Exception("target({0}) not found".format(target))

    round_line = lines[line_target + 3]
    match = re.search("round_ns = (\d+) ", round_line)
    if not match:
	raise Exception("cannot read round_ns")
    round_ns = match.group(1)

    result = lines[line_target + 4].lstrip()
    result = result.split(",")
    result = [int(x) for x in result if x]
    return round_ns, result

def parse_arguments():
    parser = argparse.ArgumentParser(description='plot tw_tpoic result')
    ## normal
    parser.add_argument("result_dir",
                        help="filename should must contains 1e1n, 1e2n, 2e1c*p{i,o}ng, 2e2c*p{i,}ong")
    parser.add_argument("graph_title")
    return parser.parse_args()

def plot_target_1e(files, target, pos):
    '''
    files: dictinary such as {"1e1n": path_to_file, ...}
    target: metrics name such as diff_wake, ping_wake, ...
    pos: graph position
    label: data label such as 1e1n
    '''
    round_ns, data_1e1n = get_round_ns_and_results(files["1e1n"], target)
    _, data_1e2n = get_round_ns_and_results(files["1e2n"], target)

    plt.subplot(pos)
    plt.title(target)
    plt.plot(data_1e1n, label="1e1n({})".format(np.sum(data_1e1n)))
    plt.plot(data_1e2n, label="1e2n({})".format(np.sum(data_1e2n)))
    plt.yscale("log")
    plt.ylabel("frequency(log)")
    plt.xlabel("{} ns".format(round_ns))
    plt.legend()

def plot_target_2e(files, target, pos):
    plt.subplot(pos)

    round_ns = 0
    if target in ["diff_wake", "ping_wakeup", "ping_pong", "pong_sub", "timer_callback", "pong_callback"]:
        round_ns, data_2e1c = get_round_ns_and_results(files["2e1c_ping"], target)
        _, data_2e2c = get_round_ns_and_results(files["2e2c_ping"], target)
    elif target in ["ping_sub", "ping_callback"]:
        round_ns, data_2e1c = get_round_ns_and_results(files["2e1c_pong"], target)
        _, data_2e2c = get_round_ns_and_results(files["2e2c_pong"], target)

    plt.title(target)
    plt.plot(data_2e1c, label="2e1c({})".format(np.sum(data_2e1c)))
    plt.plot(data_2e2c, label="2e2c({})".format(np.sum(data_2e2c)))

    plt.yscale("log")
    plt.ylabel("frequency(log)")
    plt.xlabel("{} ns".format(round_ns))
    plt.legend()

def main(args):
    tgtdir = args.result_dir
    title = args.graph_title

    fname = "tw_{0}.log"

    # glob files
    files_tmp = glob.glob(os.path.join(tgtdir, "*"))
    files = {}
    for f in files_tmp:
        if re.search('1e1n', f):
            files["1e1n"] = f
        elif re.search('1e2n', f):
            files["1e2n"] = f
        elif re.search('2e1c.*ping', f):
            files["2e1c_ping"] = f
        elif re.search('2e1c.*pong', f):
            files["2e1c_pong"] = f
        elif re.search('2e2c.*ping', f):
            files["2e2c_ping"] = f
        elif re.search('2e2c.*pong', f):
            files["2e2c_pong"] = f
    print files


    # plot 1e
    plt.figure(figsize=(16, 18))
    plt.suptitle("1 executor. 1-2 nodes. {}".format(title))

    plot_target_1e(files, "diff_wake", 331)
    plot_target_1e(files, "ping_wakeup", 332)
    plot_target_1e(files, "ping_sub", 334)
    plot_target_1e(files, "pong_sub", 335)
    plot_target_1e(files, "ping_pong", 336)
    plot_target_1e(files, "timer_callback", 337)
    plot_target_1e(files, "ping_callback", 338)
    plot_target_1e(files, "pong_callback", 339)

    plt.savefig(os.path.join(tgtdir, "{}_1executor.png".format(title)))
    # plt.show()
    plt.clf()

    # plot 2e
    plt.figure(figsize=(16, 18))
    plt.suptitle("2 executor. 1-2 cores. {}".format(title))
    plot_target_2e(files, "diff_wake", 331)
    plot_target_2e(files, "ping_wakeup", 332)
    plot_target_2e(files, "ping_sub", 334)
    plot_target_2e(files, "pong_sub", 335)
    plot_target_2e(files, "ping_pong", 336)
    plot_target_2e(files, "timer_callback", 337)
    plot_target_2e(files, "ping_callback", 338)
    plot_target_2e(files, "pong_callback", 339)

    plt.savefig(os.path.join(tgtdir, "{}_2executor.png".format(title)))
    # plt.show()
    plt.clf()


if __name__ == "__main__":
    args = parse_arguments()
    main(args)
