import re
import numpy as np

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
	if re.match("{}$".format(target), line):
	    print("{} found at {}".format(target, i))
	    line_target = i

    if line_target < 0:
	raise Exception("target({0}) not found".format(target))

    round_line = lines[line_target + 3]
    match = re.search("round_ns = (\d+).*min = (.*\d+)", round_line)
    if not match:
	raise Exception("cannot read round_ns")
    round_ns = match.group(1)
    hist_min = int(match.group(2))

    result = lines[line_target + 4].lstrip()
    result = result.split(",")
    result = [int(x) for x in result if x]
    ys = np.require(range(len(result))) + hist_min / int(round_ns)

    return round_ns, result, ys

