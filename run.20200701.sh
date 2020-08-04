#!/bin/bash

DDS=fastrtps
export RMW_IMPLEMENTATION=rmw_${DDS}_cpp

# For general purpose
# FIXED_OPT="--round-ns 10000 --main-sched RR98 --child-sched RR97 --num-skip 10 --ros-args --param period_ns:=10000000 --param num_loops:=1000000 --param debug_print:=false"
FIXED_OPT="--round-ns 10000 --main-sched RR98 --child-sched RR98 --num-skip 10 --ros-args --param period_ns:=10000000 --param num_loops:=1000000 --param debug_print:=false"
# For StaticSingleThreadedExecutor
# FIXED_OPT="--static-executor --round-ns 10000 --main-sched RR98 --child-sched RR97 --num-skip 10 --ros-args --param period_ns:=10000000 --param num_loops:=1000000 --param debug_print:=false"

# no MessagePoolMemoryStrategy
# FIXED_OPT="--default-memory-strategy --round-ns 10000 --main-sched RR98 --child-sched RR97 --num-skip 10 --ros-args --param period_ns:=10000000 --param num_loops:=1000000 --param debug_print:=false"

# TS
# FIXED_OPT="--round-ns 10000 --main-sched TS --child-sched TS --num-skip 10 --ros-args --param period_ns:=10000000 --param num_loops:=1000000 --param debug_print:=false"

LOG=explog/20200730_RR98-RR98/${DDS}
mkdir -p ${LOG}

# 1e1n
LOGFILE=${LOG}/${DDS}_1e1n.log
echo "1e1n"
date > ${LOGFILE}
taskset -c 1 ./build/tw_topic/tw_ping_pong --run-type 1e1n ${FIXED_OPT} | tee -a ${LOGFILE}
date >> ${LOGFILE}
 
exit



sleep 10
 
# 1e2n
LOGFILE=${LOG}/${DDS}_1e2n.log
echo "1e2n"
date > ${LOGFILE}
taskset -c 1 ./build/tw_topic/tw_ping_pong --run-type 1e2n ${FIXED_OPT} | tee -a ${LOGFILE}
date >> ${LOGFILE}

sleep 10

# 2n1c
echo "2n1c"
LOGFILE=${LOG}/${DDS}_2e1c.log
date > ${LOGFILE}.pong
date > ${LOGFILE}.ping
taskset -c 1 ./build/tw_topic/tw_ping_pong --run-type 2e_pong ${FIXED_OPT} >> ${LOGFILE}.pong &
taskset -c 1 ./build/tw_topic/tw_ping_pong --run-type 2e_ping ${FIXED_OPT} | tee -a ${LOGFILE}.ping
kill -2 $!
date >> ${LOGFILE}.pong
date >> ${LOGFILE}.ping

sleep 10

# 2n2c
echo "2n2c"
LOGFILE=${LOG}/${DDS}_2e2c.log
date > ${LOGFILE}.pong
date > ${LOGFILE}.ping
taskset -c 1 ./build/tw_topic/tw_ping_pong --run-type 2e_pong ${FIXED_OPT} >> ${LOGFILE}.pong &
taskset -c 2 ./build/tw_topic/tw_ping_pong --run-type 2e_ping ${FIXED_OPT} | tee -a ${LOGFILE}.ping
kill -2 $!
date >> ${LOGFILE}.pong
date >> ${LOGFILE}.ping
