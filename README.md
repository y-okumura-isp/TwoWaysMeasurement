Two Ways Test
====

# About
- Check performance by following scenarios:
  - ping (publisher only)
  - ping (puslish -> subscribe)
  - ping-pong
- ping-pong is implemented by 2 topics with following publisher and subscriber.
  - ping-publisher(ping-pub) sends ping periodically using ping-topic.
  - ping-subscriber(ping-sub) and pong-publisher(pong-pub)
    ping-sub subscribes ping-topic and optionally pong-pub sends pong by pong-topic.
  - pong-subscriber(pong-sub) subscribes pong-topic.
- There are some variations for the number of Executors and Nodes, e.g.
  - 1 Executor 1 Node
  - 1 Executor 2 Node
    - Node1 contains ping-pub + pong_sub
	- Node2 contains ping-sub + pong-pub
  - seperate above 2 Node into 2 Executors

# Build

```
# This has many sub packages
colcon list

# build what you want
cd ..
colcon build --symlink-install --packages-select tw_rclcpp twmsgs
```

# Run

```
./build/tw_rclcpp_1exec_topic/tw_1exec_1node \
    --ros-args --param num_loops:=1000 -param period_ns:=1000000
```


# Common Output
- Processes outputs following log to stdout as following.
  They output "recent values" and "average" of some metrics.

```
wakeup_jitters recent values: 
  202076 211784 216894 219512 233099 ...
wakeup_jitters average: 255350
recv_jitters recent values: 
  234106 178137 168598 168912 181125 ...
recv_jitters average: 178862
```

Here are meanings.
**unit are [ns], but chrono::system_clock is used internally.**
**As in https://cpprefjp.github.io/reference/chrono/system_clock.html, gcc has microseconds resolution.**

| name           | value                                                                | unit |
|----------------|----------------------------------------------------------------------|------|
| wakeup_jitters | ping-pub wake up time-diff between expected wake up time and actual. | [ns] |
| recv_jitters   | time-diff between ping-pub send and ping-sub recv.                   | [ns] |

# Usage
- C-c to stop processes then metrics are printed.

```
# ping publish -> subscribe
## 1 executor 1 node
./build/tw_rclcpp/tw_1exec_1node

## 1 executor 2 node
./build/tw_rclcpp/tw_1exec_2node

################################
# 2 executors
# Run sub->pub by different terminal
################################
## 2 executors with same CPU. Run sub first.
taskset -c 1 ./build/tw_rclcpp/tw_2exec_sub
taskset -c 1 ./build/tw_rclcpp/tw_2exec_pub

## 2 executors with different CPU
taskset -c 1 ./build/tw_rclcpp/tw_2exec_sub
taskset -c 2 ./build/tw_rclcpp/tw_2exec_pub
```
