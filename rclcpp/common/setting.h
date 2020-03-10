#ifndef SETTING_H_
#define SETTING_H_

const char * node_name_pub = "one_node_sub";
const char * node_name_sub = "one_node_pub";
const char * namespace_ = "ns";
const char * topic_name = "ping";
const int qos = 10;
const int period_ns = 100 * 1000 * 1000; // wake up period[ns]
const int num_bin = 10000; // number of time reports.

#endif  /* SETTING_H_ */
