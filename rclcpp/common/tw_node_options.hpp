#ifndef SETTING_H_
#define SETTING_H_

class TwoWaysNodeOptions {
public:
  TwoWaysNodeOptions()
      : node_name_pub("one_node_sub"),
        node_name_sub("one_node_pub"),
        namespace_("ns"),
        topic_name("ping"),
        qos(10),
        period_ns(100 * 1000 * 1000),
        num_bin(10000) {}

  const char * node_name_pub;
  const char * node_name_sub;
  const char * namespace_;
  const char * topic_name;
  const int qos;
  // wake up period[ns]
  const int period_ns;
  const int num_bin; // number of time reports.
};

#endif  /* SETTING_H_ */
