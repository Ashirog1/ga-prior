#pragma once

#include "bits/stdc++.h"

/// random libary
double rng(double l, double r);
std::vector<double> partial_sum(const std::vector<double> &prob);
int rng_p(const std::vector<double> &partial);

enum vehicle_types { TRUCK, DRONE };

class Customer {
public:
private:
public:
  int lower_weight = 0, upper_weight = 0, cost = 0, current_weight;
  double x = 0, y = 0;
  int customer_id;
};

class globalSetting {
public:
  int NUM_CUSTOMER, NUM_TRUCK, NUM_DRONE, TIME_LIMIT;
  double TRUCK_SPEED, DRONE_SPEED;
  int TRUCK_CAPACITY, DRONE_CAPACITY;
  int DRONE_DURATION;

  /// ga paramater
  int GA_ITER = 10, MUT_LOCALSEARCH_ITER = 10, POPULATION_SIZE = 100;

  int LOCALSEARCH_ITER = 5;
  int ELITE_SET = 30, MUT_SIZE = 10;
  double WDTRUCK = (double)0.23, WDDRONE = (double)0.025;

  int limit_weight(vehicle_types vehicle_type);

  double speed_of_vehicle(vehicle_types vehicle_type);

  double time_limit(vehicle_types vehicle_type);

  double euclid_distance(const Customer &a, const Customer &b);

  double time_travel(const int &a, const int &b, vehicle_types vehicle_type);

  std::vector<Customer> CUSTOMERS;

  void read_input();

  /// GA setting
};

class solutionRespent {
public:
  using TRoute = std::vector<std::pair<int, int>>;
  std::vector<TRoute> truck_route;
  std::vector<std::vector<TRoute>> drone_route;
  globalSetting config;
  std::vector<int> current_deliver;
  int current_route_idx;
  std::vector<int> weight_truck;
  std::vector<std::vector<int>> weight_drone;

  solutionRespent(globalSetting &conf);

  void init_empty();
  void init_by_distance();
  void repair_flow();
  void push_remain_cus();
  double evaluate();
  double fitness();
  bool is_valid();
  void print();
  void local_search();
  void move_11();
  void move_10();
  void normalize();
  /// to handle pushing process
};


/// inline route util (find position to insert by distance)
inline bool valid(std::vector<std::pair<int, int>> route, globalSetting&config) {
  /// recalc 
  return false;
}

inline double insert(int a, int b, int c, vehicle_types vh, globalSetting&config) {
  return -config.time_travel(a, b, vh) + config.time_travel(b, c, vh) + config.time_travel(a, c, vh);
}

inline double travel_time(const std::vector<std::pair<int, int>> cus, vehicle_types vh, globalSetting&config) {
  if (cus.empty()) return (double)0;
  double ret = config.time_travel(0, cus[0].first, vh);
  for (int i = 0; i < cus.size(); ++i) {
    ret += config.time_travel(cus[i].first, (i + 1 < cus.size() ? cus[i + 1].first : 0), vh);
  }
  return ret;
}

/// inline util
template<typename T>inline void print_out(std::vector<T> vec) {
  std::cout << "[";
  for (auto v : vec) {
    std::cout << v << ", ";
  }
  std::cout << "]";
}

template<typename T>inline void print_out(std::vector<std::pair<T, T>> vec) {
  std::cout << "[";
  for (auto v : vec) {
    std::cout << "[" << v.first << ' ' << v.second << "], ";
  }
  std::cout << "]";
}