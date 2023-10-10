#include "def.h"
#include "bits/stdc++.h"
#include "network_simplex.h"

int globalSetting::limit_weight(vehicle_types vehicle_type) {
  if (vehicle_type == TRUCK)
    return globalSetting::TRUCK_CAPACITY;
  else
    return globalSetting::DRONE_CAPACITY;
}

double globalSetting::speed_of_vehicle(vehicle_types vehicle_type) {
  if (vehicle_type == TRUCK)
    return globalSetting::TRUCK_SPEED;
  else
    return globalSetting::DRONE_SPEED;
}

double globalSetting::time_limit(vehicle_types vehicle_type) {
  if (vehicle_type == TRUCK)
    return globalSetting::TIME_LIMIT;
  else
    return std::min(globalSetting::TIME_LIMIT, globalSetting::DRONE_DURATION);
}

double globalSetting::euclid_distance(const Customer &a, const Customer &b) {
  return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + 0.5);
}

double globalSetting::time_travel(const int &a, const int &b, vehicle_types vehicle_type) {
  return (double)euclid_distance(CUSTOMERS[a], CUSTOMERS[b]) / speed_of_vehicle(vehicle_type);
}

void globalSetting::read_input() {
  // std::ifstream in("input.txt");

  std::cin >> NUM_CUSTOMER >> TRUCK_CAPACITY;

  NUM_TRUCK = NUM_CUSTOMER + 5;
  NUM_DRONE = 0;
  TRUCK_SPEED = 1;
  DRONE_SPEED = 0; // make sure never use drone
  Customer tmp;

  TIME_LIMIT = 10000000;

  NUM_CUSTOMER++;

  CUSTOMERS.resize(NUM_CUSTOMER);

  for (int i = 1; i < NUM_CUSTOMER; ++i) {
    std::cin >> CUSTOMERS[i].lower_weight;

    CUSTOMERS[i].upper_weight = CUSTOMERS[i].lower_weight;

    CUSTOMERS[i].cost = 1;
  }

  for (int i = 0; i < NUM_CUSTOMER; ++i) {
    std::cin >> CUSTOMERS[i].x >> CUSTOMERS[i].y;

    // std::cout <<CUSTOMERS[i].x << ' '  << CUSTOMERS[i].y << '\n';
  }
}

double rng(double l, double r) {
  std::uniform_real_distribution<double> unif(l, r);
  std::random_device re;
  return unif(re);
}

int rng(int l, int r) {
  std::uniform_int_distribution<int> unif(l, r);
  std::random_device re;
  return unif(re);
}

std::vector<double> partial_sum(const std::vector<double> &prob) {
  std::vector<double> partial(prob.size(), 0);
  for (int i = 0; i < (int)prob.size(); ++i) {
    partial[i] = (i == 0 ? 0 : partial[i - 1]) + prob[i];
  }
  return partial;
}

int rng_p(const std::vector<double> &partial) {
  double dice = rng((double)0, partial.back());
  return std::lower_bound(partial.begin(), partial.end(), dice) - partial.begin();
}

solutionRespent::solutionRespent(globalSetting &conf) { config = conf; }

void solutionRespent::init_empty() {
  truck_route.resize(config.NUM_TRUCK, TRoute());
  drone_route.resize(config.NUM_DRONE);
  current_deliver.resize(config.NUM_CUSTOMER, 0);
}

void solutionRespent::init_by_distance() {
  init_empty();
  std::fill(current_deliver.begin(), current_deliver.end(), 0);

  const auto get_remain_weight = [&]() {
    std::vector<double> remain_weight(config.NUM_CUSTOMER, 0);
    for (int i = 0; i < config.NUM_CUSTOMER; ++i) {
      remain_weight[i] = std::max(0.0, (double)config.CUSTOMERS[i].lower_weight - current_deliver[i]);
    }
    return remain_weight;
  };
  const auto build_route = [&](vehicle_types vehicle_type, double system_limit = 0) {
    std::vector<std::pair<int, int>> route;
    int truck_weight = config.limit_weight(vehicle_type);
    double travel_time = 0;
    int tmp = 0;

    double time_travel_limit = config.time_limit(vehicle_type);
    if (vehicle_type == DRONE) {
      time_travel_limit = std::min(time_travel_limit, system_limit);
    }

    while (true) {
      if (truck_weight == 0)
        break;
      int next_cus = -1;
      if (tmp == 0) {
        next_cus = rng_p(get_remain_weight());
      } else {
        double min_distance = std::numeric_limits<double>::max();
        for (int cus = 1; cus < config.NUM_CUSTOMER; ++cus) {
          if (current_deliver[cus] >= config.CUSTOMERS[cus].lower_weight)
            continue;
          /*if (min_distance > config.time_travel(tmp, cus, vehicle_type) + config.time_travel(cus, 0, vehicle_type)) {
            min_distance = config.time_travel(tmp, cus, vehicle_type) + config.time_travel(cus, 0, vehicle_type);
            next_cus = cus;
          }*/
          if (travel_time + config.time_travel(tmp, cus, vehicle_type) + config.time_travel(cus, 0, vehicle_type) >
              time_travel_limit)
            continue;
          if (min_distance > config.time_travel(tmp, cus, vehicle_type)) {
            min_distance = config.time_travel(tmp, cus, vehicle_type);
            next_cus = cus;
          }
        }
      }
      /// find next customer minimum distance
      if (next_cus == -1 or next_cus == 0) {
        travel_time += config.time_travel(tmp, 0, vehicle_type);
        break;
      }

      /// check valid time constraint
      /*  double ntravel_time = travel_time + config.time_travel(tmp, next_cus, vehicle_type) +
                              config.time_travel(next_cus, 0, vehicle_type) - config.time_travel(tmp, 0, vehicle_type);
        if (ntravel_time > time_travel_limit)
          break;*/
      travel_time = travel_time + config.time_travel(tmp, next_cus, vehicle_type);
      tmp = next_cus;
      // travel_time = ntravel_time;
      route.emplace_back(next_cus, 0);
      int weight = std::min(truck_weight, config.CUSTOMERS[next_cus].lower_weight - current_deliver[next_cus]);
      current_deliver[next_cus] += weight;
      truck_weight -= weight;
      if (weight == 0)
        break;
    }
    return std::make_pair(route, travel_time);
  };

  for (int i = 0; i < config.NUM_TRUCK; ++i) {
    truck_route[i] = build_route(TRUCK).first;
  }
  for (int i = 0; i < config.NUM_DRONE; ++i) {
    double system_limit = 0;
    drone_route[i].clear();
    while (true) {
      auto [route, travel_time] = build_route(DRONE, config.DRONE_DURATION - system_limit);
      if (route.empty() or travel_time == 0)
        break;
      drone_route[i].push_back(route);
      system_limit += travel_time;
    }
  }
}

void solutionRespent::repair_flow() {
  weight_truck.assign(config.NUM_TRUCK, 0);
  weight_drone.resize(config.NUM_DRONE);
  std::fill(current_deliver.begin(), current_deliver.end(), 0);

  int N = 0, M = config.NUM_CUSTOMER;
  for (int i = 0; i < config.NUM_TRUCK; ++i)
    N += 1;
  for (int i = 0; i < config.NUM_DRONE; ++i) {
    N += drone_route[i].size();
  }
  int S = N + M + 1, T = S + 1;
  network_simplex<long, long, int64_t> ns(T + 5);

  int cur_route = 0;
  for (int i = 0; i < config.NUM_TRUCK; ++i) {
    ++cur_route;
    for (auto cus : truck_route[i]) {
      ns.add(cur_route, N + cus.first, 0, INT_MAX, 0);
    }
    ns.add(S, cur_route, 0, config.TRUCK_CAPACITY, 0);
  }
  for (int i = 0; i < config.NUM_DRONE; ++i) {
    for (auto &route : drone_route[i]) {
      ++cur_route;
      for (auto cus : route) {
        ns.add(cur_route, N + cus.first, 0, INT_MAX, 0);
        // debug(cur_route, N + cus.customer_id, N);
      }
      ns.add(S, cur_route, 0, config.DRONE_CAPACITY, 0);
    }
  }
  for (int i = 1; i < config.NUM_CUSTOMER; ++i) {
    ns.add(N + i, T, config.CUSTOMERS[i].lower_weight, config.CUSTOMERS[i].upper_weight, -config.CUSTOMERS[i].cost);
  }
  ns.add(T, S, 0, INT_MAX, 0);
  bool ok = ns.mincost_circulation();

  /// assign weight based on mcmf solution
  std::fill(current_deliver.begin(), current_deliver.end(), 0);
  cur_route = 0;
  int e = 0;
  for (int i = 0; i < config.NUM_TRUCK; ++i) {
    weight_truck[i] = 0;
    for (auto &cus : truck_route[i]) {
      cus.second = ns.get_flow(e++);
      current_deliver[cus.first] += cus.second;
      weight_truck[i] += cus.second;
    }
    e++;
  }
  for (int i = 0; i < config.NUM_DRONE; ++i) {
    weight_drone[i].clear();
    for (auto &route : drone_route[i]) {
      int tmp = 0;
      for (auto &cus : route) {
        cus.second = ns.get_flow(e++);
        current_deliver[cus.first] += cus.second;
        tmp += cus.second;
      }
      weight_drone[i].push_back(tmp);
      e++;
    }
  }
}

void solutionRespent::push_remain_cus() {
  for (int i = 1; i < config.NUM_CUSTOMER; ++i) {
    if (current_deliver[i] < config.CUSTOMERS[i].upper_weight) {
      int rem_weight = config.CUSTOMERS[i].upper_weight - current_deliver[i];
      while (rem_weight > 0) {
        /// find route to insert?
        int prv_weight = rem_weight;
        for (int t = 0; t < config.NUM_TRUCK; ++t) {
          if (weight_truck[t] >= config.TRUCK_CAPACITY)
            continue;

          if (truck_route[t].size() == 0) {
            int pushed_weight = std::min(rem_weight, config.TRUCK_CAPACITY);
            truck_route[t].emplace_back(i, std::min(rem_weight, config.TRUCK_CAPACITY));

            rem_weight -= pushed_weight;
            weight_truck[t] += pushed_weight;
            current_deliver[i] += pushed_weight;
            continue;
          }

          int ind = 0;
          double min_added = insert(0, truck_route[t][0].first, i, TRUCK, config);
          int cur_weight = 0;
          for (int i = 0; i < truck_route[t].size(); ++i) {
            cur_weight += truck_route[t][i].second;
            int nxt = (i + 1 < truck_route[t].size() ? truck_route[t][i].first : 0);
            double cur_add = insert(truck_route[t][i].first, nxt, i, TRUCK, config);
            if (min_added > cur_add) {
              min_added = cur_add;
              ind = i;
            }
          }
          if (travel_time(truck_route[t], TRUCK, config) + min_added < config.TIME_LIMIT) {
            int pushed_weight = std::min(rem_weight, config.TRUCK_CAPACITY - weight_truck[t]);
            truck_route[t].insert(truck_route[t].begin() + ind, {i, pushed_weight});

            rem_weight -= pushed_weight;
            weight_truck[t] += pushed_weight;

            current_deliver[i] += pushed_weight;
          }
        }
        /// std::cout << i << ' ' << rem_weight << '\n';

        /// do the same thing for drone
        /// TODO:
        /// create new drone route?
        for (int d = 0; d < config.NUM_DRONE; ++d) {
          /// calculate existed system limit wheight
          /// std::cout << d << ' ';
          double system_time = 0;
          int exist_empty_route = -1;
          for (int r = 0; r < drone_route[d].size(); ++r) {
            if (drone_route[d][r].empty())
              exist_empty_route = r;
            system_time += travel_time(drone_route[d][r], DRONE, config);
          }

          std::vector<std::pair<int, int>> r;

          if (system_time + 2.0 * config.time_travel(0, i, DRONE) < config.DRONE_DURATION and
              2.0 * config.time_travel(0, i, DRONE) < config.TIME_LIMIT) {
            int pushed_weight = std::min(rem_weight, config.DRONE_CAPACITY);
            r.emplace_back(i, pushed_weight);

            rem_weight -= pushed_weight;

            current_deliver[i] += pushed_weight;
            if (exist_empty_route == -1)
              drone_route[d].emplace_back(r);
            else
              drone_route[d][exist_empty_route] = r;
          }

          // print_out(r); std::cout << '\n';
        }
        if (rem_weight == prv_weight)
          break;
      }
    }
  }
}

double solutionRespent::evaluate() {
  /// make sure to run pipelines before run eval

  double ret = 0;
  for (int i = 0; i < config.NUM_CUSTOMER; ++i) {
    ret += (double)current_deliver[i] * config.CUSTOMERS[i].cost;
  }
  return ret;
}

double solutionRespent::fitness() {
  double truck_travel_time = 0, drone_travel_time = 0;
  for (int i = 0; i < config.NUM_TRUCK; ++i) {
    truck_travel_time += travel_time(truck_route[i], TRUCK, config);
  }
  for (int i = 0; i < config.NUM_DRONE; ++i) {
    for (int r = 0; r < drone_route[i].size(); ++r) {
      drone_travel_time += travel_time(drone_route[i][r], DRONE, config);
    }
  }
  return -truck_travel_time * config.WDTRUCK - drone_travel_time * config.WDDRONE;
}

bool solutionRespent::is_valid() {
  for (int i = 1; i < config.NUM_CUSTOMER; ++i) {
    if (current_deliver[i] < config.CUSTOMERS[i].lower_weight)
      return false;
    if (current_deliver[i] > config.CUSTOMERS[i].upper_weight)
      return false;
  }
  // std::cout << "pass weight check\n";
  for (int i = 0; i < config.NUM_TRUCK; ++i) {
    if (travel_time(truck_route[i], TRUCK, config) > config.TIME_LIMIT)
      return false;
  }
  for (int i = 0; i < config.NUM_DRONE; ++i) {
    double drone_travel_time = 0;
    for (int r = 0; r < drone_route[i].size(); ++r) {
      int tmp = travel_time(drone_route[i][r], DRONE, config);
      drone_travel_time += tmp;
    }
    if (drone_travel_time > config.DRONE_DURATION)
      return false;
  }
  return true;
}

void solutionRespent::print() {
  std::cout << "truck route \n";
  for (int i = 0; i < config.NUM_TRUCK; ++i) {
    std::cout << "route of truck " << i << " ";
    print_out(truck_route[i]);
    std::cout << '\n';
  }
  for (int i = 0; i < config.NUM_DRONE; ++i) {
    std::cout << "route of drone " << i << '\n';
    for (auto route : drone_route[i]) {
      print_out(route);
      std::cout << '\n';
    }
    std::cout << '\n';
  }
}
