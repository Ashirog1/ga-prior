#include "ga.h"
#include "bits/stdc++.h"

Chromosome::Chromosome(const globalSetting &conf) {
  config = conf;
  chr.clear();
}

Chromosome &Chromosome::operator=(const std::vector<int> &other) {
  for (auto v : other) {
    chr.emplace_back(v, 0);
  }
  return *this;
}

void Chromosome::push_back(int c) {
  if (chr.size() >= 20)
    return;
  chr.emplace_back(c, 0);
}

void Chromosome::push_back(std::pair<int, int> c) {
  if (chr.size() >= 20)
    return;
  chr.emplace_back(c);
}

bool Chromosome::operator<(Chromosome &oth) {
  for (int i = 0; i < std::min(chr.size(), oth.chr.size()); ++i) {
    if (chr[i] < oth.chr[i])
      return true;
  }
  return false;
}

Chromosome encoding(solutionRespent &sol) {
  Chromosome chr(sol.config);
  for (auto truck : sol.truck_route)
    for (auto cus : truck) {
      if (cus.second != 0)
        chr.chr.push_back(cus);
    }

  for (auto drone : sol.drone_route) {
    for (auto route : drone)
      for (auto cus : route) {
        if (cus.second != 0)
          chr.chr.push_back(cus);
      }
  }
  return chr;
}

Chromosome encoding_norm(solutionRespent &sol) {
  Chromosome chr(sol.config);
  for (auto truck : sol.truck_route)
    for (auto cus : truck) {
      chr.chr.push_back(cus);
    }

  for (auto drone : sol.drone_route) {
    for (auto route : drone)
      for (auto cus : route) {
        chr.chr.push_back(cus);
      }
  }
  return chr;
}

solutionRespent decoding(Chromosome &chr) {
  solutionRespent sol(chr.config);

  sol.init_empty();
  // std::cout << "debug ";
  // print_out(chr.chr);
  // std::cout << '\n';

  int current_vehicle = 0, trip_id = 0;
  vehicle_types cur_vehicle_type = TRUCK;

  double cur_travel_time = 0, cur_system_time = 0;
  int prv_cus = 0;
  bool is_empty = true;

  for (auto gen : chr.chr) {
    /// std::cout << cus << ' ';
    auto cus = gen.first;
    while (true) {
      if (current_vehicle >= chr.config.NUM_TRUCK + chr.config.NUM_DRONE)
        break;
      cur_vehicle_type = (current_vehicle >= chr.config.NUM_TRUCK ? DRONE : TRUCK);
      double ncur_travel_time = cur_travel_time + chr.config.time_travel(prv_cus, cus, cur_vehicle_type) +
                                chr.config.time_travel(cus, 0, cur_vehicle_type) -
                                chr.config.time_travel(prv_cus, 0, cur_vehicle_type);
      double ncur_system_time = cur_system_time - cur_travel_time + ncur_travel_time;

      /// check valid constraint
      bool next_trip = false;

      if (current_vehicle < chr.config.NUM_TRUCK) {
        if (prv_cus != cus and ncur_travel_time < chr.config.TIME_LIMIT) {
          cur_travel_time = ncur_travel_time;
          cur_system_time = ncur_system_time;
          prv_cus = cus;
          break;
        } else {
          next_trip = true;
          ++current_vehicle;
        }
      } else {
        if (prv_cus != cus and ncur_travel_time < chr.config.TIME_LIMIT and
            ncur_system_time < chr.config.DRONE_DURATION) {
          cur_travel_time = ncur_travel_time;
          cur_system_time = ncur_system_time;
          prv_cus = cus;
          break;
        } else {
          next_trip = true;
          ++trip_id;
        }
      }
      /// make sure to split
      /// std::cout << cus << ' ' << current_vehicle << ' ' << trip_id << ' ' << next_trip << ' ' << is_empty << '\n';
      if (next_trip) {
        cur_travel_time = 0;
        prv_cus = 0;
        if (cur_vehicle_type == TRUCK) {
          cur_system_time = 0;
        } else {
          if (is_empty) {
            current_vehicle++;
            trip_id = 0;
            cur_system_time = 0;
            cur_travel_time = 0;
            is_empty = true;
          } else {
            is_empty = true;
            cur_travel_time = 0;
          }
        }
      } else {
        break;
      }
    }
    /// find next trip
    /// append to next trip
    /// std::cout << "found " << cus << ' ' << current_vehicle << ' ' << trip_id << '\n';
    if (current_vehicle >= chr.config.NUM_TRUCK + chr.config.NUM_DRONE)
      break;
    is_empty = false;
    if (current_vehicle < chr.config.NUM_TRUCK) {
      sol.truck_route[current_vehicle].emplace_back(cus, 0);
    } else {
      if (trip_id == sol.drone_route[current_vehicle - chr.config.NUM_TRUCK].size()) {
        std::vector<std::pair<int, int>> r;
        sol.drone_route[current_vehicle - chr.config.NUM_TRUCK].push_back(r);
      }
      if (trip_id > sol.drone_route[current_vehicle - chr.config.NUM_TRUCK].size())
        break;
      // std::cout << "drone\n";
      // std::cout << current_vehicle << '\n';
      // std::cout << "trip_id" << ' ' << trip_id << ' ' << sol.drone_route[current_vehicle -
      // chr.config.NUM_TRUCK].size() << '\n';
      sol.drone_route[current_vehicle - chr.config.NUM_TRUCK][trip_id].emplace_back(cus, 0);
      // std::cout << "drone\n";
    }
  }
  // std::cout << "done";
  // std::cout << '\n';
  return sol;
}

solutionRespent decode_with_weight(Chromosome &chr) {
  solutionRespent sol(chr.config);

  sol.init_empty();
  // std::cout << "debug ";
  // print_out(chr.chr);
  // std::cout << '\n';

  int current_vehicle = 0, trip_id = 0;
  vehicle_types cur_vehicle_type = TRUCK;

  double cur_travel_time = 0, cur_system_time = 0;
  int prv_cus = 0;
  bool is_empty = true;

  for (auto gen : chr.chr) {
    /// std::cout << cus << ' ';
    auto cus = gen.first;
    while (true) {
      if (current_vehicle >= chr.config.NUM_TRUCK + chr.config.NUM_DRONE)
        break;
      cur_vehicle_type = (current_vehicle >= chr.config.NUM_TRUCK ? DRONE : TRUCK);
      double ncur_travel_time = cur_travel_time + chr.config.time_travel(prv_cus, cus, cur_vehicle_type) +
                                chr.config.time_travel(cus, 0, cur_vehicle_type) -
                                chr.config.time_travel(prv_cus, 0, cur_vehicle_type);
      double ncur_system_time = cur_system_time - cur_travel_time + ncur_travel_time;

      /// check valid constraint
      bool next_trip = false;

      if (current_vehicle < chr.config.NUM_TRUCK) {
        if (prv_cus != cus and ncur_travel_time < chr.config.TIME_LIMIT) {
          cur_travel_time = ncur_travel_time;
          cur_system_time = ncur_system_time;
          prv_cus = cus;
          break;
        } else {
          next_trip = true;
          ++current_vehicle;
        }
      } else {
        if (prv_cus != cus and ncur_travel_time < chr.config.TIME_LIMIT and
            ncur_system_time < chr.config.DRONE_DURATION) {
          cur_travel_time = ncur_travel_time;
          cur_system_time = ncur_system_time;
          prv_cus = cus;
          break;
        } else {
          next_trip = true;
          ++trip_id;
        }
      }
      /// make sure to split
      /// std::cout << cus << ' ' << current_vehicle << ' ' << trip_id << ' ' << next_trip << ' ' << is_empty << '\n';
      if (next_trip) {
        cur_travel_time = 0;
        prv_cus = 0;
        if (cur_vehicle_type == TRUCK) {
          cur_system_time = 0;
        } else {
          if (is_empty) {
            current_vehicle++;
            trip_id = 0;
            cur_system_time = 0;
            cur_travel_time = 0;
            is_empty = true;
          } else {
            is_empty = true;
            cur_travel_time = 0;
          }
        }
      } else {
        break;
      }
    }
    /// find next trip
    /// append to next trip
    /// std::cout << "found " << cus << ' ' << current_vehicle << ' ' << trip_id << '\n';
    if (current_vehicle >= chr.config.NUM_TRUCK + chr.config.NUM_DRONE)
      break;
    is_empty = false;
    if (current_vehicle < chr.config.NUM_TRUCK) {
      sol.truck_route[current_vehicle].emplace_back(gen);
      sol.current_deliver[gen.first] += gen.second;
    } else {
      if (trip_id == sol.drone_route[current_vehicle - chr.config.NUM_TRUCK].size()) {
        std::vector<std::pair<int, int>> r;
        sol.drone_route[current_vehicle - chr.config.NUM_TRUCK].push_back(r);
      }
      if (trip_id > sol.drone_route[current_vehicle - chr.config.NUM_TRUCK].size())
        break;
      // std::cout << "drone\n";
      // std::cout << current_vehicle << '\n';
      // std::cout << "trip_id" << ' ' << trip_id << ' ' << sol.drone_route[current_vehicle -
      // chr.config.NUM_TRUCK].size() << '\n';
      sol.drone_route[current_vehicle - chr.config.NUM_TRUCK][trip_id].emplace_back(gen);
      sol.current_deliver[gen.first] += gen.second;
      // std::cout << "drone\n";
    }
  }
  // std::cout << "done";
  // std::cout << '\n';
  return sol;
}

Chromosome crossover_onepoint(const Chromosome &a, const Chromosome &b) {
  int pivot = rng(0, std::min(a.chr.size(), b.chr.size()));

  Chromosome ret(a.config);
  ret.chr.clear();
  for (int i = 0; i < pivot; ++i) {
    ret.chr.push_back(a.chr[i]);
  }
  for (int i = pivot; i < b.chr.size(); ++i) {
    ret.chr.push_back(b.chr[i]);
  }
  return ret;
}

std::pair<double, solutionRespent> move_swap_point(solutionRespent &cur_sol) {
  /// run pipeline after swap??
  /// run swap inter route
  auto sol = cur_sol;
  sol.repair_flow();
  for (int i = 0; i < sol.config.NUM_TRUCK; ++i) {
    double cur_distance = travel_time(sol.truck_route[i], TRUCK, sol.config);
    for (int j = 0; j + 1 < sol.truck_route[i].size(); ++j) {
      std::swap(sol.truck_route[i][j], sol.truck_route[i][j + 1]);

      double new_distance = travel_time(sol.truck_route[i], TRUCK, sol.config);

      if (cur_distance > new_distance) {
        cur_distance = new_distance;
      } else {
        // If the new distance is not better, undo the swapf
        std::swap(sol.truck_route[i][j], sol.truck_route[i][j + 1]);
      }
    }
  }
  cur_sol = sol; /// auto run

  /// run swap intra route
  const auto calc_adj = [&](std::vector<std::pair<int, int>> &route, int i, int cus, vehicle_types vh) {
    return sol.config.time_travel(cus, (i > 0) ? route[i - 1].first : 0, vh) +
           sol.config.time_travel(cus, i + 1 < (route.size() ? route[i + 1].first : 0), vh);
  };
  std::vector<double> cache_time_travel(sol.config.NUM_TRUCK, 0);
  for (int i = 0; i < sol.config.NUM_TRUCK; ++i) {
    cache_time_travel[i] = travel_time(sol.truck_route[i], TRUCK, sol.config);
  }

  for (int i = 0; i < sol.config.NUM_TRUCK; ++i) {
    for (int j = 0; j < sol.truck_route[i].size(); ++j) {
      for (int k = i + 1; k < sol.config.NUM_TRUCK; ++k) {
        for (int t = 0; t < sol.truck_route[k].size(); ++t) {
          /// try to swap j and t
          double old_d1 = calc_adj(sol.truck_route[i], j, sol.truck_route[i][j].first, TRUCK);
          double old_d2 = calc_adj(sol.truck_route[k], t, sol.truck_route[k][t].first, TRUCK);
          double old_d = old_d + old_d2;

          double new_d1 = calc_adj(sol.truck_route[i], j, sol.truck_route[k][t].first, TRUCK);
          double new_d2 = calc_adj(sol.truck_route[k], t, sol.truck_route[i][j].first, TRUCK);
          double new_d = new_d1 + new_d2;

          /// interchange w1 and w2
          if (new_d < old_d) {
            /// improve distance

            /// checking distance constraint
            if (cache_time_travel[i] - old_d1 + new_d1 < sol.config.time_limit(TRUCK) and
                cache_time_travel[k] - old_d2 + new_d2 < sol.config.time_limit(TRUCK)) {
              /// checking weight constraint

              // if (sol.weight_truck[i] - sol.truck_route[i][j].second + sol.truck_route[k][t].second <=
              //         sol.config.TRUCK_CAPACITY and
              //     sol.weight_truck[k] - sol.truck_route[k][t].second + sol.truck_route[i][j].second <=
              //         sol.config.TRUCK_CAPACITY) {
              int a = sol.truck_route[i][j].first, b = sol.truck_route[k][t].first, wa = sol.truck_route[i][j].second,
                  wb = sol.truck_route[k][t].second;
              if (sol.current_deliver[a] - wa + wb >= sol.config.CUSTOMERS[a].lower_weight and
                  sol.current_deliver[b] - wb + wa >= sol.config.CUSTOMERS[b].lower_weight) {
                std::swap(sol.truck_route[i][j].first, sol.truck_route[k][t].first);

                cache_time_travel[i] += -old_d1 + new_d1;
                cache_time_travel[k] += -old_d2 + new_d2;

                sol.current_deliver[a] += -wa + wb;
                sol.current_deliver[b] += -wb + wa;
              }
            }
          }
        }
      }

      for (int k = 0; k < sol.config.NUM_DRONE; ++k) {
        for (int dr_id = 0; dr_id < sol.drone_route[k].size(); ++dr_id) {
          for (int t = 0; t < sol.drone_route[k][dr_id].size(); ++t) {
            double old_d1 = calc_adj(sol.truck_route[i], j, sol.truck_route[i][j].first, TRUCK);
            double old_d2 = calc_adj(sol.drone_route[k][dr_id], t, sol.drone_route[k][dr_id][t].first, DRONE);
            double old_d = old_d + old_d2;

            double new_d1 = calc_adj(sol.truck_route[i], j, sol.drone_route[k][dr_id][t].first, TRUCK);
            double new_d2 = calc_adj(sol.drone_route[k][dr_id], t, sol.truck_route[i][j].first, DRONE);
            double new_d = new_d1 + new_d2;

            if (new_d < old_d) {
              /// improve distance

              /// checking distance constraint
              if (cache_time_travel[i] - old_d1 + new_d1 < sol.config.time_limit(TRUCK) and
                  travel_time(sol.drone_route[k][dr_id], DRONE, sol.config) - old_d2 + new_d2 <
                      sol.config.time_limit(DRONE)) {
                /// checking weight constraint
                // if (sol.weight_truck[i] - sol.truck_route[i][j].second + sol.drone_route[k][dr_id][t].second <=
                //         sol.config.TRUCK_CAPACITY and
                //     sol.weight_drone[k][dr_id] - sol.drone_route[k][dr_id][t].second + sol.truck_route[i][j].second
                //     <=
                //         sol.config.DRONE_CAPACITY) {

                int a = sol.truck_route[i][j].first, b = sol.drone_route[k][dr_id][t].first,
                    wa = sol.drone_route[k][dr_id][t].second, wb = sol.drone_route[k][dr_id][t].second;
                if (sol.current_deliver[a] - wa + wb >= sol.config.CUSTOMERS[a].lower_weight and
                    sol.current_deliver[b] - wb + wa >= sol.config.CUSTOMERS[b].lower_weight) {
                  std::swap(sol.truck_route[i][j].first, sol.drone_route[k][dr_id][t].first);

                  cache_time_travel[i] += -old_d1 + new_d1;

                  // sol.weight_truck[i] += -sol.truck_route[i][j].second + sol.truck_route[k][t].second;
                  // sol.weight_drone[k][dr_id] += -sol.drone_route[k][dr_id][t].second + sol.truck_route[i][j].second;
                  // }

                  sol.current_deliver[a] += -wa + wb;
                  sol.current_deliver[b] += -wa + wb;
                }
              }
            }
          }
        }
      }
    }
  }

  sol.repair_flow();
  sol.push_remain_cus();
  sol.repair_flow();
  sol.normalize();

  return {sol.fitness(), sol};
}

std::pair<double, solutionRespent> move_swap_edge(solutionRespent &cur_sol) {
  // auto sol = cur_sol;
  // sol.repair_flow();

  // for (int i = 0; i < sol.config.NUM_TRUCK; ++i) {
  //   double cur_distance = travel_time(sol.truck_route[i], TRUCK, sol.config);
  //   for (int j = 0; j + 1 < sol.truck_route[i].size(); ++j) {
  //     std::swap(sol.truck_route[i][j], sol.truck_route[i][j + 1]);

  //     double new_distance = travel_time(sol.truck_route[i], TRUCK, sol.config);

  //     if (cur_distance > new_distance) {
  //       cur_distance = new_distance;
  //     } else {
  //       // If the new distance is not better, undo the swapf
  //       std::swap(sol.truck_route[i][j], sol.truck_route[i][j + 1]);
  //     }
  //   }
  // }

  return {cur_sol.fitness(), cur_sol};
}

std::pair<double, solutionRespent> move_duplicate(solutionRespent &cur_sol) {
  auto sol = cur_sol;
  sol.repair_flow();
  const auto calc_adj = [&](std::vector<std::pair<int, int>> &route, int i, int cus, vehicle_types vh) {
    return sol.config.time_travel(cus, (i > 0) ? route[i - 1].first : 0, vh) +
           sol.config.time_travel(cus, i + 1 < (route.size() ? route[i + 1].first : 0), vh);
  };
  std::vector<double> cache_time_travel(sol.config.NUM_TRUCK, 0);
  for (int i = 0; i < sol.config.NUM_TRUCK; ++i) {
    cache_time_travel[i] = travel_time(sol.truck_route[i], TRUCK, sol.config);
  }

  for (int i = 0; i < sol.config.NUM_TRUCK; ++i) {
    for (int j = 0; j < sol.truck_route[i].size(); ++j) {
      for (int k = i + 1; k < sol.config.NUM_TRUCK; ++k) {
        for (int t = 0; t < sol.truck_route[k].size(); ++t) {
          /// try to swap j and t
          /// try to duplicate
          if (sol.truck_route[i][j].second > sol.truck_route[k][t].second) {
            /// k, t become i, j -> insert k t to prev of i, j, (i ,j) reassign weight
            /// old1 is cost of insert k, t into prev i, j
            double old_d1 = 0;
            double old_d2 = calc_adj(sol.truck_route[k], t, sol.truck_route[k][t].first, TRUCK);
            double old_d = old_d + old_d2;

            double new_d1 = insert((j > 0) ? sol.truck_route[i][j - 1].first : 0, sol.truck_route[i][j].first,
                                   sol.truck_route[k][t].first, TRUCK, sol.config);
            double new_d2 = calc_adj(sol.truck_route[k], t, sol.truck_route[i][j].first, TRUCK);
            double new_d = new_d1 + new_d2;

            // if (new_d < old_d) {
            /// improve distance

            /// checking distance constraint
            if (cache_time_travel[i] - old_d1 + new_d1 < sol.config.time_limit(TRUCK) and
                cache_time_travel[k] - old_d2 + new_d2 < sol.config.time_limit(TRUCK)) {
              /// checking weight constraint
              // std::cout << i  << ' ' << j << ' ' << k << ' ' << t << '\n';
              // std::cout << "prv\n";
              // sol.print();
              int base = sol.truck_route[i][j].second;
              std::swap(sol.truck_route[i][j].first, sol.truck_route[k][t].first);
              sol.truck_route[i][j].second = sol.truck_route[k][t].second;
              sol.truck_route[i].insert(sol.truck_route[i].begin() + j + 1,
                                        {sol.truck_route[k][t].first, base - sol.truck_route[k][t].second});

              cache_time_travel[i] += -old_d1 + new_d1;
              cache_time_travel[k] += -old_d2 + new_d2;
            }
            // }
          }
        }
      }
    }
  }
  sol.repair_flow();
  sol.push_remain_cus();
  sol.repair_flow();
  sol.normalize();
  return {sol.fitness(), sol};
}

std::pair<double, Chromosome> move_erase(Chromosome &chr) {}

GA::GA(globalSetting &conf) { config = conf; }

void GA::init_population() {
  population.clear();
  for (int i = 0; i < config.POPULATION_SIZE; ++i) {
    solutionRespent sol(config);
    sol.init_by_distance();
    sol.repair_flow();
    sol.push_remain_cus();
    sol.repair_flow();
    sol.normalize();
    // sol.print();
    population.push_back(encoding(sol));
  }
}

void GA::choose_next_population() {
  /// make sure run pipeline every new chromosome popup in population
  std::vector<std::pair<double, Chromosome>> val;
  for (auto &chr : population) {
    val.emplace_back(pipeline(chr));
  }
  std::sort(val.begin(), val.end(),
            [](std::pair<double, Chromosome> a, std::pair<double, Chromosome> b) { return a.first > b.first; });

  while (val.size() > config.ELITE_SET)
    val.pop_back();

  population.clear();

  for (auto [_, chr] : val) {
    population.push_back(chr);
  }

  while (population.size() < config.POPULATION_SIZE) {
    int i = rng(0, config.ELITE_SET - 2);
    int j = rng(i + 1, config.ELITE_SET - 1);
    Chromosome child1 = crossover_onepoint(population[i], population[j]);

    population.push_back(pipeline(child1).second);

    if (population.size() < config.POPULATION_SIZE) {
      Chromosome child2 = crossover_onepoint(population[j], population[i]);
      population.push_back(pipeline(child2).second);
    }
  }
}

void GA::local_search_mutation() {
  for (int i = 0; i < config.MUT_LOCALSEARCH_ITER; ++i) {
    int r = rng(0, population.size() - 1);
    // std::cout << r << '\n';
    // std::cout << population[r].chr.size() << '\n';
    // print_out(population[r].chr);

    population[r] = local_search(population[r]);
  }
}

void GA::mutation_process() {
  std::vector<int> perm(population.size());
  std::iota(perm.begin(), perm.end(), 0);
  std::random_device rd;
  std::mt19937 generator(rd());

  std::shuffle(perm.begin(), perm.end(), generator);
  for (int iter = 0; iter < config.MUT_SIZE; ++iter) {
    int i = perm[iter];

    int x = rng(0, 3);
    if (x == 0) {
      population[i] = mutation_swap(population[i]);
    } else if (x == 1) {
      population[i] = mutation_rev_swap(population[i]);
    } else if (x == 2) {
      population[i] = mutation_del(population[i]);
    } else {
      population[i] = mutation_ins(population[i]);
    }

    population[i] = pipeline(population[i]).second;
  }
}

void GA::ga_process() {
  init_population();

  for (int i = 0; i < config.GA_ITER; ++i) {
    // std::cout << "start generation " << i << '\n';
    // std::cout << pipeline(population[0]).first << '\n';
    choose_next_population();
    // std::cout << "done next population\n";
    mutation_process();
    // std::cout << "done mutation\n";

    if (i % config.MUT_LOCALSEARCH_ITER == 0) {
      local_search_mutation();
    }

    // std::cout << pipeline(population[0]).first << '\n';
  }
}
