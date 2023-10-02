#include "ga.h"
#include "bits/stdc++.h"

Chromosome::Chromosome(const globalSetting &conf) {
  config = conf;
  chr.clear();
}

void Chromosome::push_back(int c) { chr.emplace_back(c); }

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
    for (auto cus : truck)
      chr.push_back(cus.first);

  for (auto drone : sol.drone_route) {
    for (auto route : drone)
      for (auto cus : route) {
        chr.push_back(cus.first);
      }
  }
  return chr;
}

solutionRespent decoding(Chromosome &chr) {
  solutionRespent sol(chr.config);

  sol.init_empty();
  // std::cout << "debug ";
  // print_out(chr.chr);

  int current_vehicle = 0, trip_id = 0;
  vehicle_types cur_vehicle_type = TRUCK;

  double cur_travel_time = 0, cur_system_time = 0;
  int prv_cus = 0;
  bool is_empty = true;

  for (auto cus : chr.chr) {
    /// std::cout << cus << ' ';
    while (true) {
      if (current_vehicle >= chr.config.NUM_TRUCK + chr.config.NUM_DRONE)
        break;
      cur_vehicle_type = (current_vehicle >= chr.config.NUM_TRUCK ? TRUCK : DRONE);
      double ncur_travel_time = cur_travel_time + chr.config.time_travel(prv_cus, cus, cur_vehicle_type) +
                                chr.config.time_travel(cus, 0, cur_vehicle_type) -
                                chr.config.time_travel(prv_cus, 0, cur_vehicle_type);
      double ncur_system_time = cur_system_time - cur_travel_time + ncur_travel_time;

      /// check valid constraint
      bool next_trip = false;
      if (current_vehicle < chr.config.NUM_TRUCK) {
        if (ncur_travel_time < chr.config.TIME_LIMIT) {
          cur_travel_time = ncur_travel_time;
          cur_system_time = ncur_system_time;
          prv_cus = cus;
          break;
        } else {
          next_trip = true;
          ++current_vehicle;
        }
      } else {
        if (ncur_travel_time < chr.config.TIME_LIMIT and ncur_system_time < chr.config.DRONE_DURATION) {
          cur_travel_time = ncur_travel_time;
          cur_system_time = ncur_system_time;
          prv_cus = cus;
          break;
        } else {
          next_trip = true;
          ++trip_id;
        }
      }
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
            is_empty = true;
          }
        }
      }
    }
    /// find next trip
    /// append to next trip
    // std::cout << cus << ' ' << current_vehicle << ' ' << trip_id << '\n';
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

std::pair<double, Chromosome> move_swap_point(Chromosome &chr) {
  /// run pipeline after swap??
  if (chr.chr.size() < 2)
    return std::make_pair(-1, chr);

  double v = 0;
  Chromosome best(chr);

  for (int i = 0; i < chr.chr.size(); ++i) {
    for (int j = i + 1; j < chr.chr.size(); ++j) {
      auto a = chr;
      std::swap(a.chr[i], a.chr[j]);

      auto tmp = pipeline(a);
      if (v < tmp.first) {
        v = tmp.first;
        best = a;
      }
    }
  }
  return pipeline(chr);
}

GA::GA(globalSetting &conf) { config = conf; }

void GA::init_population() {
  population.clear();
  for (int i = 0; i < config.POPULATION_SIZE; ++i) {
    solutionRespent sol(config);
    sol.init_by_distance();
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

    print_out(population[r].chr);

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
    std::cout << "start generation " << i << '\n';
    choose_next_population();
    std::cout << "done next population\n";
    mutation_process();
    std::cout << "done mutation\n";

    if (i % config.MUT_LOCALSEARCH_ITER == 0) {
      local_search_mutation();
      std::cout << "done localsearch\n";
    }
  }
}
