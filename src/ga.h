#pragma once

#include "def.h"
#include "bits/stdc++.h"


/// @brief  easy way to handle localsearch
/// do move on Chromosome instead

class Chromosome {
public:
  std::vector<std::pair<int, int>> chr; 
  globalSetting config;
  Chromosome(const globalSetting&conf);
  void push_back(int c);
  void push_back(std::pair<int, int> c);
  bool operator < (Chromosome&oth);
  Chromosome& operator=(const std::vector<int> &other);
};

Chromosome encoding(solutionRespent &sol);
Chromosome encoding_norm(solutionRespent &sol);

solutionRespent decoding(Chromosome &chr);


solutionRespent decode_with_weight(Chromosome &chr);

inline std::pair<double, Chromosome> pipeline(Chromosome&chr) {
  solutionRespent sol = decoding(chr);
  
  sol.repair_flow();

  /// sol.print(); std::cout << '\n';
  // std::cout << chr.chr.size() << '\n';
  sol.push_remain_cus();

  sol.repair_flow();

  if (not sol.is_valid()) return {sol.fitness() - INT_MAX, encoding_norm(sol) };

  return {sol.fitness(), encoding(sol)};
}



/// local seearch util


std::pair<double, Chromosome> move_swap_point(Chromosome&chr);

/// @brief swap u->v and w
std::pair<double, Chromosome> move_swap_edge(Chromosome&chr);

std::pair<double, Chromosome> move_insert_point(Chromosome&chr);

std::pair<double, Chromosome> move_insert_edge(Chromosome&chr);

std::pair<double, Chromosome> move_duplicate(Chromosome&chr);

std::pair<double, Chromosome> move_erase(Chromosome&chr);

inline Chromosome local_search(Chromosome &chr) {
  ///
  auto cur = pipeline(chr);
  for (int iter = 0; iter < chr.config.LOCALSEARCH_ITER; ++iter) {

    auto nxt = move_swap_point(chr);
    
    auto nxt2 = move_swap_edge(chr);

    // print_out(nxt.second.chr); std::cout << '\n';
    // print_out(nxt2.second.chr); std::cout << '\n';
    if (nxt2.first > nxt.first) {
      nxt = nxt2;
    }

    nxt2 = move_duplicate(chr);
    if (nxt2.first > nxt.first) {
      nxt = nxt2;
    }

    // nxt2 = move_erase(chr);
    if (nxt2.first > nxt.first) {
      nxt = nxt2;
    }

    if (nxt.first > cur.first) {
      cur = nxt;
    }
  }
  return cur.second;
}

inline Chromosome mutation_swap(const Chromosome&chr) {
  if (chr.chr.empty()) return chr;
  Chromosome ret = chr;
  int len = rng(1, std::max(1, (int)chr.chr.size() / 3));

  int n = chr.chr.size();

  int s1 = rng(0, n - 2 * len);
  int s2 = rng(s1 + len, n - len);

  for (int i = 0; i < len; ++i) {
    std::swap(ret.chr[s1 + i], ret.chr[s2 + i]);
  }
  return ret;
}

inline Chromosome mutation_rev_swap(const Chromosome&chr) {
  if (chr.chr.empty()) return chr;
  Chromosome ret = chr;
  int len = rng(1, std::max(1, (int)chr.chr.size() / 3));

  int n = chr.chr.size();

  int s1 = rng(0, n - 2 * len);
  int s2 = rng(s1 + len, n - len);

  for (int i = 0; i < len; ++i) {
    std::swap(ret.chr[s1 + i], ret.chr[s2 + len - 1 - i]);
  }
  return ret;
}

inline Chromosome mutation_del(const Chromosome&chr) {
  if (chr.chr.empty()) return chr;
  Chromosome ret = chr;
  int i = rng(0, chr.chr.size() - 1);

  ret.chr.erase(ret.chr.begin() + i);

  return ret;
}


/// crossover
Chromosome crossover_onepoint(const Chromosome&a, const Chromosome&b);

inline Chromosome mutation_ins(const Chromosome&chr) {
  Chromosome ret = chr;
  int cus = rng(1, chr.config.NUM_CUSTOMER - 1);

  int i = rng(0, chr.chr.size());

  ret.chr.insert(ret.chr.begin() + i, {cus, 0});
  return chr;
}




class GA {
public:
  std::vector<Chromosome> population;
  globalSetting config;

  GA(globalSetting&conf);
  void init_population();

  void choose_next_population();

  void mutation_process();

  void ga_process();
  
  void local_search_mutation();
};
