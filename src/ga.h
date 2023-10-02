#pragma once

#include "def.h"
#include "bits/stdc++.h"


/// @brief  easy way to handle localsearch
/// do move on Chromosome instead

class Chromosome {
public:
  std::vector<int> chr; 
  globalSetting config;
  Chromosome(const globalSetting&conf);
  void push_back(int c);
  bool operator < (Chromosome&oth);

};

Chromosome encoding(solutionRespent &sol);

solutionRespent decoding(Chromosome &chr);

inline std::pair<double, Chromosome> pipeline(Chromosome&chr) {
  solutionRespent sol = decoding(chr);
  
  sol.repair_flow();

  /// sol.print(); std::cout << '\n';

  sol.push_remain_cus();

  sol.repair_flow();

  return {sol.evaluate(), encoding(sol)};
}



/// local seearch util


std::pair<double, Chromosome> move_swap_point(Chromosome&chr);

/// @brief swap u->v and w
std::pair<double, Chromosome> move_swap_edge(Chromosome&chr);

std::pair<double, Chromosome> move_insert_point(Chromosome&chr);

std::pair<double, Chromosome> move_insert_edge(Chromosome&chr);

inline Chromosome local_search(Chromosome &chr) {
  ///
  auto cur = pipeline(chr);
  for (int iter = 0; iter < chr.config.LOCALSEARCH_ITER; ++iter) {
    auto nxt = move_swap_point(chr);

    if (nxt.first > cur.first) {
      cur.second = nxt.second;
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

  ret.chr.insert(ret.chr.begin() + i, cus);
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
