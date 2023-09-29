#include "src/ga.h"
#include "src/def.h"
#include <bits/stdc++.h>

namespace testing {
  void test_init() {
    globalSetting config;
    config.read_input();

    solutionRespent sol(config);
    sol.init_by_distance();
    print_out(sol.current_deliver);
    sol.repair_flow();
    print_out(sol.current_deliver);
    sol.print();
  }

  void test_chromosome() {
    /// test encoding and decoding
    globalSetting config;
    config.read_input();

    solutionRespent sol(config);
    sol.init_by_distance();
    std::cout << "done";

    sol.print();

    std::cout << '\n';
    auto chr = encoding(sol);

    print_out(chr.chr);

    std::cout << '\n';

    auto sol2 = decoding(chr);

    sol2.print(); 
    chr = pipeline(chr).second;

    print_out(chr.chr);
  }

  void test_ga() {
    globalSetting config;
    config.read_input();

    GA ga(config);
    ga.ga_process();

    std::cout << pipeline(ga.population[0]).first << ' ';
  }
};

int main(int argc, char **argv) {
  /// handle argv


  ///
  testing::test_ga();
}
