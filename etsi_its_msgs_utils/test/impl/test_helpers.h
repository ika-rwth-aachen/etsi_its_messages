#pragma once

#include <random>

class TestHelper {

public:

  double randomDouble(double min, double max) {
    std::uniform_real_distribution<double> uniform_distribution_double(min, max);
    return uniform_distribution_double(random_engine);
  }

  int randomInt(int min, int max) {
    std::uniform_int_distribution<int> uniform_distribution_int(min, max);
    return uniform_distribution_int(random_engine);
  }

private:
  std::default_random_engine random_engine;

};