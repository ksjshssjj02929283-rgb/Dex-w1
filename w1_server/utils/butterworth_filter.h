#pragma once
#include <array>
#include <cstddef>

#define FILTER_ORDER 2

struct BiquadFilter
{
    std::array<double, FILTER_ORDER + 1> b;
    std::array<double, FILTER_ORDER + 1> a;
};

class ButterworthFilter
{
   public:
    ButterworthFilter();
    double Apply(double input);

   private:
    BiquadFilter filter_;
    std::array<double, FILTER_ORDER + 1> inputHistory_{};
    std::array<double, FILTER_ORDER + 1> outputHistory_{};
    bool initialized_{false};
};