#pragma once

#include <chrono>
#include <iostream>
#include <string>

namespace sc_pgo
{

class TicToc
{
public:
  TicToc()
  {
    tic();
  }

  void tic()
  {
    start_ = std::chrono::steady_clock::now();
  }

  double toc() const
  {
    const auto end = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = end - start_;
    return elapsed.count() * 1000.0;
  }

private:
  std::chrono::time_point<std::chrono::steady_clock> start_;
};

class TicTocLogger
{
public:
  explicit TicTocLogger(const bool display = false)
  : display_(display)
  {
    tic();
  }

  void tic()
  {
    start_ = std::chrono::steady_clock::now();
  }

  void toc(const std::string & task) const
  {
    if (!display_) {
      return;
    }
    const auto end = std::chrono::steady_clock::now();
    const std::chrono::duration<double> elapsed = end - start_;
    std::cout.precision(3);
    std::cout << task << ": " << elapsed.count() * 1000.0 << " msec." << std::endl;
  }

private:
  std::chrono::time_point<std::chrono::steady_clock> start_;
  bool display_ = false;
};

}  // namespace sc_pgo
