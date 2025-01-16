/**
 * *concurrent-barrier.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/concurrent-barrier.hpp
 * @brief   Thread safe barrier for syncronization.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#ifndef STURDR_CONCURRENT_BARRIER_HPP
#define STURDR_CONCURRENT_BARRIER_HPP

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace sturdr {
class ConcurrentBarrier {
 public:
  explicit ConcurrentBarrier(std::size_t n_threads)
      : thresh_(n_threads), cnt_(n_threads), inst_(0) {
  }

  void Wait() {
    std::unique_lock<std::mutex> lock{mutex_};
    std::size_t gen = inst_;
    if (!--cnt_) {
      // all threads have reached the barrier
      inst_++;
      cnt_ = thresh_;
      cond_var_.notify_all();
    } else {
      cond_var_.wait(lock, [this, gen] { return gen != inst_; });
      // NOTE: The predicate lambda here protects against spurious wakeups of the thread. As long as
      //       'this->inst_' is equal to 'gen', the thread will not wake. 'this->inst_' will only
      //       increment when all threads have reached the barrier and are ready to be unblocked.
    }
  }

  void WaitFor(const std::chrono::milliseconds &timeout) {
    std::unique_lock<std::mutex> lock{mutex_};
    std::size_t gen = inst_;
    if (!--cnt_) {
      // all threads have reached the barrier
      inst_++;
      cnt_ = thresh_;
      cond_var_.notify_all();
    } else {
      cond_var_.wait_for(lock, timeout, [this, gen] { return gen != inst_; });
      // NOTE: The predicate lambda here protects against spurious wakeups of the thread. As long as
      //       'this->inst_' is equal to 'gen', the thread will not wake. 'this->inst_' will only
      //       increment when all threads have reached the barrier and are ready to be unblocked.
    }
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable cond_var_;
  std::size_t thresh_;  // number of total threads using barrier
  std::size_t cnt_;     // number of waiting threads
  std::size_t inst_;    // counter of barrier useages
};

}  // namespace sturdr

#endif