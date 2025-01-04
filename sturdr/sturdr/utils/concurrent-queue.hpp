/**
 * *concurrent-queue.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/utils/concurrent-queue.hpp
 * @brief   Thread safe std::queue for syncronization.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#ifndef STURDR_CONCURRENT_QUEUE_HPP
#define STURDR_CONCURRENT_QUEUE_HPP

#include <condition_variable>
#include <mutex>
#include <queue>

namespace sturdr {

template <typename T>
class ConcurrentQueue {
 private:
  mutable std::mutex mutex_;
  std::queue<T> queue_;
  std::condition_variable cond_var_;

 public:
  /**
   * *=== ConcurrentQueue ===*
   * @brief Constructor
   */
  explicit ConcurrentQueue(){};

  /**
   * *=== ~ConcurrentQueue ===*
   * @brief Destructor
   */
  ~ConcurrentQueue(){};

  /**
   * *=== push ===*
   * @brief Push data into the queue
   */
  void push(const T& data) {
    // acquire lock
    std::unique_lock<std::mutex> lock(mutex_);

    // Add item to queue
    queue_.push(data);

    // Notify waiting thread
    cond_var_.notify_one();
  }

  /**
   * *=== pop ===*
   * @brief Remove element from the queue
   * @return True|False based on if queue had item to return
   */
  bool pop(T& data) {
    // acquire lock
    std::unique_lock<std::mutex> lock(mutex_);

    // return value
    if (!queue_.empty()) {
      data = queue_.front();
      queue_.pop();
      return true;
    }
    return false;
  };

  /**
   * *=== size ===*
   * @brief Gets the size of the queue
   * @returns The queue size
   */
  std::size_t size() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.size();
  };

  /**
   * *=== clear ===*
   * @brief Clears the queue.
   */
  void clear() {
    queue_ = std::queue<T>();
  };
};

}  // end namespace sturdr
#endif