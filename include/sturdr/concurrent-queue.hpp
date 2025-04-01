/**
 * *concurrent-queue.hpp*
 *
 * =======  ========================================================================================
 * @file    sturdr/concurrent-queue.hpp
 * @brief   Thread safe std::queue for syncronization.
 * @date    December 2024
 * @author  Daniel Sturdivant <sturdivant20@gmail.com>
 * =======  ========================================================================================
 */

#ifndef STURDR_CONCURRENT_QUEUE_HPP
#define STURDR_CONCURRENT_QUEUE_HPP

#include <any>
#include <condition_variable>
#include <mutex>
#include <queue>

namespace sturdr {

class ConcurrentQueue {
 private:
  mutable std::mutex mutex_;
  std::queue<std::any> queue_;
  std::condition_variable cond_var_;
  bool is_finished_;

 public:
  /**
   * *=== ConcurrentQueue ===*
   * @brief Constructor
   */
  explicit ConcurrentQueue() : is_finished_{false} {};

  /**
   * *=== ~ConcurrentQueue ===*
   * @brief Destructor
   */
  ~ConcurrentQueue() = default;

  /**
   * *=== push ===*
   * @brief Push data into the queue
   */
  template <typename T>
  void push(const T& data) {
    std::unique_lock<std::mutex> lock(mutex_);  // acquire lock
    queue_.push(data);                          // copy item to queue
    cond_var_.notify_one();                     // notify waiting thread
  }

  /**
   * *=== pop ===*
   * @brief Remove element from the queue
   * @return True|False based on if queue had item to return
   */
  bool pop(std::any& data) {
    // acquire lock and wait for data
    std::unique_lock<std::mutex> lock(mutex_);
    cond_var_.wait(lock, [this] { return !queue_.empty() || is_finished_; });

    // make sure we have not reached the end
    if (is_finished_ && queue_.empty()) return false;

    // copy into provided memory
    data = queue_.front();
    queue_.pop();
    return true;
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
    std::unique_lock<std::mutex> lock(mutex_);
    queue_ = std::queue<std::any>();
  };
  bool empty() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void NotifyComplete() {
    is_finished_ = true;
    cond_var_.notify_all();
  }
  bool IsFinished() const {
    std::unique_lock<std::mutex> lock(mutex_);
    return is_finished_;
  }
};

}  // namespace sturdr
#endif