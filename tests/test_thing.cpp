#include <condition_variable>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "sturdr/concurrent-queue.hpp"
#include "sturdr/structs-enums.hpp"

void WorkerThread(
    uint8_t i,
    std::shared_ptr<sturdr::ConcurrentQueue<uint8_t>> q,
    std::shared_ptr<std::mutex> mtx,
    std::shared_ptr<std::condition_variable> cv,
    std::shared_ptr<bool> done) {
  for (int j = 0; j < 100; j++) {
    // std::cout << "thread " << i << "pushing...\n";
    q->push(i);
    // std::cout << "thread " << i << "waiting...\n";
    std::unique_lock<std::mutex> lock(*mtx);
    cv->wait(lock, [done] { return *done; });
    *done = false;
  }
}
// void WorkerThread(uint8_t i) {
//   std::cout << "thread made\n";
// }

int main() {
  const uint8_t N = 8;
  std::map<uint8_t, sturdr::ChannelNavData> data;
  std::shared_ptr<sturdr::ConcurrentQueue<uint8_t>> q =
      std::make_shared<sturdr::ConcurrentQueue<uint8_t>>();
  std::shared_ptr<std::mutex> mtx = std::make_shared<std::mutex>();
  std::vector<std::shared_ptr<std::condition_variable>> cv(N);
  std::vector<std::shared_ptr<bool>> done(N);
  std::vector<std::shared_ptr<std::thread>> th(N);

  // std::cout << "creating threads...\n";
  uint8_t channel_id = 0;
  for (uint8_t i = 0; i < N; i++) {
    data.insert({static_cast<uint8_t>(i), sturdr::ChannelNavData()});
    done[i] = std::make_shared<bool>(false);
    cv[i] = std::make_shared<std::condition_variable>();
    th[i] = std::make_shared<std::thread>(&WorkerThread, i, q, mtx, cv[i], done[i]);
    // std::cout << "i = " << (int)i << "\n";
  }
  std::cout << "starting...\n";

  for (int i = 0; i < 100 * N; i++) {
    // std::cout << "main thread waiting...\n";
    q->pop(channel_id);
    std::unique_lock<std::mutex> lock(*mtx);

    data[channel_id].ReadyForVT = true;

    std::cout << "Ready4VT: [ ";
    bool tmp = true;
    for (const std::pair<const uint8_t, sturdr::ChannelNavData> &it : data) {
      tmp &= it.second.ReadyForVT;
      std::cout << it.second.ReadyForVT << " ";
    }
    std::cout << "]\n";
    std::cout << "tmp: " << tmp << "\n";

    if (tmp) {
      for (std::pair<const uint8_t, sturdr::ChannelNavData> &it : data) {
        it.second.ReadyForVT = false;
      }
      std::cout << i << " reset...\n";
      for (uint8_t j = 0; j < N; j++) {
        *done[j] = true;
        cv[j]->notify_one();
      }
    }
  }

  for (std::shared_ptr<std::thread> &t : th) {
    t->join();
  }

  return 0;
}