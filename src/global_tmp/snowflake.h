// Twitter snowflake algorithm
// from https://github.com/sniper00/snowflake-cpp/tree/master and modified
// origin bits:
// 1 no use
// 41 time ms
// 5 data center
// 5 worker
// 12 seq/ms
// changed to:
// 35 time ms
// 21 DATACENTER_ID_BITS IP lower bit "data center"
// 4 WORKER_ID_BITS process id "worker"
// 4 SEQUENCE_BITS seq/ms

#pragma once
#include <chrono>
#include <cstdint>
#include <mutex>
#include <stdexcept>

class snowflake_nonlock {
 public:
  void lock() {}
  void unlock() {}
};

// @param Twepoch start timestamp (ms)
template <uint64_t Twepoch, typename Lock = snowflake_nonlock>
class snowflake {
  using lock_type = Lock;
  static constexpr uint64_t TWEPOCH = Twepoch;
  // static constexpr uint64_t WORKER_ID_BITS = 5L;
  static constexpr uint64_t WORKER_ID_BITS = 4L;
  // static constexpr uint64_t DATACENTER_ID_BITS = 5L;
  static constexpr uint64_t DATACENTER_ID_BITS = 21L;
  static constexpr uint64_t MAX_WORKER_ID = (1 << WORKER_ID_BITS) - 1;
  static constexpr uint64_t MAX_DATACENTER_ID = (1 << DATACENTER_ID_BITS) - 1;
  // static constexpr uint64_t SEQUENCE_BITS = 12L;
  static constexpr uint64_t SEQUENCE_BITS = 4L;
  static constexpr uint64_t WORKER_ID_SHIFT = SEQUENCE_BITS;
  static constexpr uint64_t DATACENTER_ID_SHIFT =
      SEQUENCE_BITS + WORKER_ID_BITS;
  static constexpr uint64_t TIMESTAMP_LEFT_SHIFT =
      SEQUENCE_BITS + WORKER_ID_BITS + DATACENTER_ID_BITS;
  static constexpr uint64_t SEQUENCE_MASK = (1 << SEQUENCE_BITS) - 1;

  using time_point = std::chrono::time_point<std::chrono::steady_clock>;

  time_point start_time_point_ = std::chrono::steady_clock::now();
  uint64_t start_millsecond_ =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();

  uint64_t last_timestamp_ = -1;
  uint64_t workerid_ = 0;
  uint64_t datacenterid_ = 0;
  uint64_t sequence_ = 0;
  lock_type lock_;

 public:
  snowflake() = default;

  snowflake(const snowflake&) = delete;

  snowflake& operator=(const snowflake&) = delete;

  void init(uint64_t workerid, uint64_t datacenterid) {
    if (workerid > MAX_WORKER_ID || workerid < 0) {
      throw std::runtime_error("worker Id can't be greater or less than 0");
    }

    if (datacenterid > MAX_DATACENTER_ID || datacenterid < 0) {
      throw std::runtime_error("datacenter Id can't be greater or less than 0");
    }

    workerid_ = workerid;
    datacenterid_ = datacenterid;
  }

  uint64_t nextid() {
    std::lock_guard<lock_type> lock(lock_);
    // std::chrono::steady_clock  cannot decrease as physical time moves forward
    auto timestamp = millsecond();
    if (last_timestamp_ == timestamp) {
      sequence_ = (sequence_ + 1) & SEQUENCE_MASK;
      if (sequence_ == 0) {
        timestamp = wait_next_millis(last_timestamp_);
      }
    } else {
      sequence_ = 0;
    }

    last_timestamp_ = timestamp;

    return ((timestamp - TWEPOCH) << TIMESTAMP_LEFT_SHIFT) |
           (datacenterid_ << DATACENTER_ID_SHIFT) |
           (workerid_ << WORKER_ID_SHIFT) | sequence_;
  }

 private:
  uint64_t millsecond() const noexcept {
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start_time_point_);
    return start_millsecond_ + diff.count();
  }

  uint64_t wait_next_millis(uint64_t last) const noexcept {
    auto timestamp = millsecond();
    while (timestamp <= last) {
      timestamp = millsecond();
    }
    return timestamp;
  }
};
