/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <rs_driver/common/common_header.h>
#include <thread>
#include <atomic>
#include <queue>
#include <future>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <functional>
namespace robosense
{
namespace lidar
{
constexpr uint16_t MAX_THREAD_NUM = 2;
struct Thread
{
  Thread() : start_(false)
  {
  }
  std::shared_ptr<std::thread> thread_;
  std::atomic<bool> start_;
};
class ThreadPool
{
public:
  typedef std::shared_ptr<ThreadPool> Ptr;
  inline ThreadPool() : stop_flag_(false), idl_thr_num_(MAX_THREAD_NUM)
  {
    for (int i = 0; i < idl_thr_num_; ++i)
    {
      pool_.emplace_back([this] {
        while (!this->stop_flag_)
        {
          std::function<void()> task;
          {
            std::unique_lock<std::mutex> lock{ this->mutex_ };
            this->cv_task_.wait(lock, [this] { return this->stop_flag_.load() || !this->tasks_.empty(); });
            if (this->stop_flag_ && this->tasks_.empty())
              return;
            task = std::move(this->tasks_.front());
            this->tasks_.pop();
          }
          idl_thr_num_--;
          task();
          idl_thr_num_++;
        }
      });
    }
  }

  inline ~ThreadPool()
  {
    stop_flag_.store(true);
    cv_task_.notify_all();
    for (std::thread& thread : pool_)
    {
      thread.join();
    }
  }

public:
  template <class F, class... Args>
  inline void commit(F&& f, Args&&... args)
  {
    if (stop_flag_.load())
      throw std::runtime_error("Commit on LiDAR threadpool is stopped.");
    using RetType = decltype(f(args...));
    auto task =
        std::make_shared<std::packaged_task<RetType()>>(std::bind(std::forward<F>(f), std::forward<Args>(args)...));
    {
      std::lock_guard<std::mutex> lock{ mutex_ };
      tasks_.emplace([task]() { (*task)(); });
    }
    cv_task_.notify_one();
  }

private:
  using Task = std::function<void()>;
  std::vector<std::thread> pool_;
  std::queue<Task> tasks_;
  std::mutex mutex_;
  std::condition_variable cv_task_;
  std::atomic<bool> stop_flag_;
  std::atomic<int> idl_thr_num_;
};
}  // namespace lidar
}  // namespace robosense
