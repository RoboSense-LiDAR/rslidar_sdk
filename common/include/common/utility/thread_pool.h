/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#pragma once
#include <vector>
#include <queue>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <future>
#include <functional>
#include <stdexcept>

namespace robosense
{
    namespace common
    {
        const int MAX_THREAD_NUM = 16;
        struct Thread
        {
            Thread()
            {
                start = false;
            }
            std::shared_ptr<std::thread> m_thread;
            std::atomic<bool> start;
        };
        class ThreadPool
        {

        private:
            inline ThreadPool();

        public:
            typedef std::shared_ptr<ThreadPool> Ptr;
            ThreadPool(ThreadPool &) = delete;
            ThreadPool &operator=(const ThreadPool &) = delete;
            ~ThreadPool();

        public:
            static Ptr getInstance();
            int idlCount();
            template <class F, class... Args>
            inline auto commit(F &&f, Args &&... args) -> std::future<decltype(f(args...))>
            {
                if (stoped.load())
                    throw std::runtime_error("Commit on ThreadPool is stopped.");
                using RetType = decltype(f(args...));
                auto task = std::make_shared<std::packaged_task<RetType()>>(
                    std::bind(std::forward<F>(f), std::forward<Args>(args)...)); // wtf !
                std::future<RetType> future = task->get_future();
                {
                    std::lock_guard<std::mutex> lock{m_lock};
                    tasks.emplace(
                        [task]() {
                            (*task)();
                        });
                }
                cv_task.notify_one();
                return future;
            }

        private:
            using Task = std::function<void()>;
            std::vector<std::thread> pool;
            std::queue<Task> tasks;
            std::mutex m_lock;
            std::condition_variable cv_task;
            std::atomic<bool> stoped;
            std::atomic<int> idl_thr_num;
            static Ptr instance_ptr;
            static std::mutex instance_mutex;
        };

    } // namespace common
} // namespace robosense