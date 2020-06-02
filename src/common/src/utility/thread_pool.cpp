#include "common/utility/thread_pool.h"
namespace robosense
{
    namespace common
    {

        ThreadPool::Ptr ThreadPool::instance_ptr = nullptr;
        std::mutex ThreadPool::instance_mutex;
        ThreadPool::ThreadPool() : stoped{false}
        {
            idl_thr_num = MAX_THREAD_NUM;
            for (int i = 0; i < idl_thr_num; ++i)
            {
                pool.emplace_back(
                    [this] {
                        while (!this->stoped)
                        {
                            std::function<void()> task;
                            {
                                std::unique_lock<std::mutex> lock{this->m_lock};
                                this->cv_task.wait(lock, [this] {
                                    return this->stoped.load() || !this->tasks.empty();
                                });
                                if (this->stoped && this->tasks.empty())
                                    return;
                                task = std::move(this->tasks.front());
                                this->tasks.pop();
                            }
                            idl_thr_num--;
                            task();
                            idl_thr_num++;
                        }
                    });
            }
        }

        ThreadPool::Ptr ThreadPool::getInstance()
        {
            if (instance_ptr == nullptr)
            {
                std::lock_guard<std::mutex> lk(instance_mutex);
                if (instance_ptr == nullptr)
                {
                    instance_ptr = std::shared_ptr<ThreadPool>(new ThreadPool);
                }
            }
            return instance_ptr;
        }
        ThreadPool::~ThreadPool()
        {
            stoped.store(true);
            cv_task.notify_all();
            for (std::thread &thread : pool)
            {
                thread.detach();
            }
        }

        int ThreadPool::idlCount() { return idl_thr_num; }

    } // namespace common
} // namespace robosense