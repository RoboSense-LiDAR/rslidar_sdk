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
#include <mutex>
#include <unistd.h>
namespace robosense
{
    namespace common
    {
        template <typename T>
        class Queue
        {
        public:
            Queue()
            {
                is_task_finished = true;
            }
            void push(const T &value)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_quque.push(value);
            }

            void pop()
            {
                if (!m_quque.empty())
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_quque.pop();
                }
            }

            void clear()
            {
                std::queue<T> empty;
                std::lock_guard<std::mutex> lock(m_mutex);
                swap(empty, m_quque);
            }

        public:
            std::queue<T> m_quque;
            std::atomic<bool> is_task_finished;

        private:
            mutable std::mutex m_mutex;
        };
    } // namespace common
} // namespace robosense