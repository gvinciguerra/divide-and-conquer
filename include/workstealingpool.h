//
// Created by Giorgio Vinciguerra on 23/05/2018.
//

#ifndef SPM_DIVIDE_AND_CONQUER_WORKSTEALINGPOOL_H
#define SPM_DIVIDE_AND_CONQUER_WORKSTEALINGPOOL_H

#include <vector>
#include <future>
#include <condition_variable>
#include <queue>
#include <set>
#include <unordered_map>

class WorkStealingPool {
    class Worker {
    public:
        std::deque<std::function<void()>> queue;
        std::mutex mutex;
        std::condition_variable cond;
        std::thread thread;
        WorkStealingPool &pool;

        explicit Worker(WorkStealingPool &);

        Worker(const Worker &) = default;

        void start();

        template<class F, class... Args>
        std::future<typename std::result_of<F(Args...)>::type> submit(F &&, Args &&...);
    };

public:
    explicit WorkStealingPool(size_t pool_size);

    template<class F, class... Args>
    std::future<typename std::result_of<F(Args...)>::type> submit(F &&, Args &&...);

    std::function<void()> steal();

    ~WorkStealingPool();

private:
    size_t pool_size;
    std::atomic_int remaining;
    std::atomic_bool shutdown;
    std::vector<std::shared_ptr<Worker>> workers; // O(1) random worker selection in steal()
    std::unordered_map<std::thread::id, std::shared_ptr<Worker>> workers_map; // O(1) access in submit()
};

int fast_rand() {
    static int seed = std::rand();
    seed = (214013 * seed + 2531011);
    return (seed >> 16) & 0x7FFF;
}


template<class F, class... Args>
std::future<typename std::result_of<F(Args...)>::type> WorkStealingPool::Worker::submit(F &&f, Args &&... args) {
    using result_type = typename std::result_of<F(Args...)>::type;
    auto task = std::make_shared<std::packaged_task<result_type()> >(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    std::unique_lock<std::mutex> lock(mutex);
    queue.emplace_front([task]() { (*task)(); });
    pool.remaining++;
    lock.unlock();
    cond.notify_one();

    return task->get_future();
}

WorkStealingPool::Worker::Worker(WorkStealingPool &pool) : pool(pool) {

}

void WorkStealingPool::Worker::start() {
    thread = std::thread([this]() -> void {
        constexpr size_t backoff_min = 1 << 4;
        constexpr size_t backoff_max = 1 << 10;
        static thread_local size_t backoff = backoff_min;
//        static double avg_time = 0;

        while (true) {
            if (pool.shutdown && queue.empty())
                break;
            if (queue.empty()) {
                auto stolen_task = std::move(pool.steal());
                if (stolen_task != nullptr) {
                    pool.remaining--;
                    backoff = backoff_min;
                    stolen_task();
                } else {
                    for (volatile unsigned i = 0; i < backoff; ++i);
                    if (backoff < backoff_max)
                        backoff <<= 1;
                }
            } else {
//                static long count = 0;
//                auto t1 = std::chrono::high_resolution_clock::now();
                std::function<void()> task;
                {
                    std::unique_lock<std::mutex> lock(mutex);
                    cond.wait(lock, [this] { return pool.shutdown || !queue.empty() || pool.remaining >= 0; });
                    if (queue.empty())
                        continue;
                    task = std::move(queue.back());
                    pool.remaining--;
                    queue.pop_back();
                }
//                auto t2 = std::chrono::high_resolution_clock::now();
//                auto time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
//                avg_time = (avg_time * count + time_span) / (count + 1);
//                ++count;
                backoff = backoff_min;
                task();
            }
        }
//        std::cout << "Avg. task retrieval time" << avg_time;
    });
}

template<class F, class... Args>
std::future<typename std::result_of<F(Args...)>::type>
WorkStealingPool::submit(F &&f, Args &&... args) {
    auto thread_id = std::this_thread::get_id();
    if (workers_map.find(thread_id) == workers_map.end()) {
        auto r = workers.size() == 1 ? 0 : fast_rand() / ((RAND_MAX + 1u) / (workers.size() - 1));
        return workers[r]->submit(f, args...);
    }
    return workers_map[thread_id]->submit(f, args...);
}

std::function<void()> WorkStealingPool::steal() {
    auto r = workers.size() == 1 ? 0 : fast_rand() / ((RAND_MAX + 1u) / (workers.size() - 1));
    auto victim = workers[r];
    if (victim->queue.empty())
        return nullptr; // reduce calls to try_lock
    std::unique_lock<std::mutex> lock(victim->mutex, std::try_to_lock);
    if (lock.owns_lock() && !victim->queue.empty()) {
        auto task = std::move(victim->queue.front());
        victim->queue.pop_front();
        return task;
    }
    return nullptr;
}

WorkStealingPool::WorkStealingPool(size_t pool_size) : pool_size(pool_size), shutdown(false), remaining(0) {
    if (pool_size < 1)
        throw std::invalid_argument("pool_size < 1");
    for (int i = 0; i < pool_size; ++i) {
        workers.emplace_back(std::make_shared<Worker>(*this));
        workers_map.emplace(workers.back()->thread.get_id(), workers.back());
    }
    for (auto &w : workers)
        w->start();
}

WorkStealingPool::~WorkStealingPool() {
    shutdown = true;
    for (auto &w : workers) {
        w->cond.notify_all();
        w->thread.join();
    }
}

#endif //SPM_DIVIDE_AND_CONQUER_WORKSTEALINGPOOL_H
