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

using namespace std::chrono_literals;


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
    std::future<typename std::result_of<F(Args...)>::type> submit(std::thread::id thread_id, F &&f, Args &&... args);

    std::function<void()> steal();

    ~WorkStealingPool();

private:
    std::atomic_bool shutdown;
    size_t pool_size;
    std::vector<std::shared_ptr<Worker>> workers; // O(1) random worker selection in steal()
    std::unordered_map<std::thread::id, std::shared_ptr<Worker>> workers_map; // O(1) access in submit()
};

template<class F, class... Args>
std::future<typename std::result_of<F(Args...)>::type> WorkStealingPool::Worker::submit(F &&f, Args &&... args) {
    using result_type = typename std::result_of<F(Args...)>::type;
    auto task = std::make_shared<std::packaged_task<result_type()> >(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    std::unique_lock<std::mutex> lock(mutex);
    queue.emplace_back([task]() { (*task)(); });
    lock.unlock();
    cond.notify_all();

    return task->get_future();
}

WorkStealingPool::Worker::Worker(WorkStealingPool &pool) : pool(pool) {

}

void WorkStealingPool::Worker::start() {
    thread = std::thread([this]() -> void {
        while (true) {
            std::unique_lock<std::mutex> lock(this->mutex);
            this->cond.wait_for(lock, 100ms, [this] { return pool.shutdown || this->queue.empty(); });
            if (pool.shutdown && this->queue.empty())
                return;
            if (this->queue.empty()) {
                lock.unlock();
                auto task = std::move(pool.steal());
                if (task != nullptr)
                    task();
            } else {
                auto task = std::move(this->queue.back());
                this->queue.pop_back();
                lock.unlock();
                task();
            }
        }
    });
}

template<class F, class... Args>
std::future<typename std::result_of<F(Args...)>::type>
WorkStealingPool::submit(std::thread::id thread_id, F &&f, Args &&... args) {
    if (workers_map.find(thread_id) == workers_map.end())
        return workers[std::rand() % workers.size()]->submit(f, args...);
    return workers_map[thread_id]->submit(f, args...);
}

std::function<void()> WorkStealingPool::steal() {
    auto r = std::rand() / ((RAND_MAX + 1u) / workers.size());
    auto victim = workers[r];
    std::unique_lock<std::mutex> lock(victim->mutex, std::try_to_lock);
    if (lock.owns_lock() && !victim->queue.empty()) {
        auto task = std::move(victim->queue.front());
        victim->queue.pop_front();
        return task;
    }
    return nullptr;
}

WorkStealingPool::WorkStealingPool(size_t pool_size) : pool_size(pool_size), shutdown() {
    for (int i = 0; i < pool_size; ++i) {
        workers.emplace_back(std::make_shared<Worker>(*this));
        workers_map.emplace(workers.back()->thread.get_id(), workers.back());
    }
    for (auto w : workers)
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
