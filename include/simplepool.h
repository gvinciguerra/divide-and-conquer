//
// Created by Giorgio Vinciguerra on 26/05/2018.
//

#ifndef SPM_DIVIDE_AND_CONQUER_SIMPLEPOOL_H
#define SPM_DIVIDE_AND_CONQUER_SIMPLEPOOL_H

#include <vector>
#include <future>
#include <condition_variable>
#include <queue>

class SimplePool {
    std::atomic_bool shutdown;
    std::mutex mutex;
    std::condition_variable cond;
    std::vector<std::thread> workers;
    std::deque<std::function<void()>> queue;

public:
    explicit SimplePool(size_t pool_size);

    template<class F, class... Args>
    std::future<typename std::result_of<F(Args...)>::type> submit(F &&f, Args &&... args);

    ~SimplePool();
};

template<class F, class... Args>
std::future<typename std::result_of<F(Args...)>::type> SimplePool::submit(F &&f, Args &&... args) {
    using result_type = typename std::result_of<F(Args...)>::type;

    auto task = std::make_shared<std::packaged_task<result_type()> >(
            std::bind(std::forward<F>(f), std::forward<Args>(args)...)
    );

    std::unique_lock<std::mutex> lock(mutex);
    queue.emplace_front([task]() { (*task)(); });
    cond.notify_one();
    return task->get_future();
}

SimplePool::SimplePool(size_t pool_size) : shutdown(false) {
    if (pool_size < 1)
        throw std::invalid_argument("pool_size < 1");
    for (int i = 0; i < pool_size; ++i) {
        workers.emplace_back(std::thread([this]() -> void {
            while (true) {
                std::unique_lock<std::mutex> lock(this->mutex);
                this->cond.wait(lock, [this] { return shutdown || !this->queue.empty(); });
                if (shutdown && this->queue.empty())
                    return;

                auto task = std::move(this->queue.back());
                this->queue.pop_back();
                lock.unlock();
                task();
            }
        }));
    }
}

SimplePool::~SimplePool() {
    shutdown = true;
    cond.notify_all();
    for (auto &w : workers)
        w.join();
}

#endif //SPM_DIVIDE_AND_CONQUER_SIMPLEPOOL_H
