//
// Created by Giorgio Vinciguerra on 23/05/2018.
//

#ifndef SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H
#define SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H

#include <vector>
#include <functional>
#include <thread>
#include <ff/lb.hpp>
#include <ff/node.hpp>
#include <ff/farm.hpp>
#include <ff/barrier.hpp>
#include <iostream>
#include "simplepool.h"
#include "workstealingpool.h"

template<typename TypeIn, typename TypeOut, bool FastFlowBackend = false>
class DivideAndConquer;

template<typename TypeIn, typename TypeOut>
class DivideAndConquer<TypeIn, TypeOut, false> {
public:
    using is_base_fun_type = std::function<bool(TypeIn &problem)>;
    using divide_fun_type = std::function<std::vector<TypeIn>(TypeIn &problem)>;
    using base_fun_type = std::function<TypeOut(TypeIn &problem)>;
    using combine_fun_type = std::function<TypeOut(std::vector<TypeOut> &solutions)>;

private:
    const is_base_fun_type &is_base;
    const divide_fun_type &divide;
    const base_fun_type &base;
    const combine_fun_type &combine;
    const size_t parallelism_degree;
    WorkStealingPool pool;

    TypeOut solve_helper(TypeIn &problem, size_t remaining_workers) {
        if (is_base(problem))
            return base(problem);
        if (remaining_workers <= 1)
            return solve_sequential(problem);

        std::vector<TypeIn> subproblems = divide(problem);
        std::vector<std::future<TypeOut>> futures;
        size_t child_workers = remaining_workers / subproblems.size();
        for (auto it = subproblems.begin() + 1; it != subproblems.end(); ++it)
            futures.emplace_back(pool.submit(&DivideAndConquer::solve_helper, this, *it, child_workers));

        std::vector<TypeOut> solutions = {solve_helper(subproblems[0], child_workers)};
        for (auto &f : futures)
            solutions.emplace_back(f.get());

        return combine(solutions);
    }

public:
    DivideAndConquer(const is_base_fun_type &is_base_fun,
                     const divide_fun_type &divide_fun,
                     const base_fun_type &base_fun,
                     const combine_fun_type &combine_fun,
                     unsigned int parallelism_degree = std::thread::hardware_concurrency())
            : is_base(is_base_fun), divide(divide_fun), base(base_fun), combine(combine_fun),
              parallelism_degree(parallelism_degree), pool(parallelism_degree) {
        if (parallelism_degree < 1)
            throw std::invalid_argument("parallelism_degree < 1");
    }

    TypeOut solve_sequential(TypeIn &problem) {
        if (is_base(problem))
            return base(problem);

        std::vector<TypeIn> subproblems = divide(problem);
        std::vector<TypeOut> solutions;
        solutions.reserve(subproblems.size());
        for (auto &p : subproblems)
            solutions.push_back(solve_sequential(p));

        return combine(solutions);
    }

    std::future<TypeOut> solve(TypeIn &problem) {
        return std::move(pool.submit(&DivideAndConquer::solve_helper, this, problem, parallelism_degree));
    }
};

template<typename TypeIn, typename TypeOut>
class DivideAndConquer<TypeIn, TypeOut, true> {
public:
    using is_base_fun_type = std::function<bool(TypeIn &problem)>;
    using divide_fun_type = std::function<std::vector<TypeIn>(TypeIn &problem)>;
    using base_fun_type = std::function<TypeOut(TypeIn &problem)>;
    using combine_fun_type = std::function<TypeOut(std::vector<TypeOut> &solutions)>;

private:
    using job_type = struct job {
        TypeIn *problem;
        size_t workers;
        ff::Barrier *barrier;
        std::vector<TypeOut> *out_vector;
        size_t out_index;
    };

    const is_base_fun_type &is_base;
    const divide_fun_type &divide;
    const base_fun_type &base;
    const combine_fun_type &combine;
    const size_t parallelism_degree;
    std::unique_ptr<ff::ff_Farm<job_type, void>> farm;

    struct Worker : ff::ff_node_t<job_type, void> {
        DivideAndConquer *dc;

    public:
        explicit Worker(DivideAndConquer *dc) : dc(dc) {}

        void *svc(job_type *job) {
            auto result = solve_helper(*(job->problem), job->workers);
            auto s = job->out_vector->size();
            job->out_vector->operator[](job->out_index) = result;
            if (job->barrier)
                job->barrier->doBarrier(ff::ff_node::get_my_id());
            else
                dc->farm->offload(EOS);

            delete job;
            return GO_ON;
        }

        TypeOut solve_helper(TypeIn &problem, size_t remaining_workers) {
            if (dc->is_base(problem))
                return dc->base(problem);
            if (remaining_workers <= 1)
                return dc->solve_sequential(problem);

            auto subproblems = dc->divide(problem);
            auto barrier = new ff::Barrier();
            auto solutions = new std::vector<TypeOut>(subproblems.size());
            size_t child_workers = remaining_workers / subproblems.size();
            barrier->barrierSetup(subproblems.size());

            size_t out_index = 1;
            for (auto it = subproblems.begin() + 1; it != subproblems.end(); ++it) {
                auto job = new job_type({&*it, child_workers, barrier, solutions, out_index++});
                dc->farm->offload(job);
            }

            auto r = solve_helper(subproblems[0], child_workers);
            solutions->operator[](0) = r;
            barrier->doBarrier(ff::ff_node::get_my_id());

            auto combined = dc->combine(*solutions);
            delete barrier;
            delete solutions;
            return combined;
        }

    };


public:
    DivideAndConquer(const is_base_fun_type &is_base_fun,
                     const divide_fun_type &divide_fun,
                     const base_fun_type &base_fun,
                     const combine_fun_type &combine_fun,
                     unsigned int parallelism_degree = std::thread::hardware_concurrency())
            : is_base(is_base_fun), divide(divide_fun), base(base_fun), combine(combine_fun),
              parallelism_degree(parallelism_degree) {
        if (parallelism_degree < 1)
            throw std::invalid_argument("parallelism_degree < 1");

        std::vector<std::unique_ptr<ff::ff_node>> workers;
        for (size_t i = 0; i < parallelism_degree; ++i)
            workers.push_back(std::make_unique<Worker>(this));

        farm = std::make_unique<ff::ff_Farm<job_type, void>>(std::move(workers), true);
        farm->remove_collector();
    }

    TypeOut solve_sequential(TypeIn &problem) {
        if (is_base(problem))
            return base(problem);

        std::vector<TypeIn> subproblems = divide(problem);
        std::vector<TypeOut> solutions;
        for (auto &p : subproblems)
            solutions.push_back(solve_sequential(p));

        return combine(solutions);
    }

    std::future<TypeOut> solve(TypeIn &problem) {
        std::packaged_task<TypeOut()> task([this, &problem] {
            auto solutions = new std::vector<TypeOut>(1);
            farm->run();
            farm->stop();
            farm->offload(new job_type({&problem, parallelism_degree, NULL, solutions, 0}));
            farm->wait();
            auto result = solutions->operator[](0);
            delete solutions;
            return result;
        });
        task();
        return std::move(task.get_future());
    }
};

#endif //SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H
