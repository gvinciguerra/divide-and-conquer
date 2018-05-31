//
// Created by Giorgio Vinciguerra on 23/05/2018.
//

#ifndef SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H
#define SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H

#include <vector>
#include <functional>
#include <thread>
#include "simplepool.h"
#include "workstealingpool.h"

template<typename TypeIn, typename TypeOut>
class DivideAndConquer {
public:
    using is_base_fun_type = std::function<bool(TypeIn &problem)>;
    using divide_fun_type = std::function<std::vector<TypeIn>(TypeIn &problem)>;
    using conquer_fun_type = std::function<TypeOut(TypeIn &problem)>;
    using combine_fun_type = std::function<TypeOut(std::vector<TypeOut> &solutions)>;

private:
    const is_base_fun_type &is_base;
    const divide_fun_type &divide;
    const conquer_fun_type &conquer;
    const combine_fun_type &combine;
    const size_t parallelism_degree;
    WorkStealingPool pool;

public:
    DivideAndConquer(const is_base_fun_type &is_base_fun,
                     const divide_fun_type &divide_fun,
                     const conquer_fun_type &conquer_fun,
                     const combine_fun_type &combine_fun,
                     unsigned int parallelism_degree = std::thread::hardware_concurrency())
            : is_base(is_base_fun), divide(divide_fun), conquer(conquer_fun), combine(combine_fun),
              parallelism_degree(parallelism_degree), pool(parallelism_degree) {
        if (parallelism_degree < 1)
            throw std::invalid_argument("parallelism_degree < 1");
    }

    TypeOut solve_sequential(TypeIn &problem) {
        if (is_base(problem))
            return conquer(problem);

        std::vector<TypeIn> subproblems = divide(problem);
        std::vector<TypeOut> solutions;
        solutions.reserve(subproblems.size());
        for (auto &p : subproblems)
            solutions.push_back(solve_sequential(p));

        return combine(solutions);
    }

    TypeOut solve_helper(TypeIn &problem, size_t remaining_workers) {
        if (is_base(problem))
            return conquer(problem);
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

    std::future<TypeOut> solve(TypeIn &problem) {
        return std::move(std::async([this, &problem]() { return solve_helper(problem, parallelism_degree); }));
    }
};

#endif //SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H
