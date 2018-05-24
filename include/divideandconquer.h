//
// Created by Giorgio Vinciguerra on 23/05/2018.
//

#ifndef SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H
#define SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H

#include <vector>
#include <functional>
#include <thread>
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
    DivideAndConquer(is_base_fun_type &is_base_fun,
                     divide_fun_type &divide_fun,
                     conquer_fun_type &conquer_fun,
                     combine_fun_type &combine_fun,
                     unsigned int parallelism_degree = std::thread::hardware_concurrency())
            : is_base(is_base_fun), divide(divide_fun), conquer(conquer_fun), combine(combine_fun),
              parallelism_degree(parallelism_degree), pool(parallelism_degree) {

    }

    TypeOut solve_sequential(TypeIn &problem) {
        if (is_base(problem))
            return conquer(problem);

        std::vector<TypeIn> subproblems = divide(problem); // FIXME: Copy
        std::vector<TypeOut> solutions;
        solutions.reserve(subproblems.size());
        for (auto &p : subproblems)
            solutions.push_back(solve_sequential(p));

        return combine(solutions);
    }

    std::future<TypeOut> solve(TypeIn &problem) {
        if (is_base(problem)) {
            auto task = std::packaged_task<TypeOut()>(
                    std::bind(conquer, problem)
            );
            task.operator()();
            return task.get_future();
        }

        auto future = pool.submit(std::this_thread::get_id(), [this, &problem]() {
            std::vector<TypeIn> subproblems = divide(problem);
            std::vector<TypeOut> solutions;
            for (auto &p : subproblems)
                solutions.push_back(solve_sequential(p));
            return combine(solutions);
        });

        return future;
    }
};

#endif //SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H
