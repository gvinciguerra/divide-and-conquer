//
// Created by Giorgio Vinciguerra on 23/05/2018.
//

#ifndef SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H
#define SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H

#include <vector>
#include <functional>
#include <thread>

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
    unsigned int parallelism_degree;

public:
    DivideAndConquer(is_base_fun_type &is_base_fun,
                     divide_fun_type &divide_fun,
                     conquer_fun_type &conquer_fun,
                     combine_fun_type &combine_fun)
            : is_base(is_base_fun), divide(divide_fun), conquer(conquer_fun), combine(combine_fun) {
        parallelism_degree = std::thread::hardware_concurrency();
    }

    TypeOut solve(TypeIn &problem) {
        if (is_base(problem))
            return conquer(problem);

        std::vector<TypeIn> subproblems = divide(problem); // FIXME: Copy
        std::vector<TypeOut> solutions;
        solutions.reserve(subproblems.size());

        for (auto p : subproblems)
            solutions.push_back(solve(p));

        return combine(solutions);
    }

    void set_parallelism_degree(unsigned int degree) {
        parallelism_degree = degree;
    }
};

#endif //SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H
