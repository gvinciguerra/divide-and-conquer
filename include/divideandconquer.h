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
    using is_base_fun_type = std::function<bool(TypeIn &problem)>;
    using divide_fun_type = std::function<std::vector<TypeIn>(TypeIn &problem)>;
    using conquer_fun_type = std::function<TypeOut(TypeIn &problem)>;
    using combine_fun_type = std::function<TypeOut(std::vector<TypeOut> solutions)>;

    constexpr is_base_fun_type &is_base;
    constexpr divide_fun_type &divide;
    constexpr conquer_fun_type &conquer;
    constexpr combine_fun_type &combine;
    unsigned int parallelism_degree;
public:

    DivideAndConquer(is_base_fun_type &is_base_fun,
                     divide_fun_type &divide_fun,
                     conquer_fun_type &conquer_fun,
                     combine_fun_type &combine_fun) : is_base(is_base_fun), divide(divide_fun), conquer(conquer_fun),
                                                      combine(combine_fun) {
        parallelism_degree = std::thread::hardware_concurrency();
    }

    void set_parallelism_degree(unsigned int degree) {
        parallelism_degree = degree;
    }
};

#endif //SPM_DIVIDE_AND_CONQUER_DIVIDEANDCONQUER_H
