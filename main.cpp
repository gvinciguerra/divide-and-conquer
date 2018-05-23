#include <iostream>
#include <vector>
#include <random>
#include "include/divideandconquer.h"

int main() {
    std::vector<int> array(100000);
    std::uniform_int_distribution<int> dist(0, array.size() * 2);
    std::srand(0);
    std::generate(array.begin(), array.end(), std::rand);

    using bounds_type = std::pair<std::vector<int>::iterator, std::vector<int>::iterator>;
    using dc_type = DivideAndConquer<bounds_type, bounds_type>;

    dc_type::is_base_fun_type is_base = [](bounds_type &problem) -> bool {
        return std::distance(problem.first, problem.second) < 64;
    };

    dc_type::divide_fun_type divide = [](bounds_type &problem) -> std::vector<bounds_type> {
        auto mid = problem.first + std::distance(problem.first, problem.second) / 2;
        std::vector<bounds_type> subproblems;
        subproblems.reserve(2);
        subproblems.emplace_back(problem.first, mid);
        subproblems.emplace_back(mid, problem.second);
        return subproblems;
    };

    dc_type::conquer_fun_type conquer = [](bounds_type &problem) -> bounds_type {
        std::sort(problem.first, problem.second);
        return problem;
    };

    dc_type::combine_fun_type combine = [](std::vector<bounds_type> &solutions) -> bounds_type {
        std::inplace_merge(solutions[0].first, solutions[0].second, solutions[1].second);
        return std::make_pair(solutions[0].first, solutions[1].second);
    };

    bounds_type problem = {array.begin(), array.end()};
    DivideAndConquer<bounds_type, bounds_type> divideAndConquer(is_base, divide, conquer, combine);
    divideAndConquer.solve(problem);
    assert(std::is_sorted(array.begin(), array.end()));

    std::cout << "Hello, World!" << std::endl;
    return 0;
}