#include <iostream>
#include <vector>
#include <random>
#include "include/divideandconquer.h"
#include <benchmark/benchmark.h>

#define data 1000
std::vector<int> array(data * data);
std::vector<std::vector<int>> matrix_a(data, std::vector<int>(data));
std::vector<std::vector<int>> matrix_b(data, std::vector<int>(data));

void BM_merge_sort(benchmark::State &state);

void BM_matrix_multiply(benchmark::State &state);

int main(int argc, char **argv) {
    std::uniform_int_distribution<int> dist(0, array.size() * 2);
    std::srand(0); // reproducibility
    std::generate(array.begin(), array.end(), std::rand);
    for (auto &r : matrix_a)
        std::generate(r.begin(), r.end(), std::rand);
    for (auto &r : matrix_b)
        std::generate(r.begin(), r.end(), std::rand);

    BENCHMARK(BM_merge_sort)->DenseRange(1, std::thread::hardware_concurrency())->Iterations(1)->UseManualTime();
    BENCHMARK(BM_matrix_multiply)->DenseRange(1, std::thread::hardware_concurrency())->Iterations(1)->UseManualTime();

    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    return 0;
}

void BM_merge_sort(benchmark::State &state) {
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

    unsigned int par_degree = state.range(0);

    std::vector<int> copy = array;
    bounds_type problem = {copy.begin(), copy.end()};
    DivideAndConquer<bounds_type, bounds_type> divideAndConquer(is_base, divide, conquer, combine, par_degree);

    using namespace std::chrono;

    for (auto _ : state) {
        auto t1 = high_resolution_clock::now();
        divideAndConquer.solve(problem).get();
        // assert(std::is_sorted(array.begin(), array.end()));
        auto t2 = high_resolution_clock::now();
        auto time_span = duration_cast<duration<double>>(t2 - t1);
        state.SetIterationTime(time_span.count());
    }
}

void BM_matrix_multiply(benchmark::State &state) {
    using submatrix_type = struct {
        size_t ra, rb, ca, cb;
    };
    using dc_type = DivideAndConquer<submatrix_type, submatrix_type>;

    std::vector<std::vector<int>> result(data, std::vector<int>(data));

    dc_type::is_base_fun_type is_base = [](submatrix_type &p) -> bool {
        return p.rb - p.ra < 64;
    };

    dc_type::divide_fun_type divide = [](submatrix_type &p) -> std::vector<submatrix_type> {
        std::vector<submatrix_type> subproblems;
        auto half = (p.rb - p.ra) / 2;
        subproblems.push_back(submatrix_type{p.ra, p.ra + half, p.ca, p.ca + half});
        subproblems.push_back(submatrix_type{p.ra, p.ra + half, p.ca + half, p.cb});
        subproblems.push_back(submatrix_type{p.ra + half, p.rb, p.ca, p.ca + half});
        subproblems.push_back(submatrix_type{p.ra + half, p.rb, p.ca + half, p.cb});
        return subproblems;
    };

    dc_type::conquer_fun_type conquer = [&result](submatrix_type &p) -> submatrix_type {
        for (size_t i = p.ra; i < p.rb; ++i)
            for (size_t j = p.ca; j < p.cb; ++j)
                for (size_t k = p.ra; k < p.rb; ++k)
                    result[i][j] += matrix_a[i][k] * matrix_b[k][j];
        return p;
    };

    dc_type::combine_fun_type combine = [](std::vector<submatrix_type> &solutions) -> submatrix_type {
        return solutions[0];
    };

    unsigned int par_degree = state.range(0);

    std::vector<int> copy = array;
    submatrix_type problem = {0, result.size(), 0, result[0].size()};
    DivideAndConquer<submatrix_type, submatrix_type> divideAndConquer(is_base, divide, conquer, combine, par_degree);

    using namespace std::chrono;

    for (auto _ : state) {
        auto t1 = high_resolution_clock::now();
        divideAndConquer.solve(problem).get();
        auto t2 = high_resolution_clock::now();
        auto time_span = duration_cast<duration<double>>(t2 - t1);
        state.SetIterationTime(time_span.count());
    }
}