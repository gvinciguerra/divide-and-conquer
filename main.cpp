#include <iostream>
#include <vector>
#include <random>
#include "include/divideandconquer.h"
#include <benchmark/benchmark.h>

#define DATA_SQRT 2048 // 2048^2 elements, must be a power of two
#define CUTOFF_SQRT 64 // 64^2 elements to stop the recursive splitting

using namespace std::chrono;

std::vector<int> array(DATA_SQRT * DATA_SQRT);
matrix_type matrix_a(DATA_SQRT, std::vector<int>(DATA_SQRT));
matrix_type matrix_b(DATA_SQRT, std::vector<int>(DATA_SQRT));

void BM_merge_sort(benchmark::State &state);

void BM_merge_sort_openmp(benchmark::State &state);

void BM_matrix_multiply(benchmark::State &state);

void BM_merge_sort_sequential(benchmark::State &state);

void BM_matrix_multiply_sequential(benchmark::State &state);

int main(int argc, char **argv) {
    std::uniform_int_distribution<int> dist(0, array.size() * 2);
    std::srand(0); // reproducibility
    std::generate(array.begin(), array.end(), std::rand);
    for (auto &r : matrix_a)
        std::generate(r.begin(), r.end(), std::rand);
    for (auto &r : matrix_b)
        std::generate(r.begin(), r.end(), std::rand);

    BENCHMARK(BM_merge_sort_sequential)->Iterations(1)->UseManualTime();
    BENCHMARK(BM_matrix_multiply_sequential)->Iterations(1)->UseManualTime();
    BENCHMARK(BM_merge_sort)->DenseRange(1, std::thread::hardware_concurrency())->Iterations(1)->UseManualTime();
    BENCHMARK(BM_matrix_multiply)->DenseRange(1, std::thread::hardware_concurrency())->Iterations(1)->UseManualTime();
    BENCHMARK(BM_merge_sort_openmp)->DenseRange(1, std::thread::hardware_concurrency())->Iterations(1)->UseManualTime();

    benchmark::Initialize(&argc, argv);
    benchmark::RunSpecifiedBenchmarks();
    return 0;
}

/*
 * MERGE SORT BENCHMARKS
 */

void BM_merge_sort(benchmark::State &state) {
    using bounds_type = std::pair<std::vector<int>::iterator, std::vector<int>::iterator>;
    using merge_sort_dc_type = DivideAndConquer<bounds_type, bounds_type>;

    merge_sort_dc_type::is_base_fun_type is_base = [](bounds_type &problem) -> bool {
        return std::distance(problem.first, problem.second) < CUTOFF_SQRT * CUTOFF_SQRT;
    };

    merge_sort_dc_type::divide_fun_type divide = [](bounds_type &problem) -> std::vector<bounds_type> {
        auto mid = problem.first + std::distance(problem.first, problem.second) / 2;
        std::vector<bounds_type> subproblems;
        subproblems.reserve(2);
        subproblems.emplace_back(problem.first, mid);
        subproblems.emplace_back(mid, problem.second);
        return subproblems;
    };

    merge_sort_dc_type::conquer_fun_type conquer = [](bounds_type &problem) -> bounds_type {
        std::sort(problem.first, problem.second);
        return problem;
    };

    merge_sort_dc_type::combine_fun_type combine = [](std::vector<bounds_type> &solutions) -> bounds_type {
        std::inplace_merge(solutions[0].first, solutions[0].second, solutions[1].second);
        return std::make_pair(solutions[0].first, solutions[1].second);
    };

    unsigned int par_degree = state.range(0);
    std::vector<int> copy = array;
    bounds_type problem = {copy.begin(), copy.end()};
    DivideAndConquer<bounds_type, bounds_type> divideAndConquer(is_base, divide, conquer, combine, par_degree);

    for (auto _ : state) {
        auto t1 = high_resolution_clock::now();
        divideAndConquer.solve(problem).get();
        // assert(std::is_sorted(array.begin(), array.end()));
        auto t2 = high_resolution_clock::now();
        auto time_span = duration_cast<duration<double>>(t2 - t1);
        state.SetIterationTime(time_span.count());
    }
}

template<class Iter>
void merge_sort(const Iter &first, const Iter &last) {
    if (last - first > 1) {
        Iter middle = first + (last - first) / 2;
        merge_sort(first, middle);
        merge_sort(middle, last);
        std::inplace_merge(first, middle, last);
    }
}

void BM_merge_sort_sequential(benchmark::State &state) {
    std::vector<int> copy = array;
    for (auto _ : state) {
        auto t1 = high_resolution_clock::now();
        merge_sort(copy.begin(), copy.end());
        auto t2 = high_resolution_clock::now();
        auto time_span = duration_cast<duration<double>>(t2 - t1);
        state.SetIterationTime(time_span.count());
    }
}

template<class Iter>
void merge_sort_openmp(Iter first, Iter last) {
    if (last - first > 1) {
        Iter middle = first + (last - first) / 2;
#pragma omp task
        merge_sort(first, middle);
#pragma omp task
        merge_sort(middle, last);
#pragma omp taskwait
        std::inplace_merge(first, middle, last);
    }
}

void BM_merge_sort_openmp(benchmark::State &state) {
    std::vector<int> copy = array;
    unsigned int par_degree = state.range(0);
    for (auto _ : state) {
        auto t1 = high_resolution_clock::now();
#pragma omp parallel num_threads(par_degree)
#pragma omp single
        merge_sort_openmp(copy.begin(), copy.end());
        auto t2 = high_resolution_clock::now();
        auto time_span = duration_cast<duration<double>>(t2 - t1);
        state.SetIterationTime(time_span.count());
    }
}

/*
 * MATRIX MULTIPLICATION BENCHMARKS
 */

void sum_square_matrices(matrix_type &c, size_t base_r, size_t base_c, matrix_type &a, matrix_type &b) {
    assert(a.size() == b.size() && b.size() * 2 == c.size());
    for (size_t i = 0; i < a.size(); ++i)
        for (size_t j = 0; j < a.size(); ++j)
            c[i + base_r][j + base_c] = a[i][j] + b[i][j];
}

void BM_matrix_multiply(benchmark::State &state) {
    auto n = matrix_a.size();
    assert(n == (n & (n - 1)) == 0); // n must be a power of two, because the algorithm doesn't do row/col padding

    using slice_type = struct {
        size_t ra, rb, ca, cb;
    };
    using pair_of_slice = std::pair<slice_type, slice_type>;
    using matrix_mult_dc_type = DivideAndConquer<pair_of_slice, std::shared_ptr<matrix_type>>;

    std::vector<std::vector<int>> result(DATA_SQRT, std::vector<int>(DATA_SQRT));

    matrix_mult_dc_type::is_base_fun_type is_base = [](pair_of_slice &p) -> bool {
        return p.first.rb - p.first.ra < CUTOFF_SQRT;
    };

    matrix_mult_dc_type::divide_fun_type divide = [](pair_of_slice &problem) {
        std::vector<pair_of_slice> subproblems;
        auto p = problem.first;
        auto half = (p.rb + p.ra) / 2;
        auto flah = (p.rb + p.ra) / 2 + 1;
        auto i11 = slice_type{p.ra, half, p.ca, half};
        auto i12 = slice_type{p.ra, half, flah, p.cb};
        auto i21 = slice_type{flah, p.rb, p.ca, half};
        auto i22 = slice_type{flah, p.rb, flah, p.cb};
        subproblems.emplace_back(i11, i11);
        subproblems.emplace_back(i12, i21);
        subproblems.emplace_back(i11, i12);
        subproblems.emplace_back(i12, i22);
        subproblems.emplace_back(i21, i11);
        subproblems.emplace_back(i22, i21);
        subproblems.emplace_back(i21, i12);
        subproblems.emplace_back(i22, i22);
        return subproblems;
    };

    matrix_mult_dc_type::conquer_fun_type conquer = [&result](pair_of_slice &p) {
        size_t n = p.first.rb - p.first.ra;
        auto c = std::make_shared<std::vector<std::vector<int>>>(n, std::vector<int>(n));
        for (size_t i = 0; i < n; ++i)
            for (size_t j = 0; j < n; ++j)
                for (size_t k = 0; k < n; ++k)
                    (*c)[i][j] += matrix_a[p.first.ra + i][p.first.ca + k] * matrix_b[p.second.ra + k][p.second.ca + j];
        return c;
    };

    matrix_mult_dc_type::combine_fun_type combine = [](std::vector<std::shared_ptr<matrix_type>> &solutions) {
        assert(solutions.size() == 8);
        size_t n = solutions[0]->size();
        auto c = std::make_shared<std::vector<std::vector<int>>>(n * 2, std::vector<int>(n * 2));
        sum_square_matrices(*c, 0, 0, *(solutions[0]), *(solutions[1]));         // C11
        sum_square_matrices(*c, 0, n - 1, *(solutions[2]), *(solutions[3]));     // C12
        sum_square_matrices(*c, n - 1, 0, *(solutions[4]), *(solutions[5]));     // C21
        sum_square_matrices(*c, n - 1, n - 1, *(solutions[6]), *(solutions[7])); // C22
        return c;
    };

    unsigned int par_degree = state.range(0);
    slice_type p = {0, result.size() - 1, 0, result[0].size() - 1};
    pair_of_slice problem = std::make_pair(p, p);
    matrix_mult_dc_type divideAndConquer(is_base, divide, conquer, combine, par_degree);

    for (auto _ : state) {
        auto t1 = high_resolution_clock::now();
        divideAndConquer.solve(problem).get();
        auto t2 = high_resolution_clock::now();
        auto time_span = duration_cast<duration<double>>(t2 - t1);
        state.SetIterationTime(time_span.count());
    }
}

void BM_matrix_multiply_sequential(benchmark::State &state) {
    std::vector<std::vector<int>> result(DATA_SQRT, std::vector<int>(DATA_SQRT));
    for (auto _ : state) {
        auto t1 = high_resolution_clock::now();
        for (size_t i = 0; i < DATA_SQRT; ++i)
            for (size_t j = 0; j < DATA_SQRT; ++j)
                for (size_t k = 0; k < DATA_SQRT; ++k)
                    result[i][j] += matrix_a[i][k] * matrix_b[k][j];
        auto t2 = high_resolution_clock::now();
        auto time_span = duration_cast<duration<double>>(t2 - t1);
        state.SetIterationTime(time_span.count());
    }
}