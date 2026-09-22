#ifndef EVALUATE_H
#define EVALUATE_H

#include "csv_loader.h"
#include "graph.h"

#include <vector>

namespace orienteering {

// 染色体 = 選択パート（N bit）+ 順序パート（MAX_CONTROLS 個の順列）
using Chromosome = std::vector<int>;

// 4目的関数の値
struct Objectives {
    double f_map;
    double f_dist;
    double f_time;
    double f_route;
};

// デコード結果
struct DecodedCourse {
    std::vector<long long> course_nodes;       // [gate, c1, c2, ..., gate]
    std::vector<int>       selected_indices;   // landmarks のインデックス（巡回順）
    bool                   is_valid;
};

// 評価結果（GAで使う総合情報）
struct EvalResult {
    double         fitness;
    Objectives     objectives;
    bool           is_valid;
    double         total_distance;
    double         total_gain;
    DecodedCourse  decoded;
};

// ============================================================
// デコード関数
// ============================================================
DecodedCourse decode(
    const Chromosome&            chromosome,
    const std::vector<Landmark>& landmarks,
    long long                    gate_node);

// ============================================================
// 4つの目的関数
// ============================================================
double f_map(int n_controls);

double f_dist(
    const std::vector<int>&      selected_indices,
    const std::vector<Landmark>& landmarks);

double f_time(double total_distance, double total_gain);

double f_route(double total_gain);

// ============================================================
// 単目的化（重み付き和）
// ============================================================
double calc_fitness(const Objectives& obj);

// ============================================================
// まとめて評価する関数（GA から呼ばれる）
// ============================================================
EvalResult evaluate(
    const Chromosome&            chromosome,
    const std::vector<Landmark>& landmarks,
    const PathCache&             path_cache,
    long long                    gate_node);

} // namespace orienteering

#endif // EVALUATE_H
