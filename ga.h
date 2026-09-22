#ifndef GA_H
#define GA_H

#include "csv_loader.h"
#include "graph.h"
#include "evaluate.h"

#include <random>
#include <utility>
#include <vector>

namespace orienteering {

using RNG = std::mt19937;

// ============================================================
// GA 操作（個体生成・選択・交叉・突然変異）
// ============================================================

// ランダムな染色体を生成（初期個体群用）
Chromosome create_random_chromosome(int N, RNG& rng);

// 【改良②】farthest-point で「散らばった8地点」の染色体を生成（初期個体群用）
Chromosome create_greedy_chromosome(int N, const std::vector<Landmark>& landmarks, RNG& rng);

// トーナメント選択
const Chromosome& tournament_select(
    const std::vector<Chromosome>& population,
    const std::vector<double>&     fitnesses,
    RNG&                           rng);

// 交叉（選択パート：一様交叉 / 順序パート：OX＝順序交叉）
std::pair<Chromosome, Chromosome> crossover(
    const Chromosome& parent1,
    const Chromosome& parent2,
    int               N,
    RNG&              rng);

// 突然変異（選択パート：ビット反転 / 順序パート：2点スワップ）
void mutate(Chromosome& chromosome, int N, RNG& rng);

// 【改良①】選択数を Q_TARGET(=8) に修復（交叉・突然変異の後に呼ぶ）
void repair_selection(Chromosome& chromosome, int N, RNG& rng);

// 【改良③】局所探索（1地点を未選択候補と交換する山登り。上位個体の仕上げ）
void local_search(Chromosome& chromosome, const std::vector<Landmark>& landmarks,
                  const PathCache& path_cache, long long gate_node);

// ============================================================
// GA メインループ
// ============================================================

struct GAResult {
    Chromosome           best_chromosome;
    EvalResult           best_eval;
    std::vector<double>  best_fitness_history;
};

GAResult run_ga(
    const std::vector<Landmark>& landmarks,
    const PathCache&             path_cache,
    long long                    gate_node,
    RNG&                         rng);

// 【改良④】多スタート：GAを n_restarts 回まわし一番良い結果を返す（決定的）
GAResult run_ga_best(
    const std::vector<Landmark>& landmarks,
    const PathCache&             path_cache,
    long long                    gate_node,
    unsigned int                 base_seed,
    int                          n_restarts);

} // namespace orienteering

#endif // GA_H
