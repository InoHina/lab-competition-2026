#include "ga.h"
#include "const.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>

namespace orienteering {

// ============================================================
// ランダムな染色体を生成
// ============================================================
Chromosome create_random_chromosome(int N, RNG& rng) {
    // 【改良①】コントロール数を目標値 Q_TARGET(=8) に固定する。
    //   f_map は重み最大(0.50)で目標が8個と明確なので、「何個選ぶか」で
    //   迷わせず、探索を「どの8個を・どの順で回るか」に集中させる。
    int n_select = Q_TARGET;

    Chromosome chrom(N + MAX_CONTROLS, 0);

    // 選択パート：N個の候補からランダムに n_select 個を選ぶ
    std::vector<int> indices(N);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng);
    for (int i = 0; i < n_select; ++i) {
        chrom[indices[i]] = 1;
    }

    // 順序パート：0〜MAX_CONTROLS-1 のランダム順列
    std::vector<int> order(MAX_CONTROLS);
    std::iota(order.begin(), order.end(), 0);
    std::shuffle(order.begin(), order.end(), rng);
    for (int i = 0; i < MAX_CONTROLS; ++i) {
        chrom[N + i] = order[i];
    }
    return chrom;
}

// ============================================================
// 【改良②】2地点の直線距離(m)。f_dist と同じ緯度経度→メートル換算。
// ============================================================
static double landmark_dist(const Landmark& a, const Landmark& b) {
    double mean_lat = (a.lat + b.lat) / 2.0;
    double dlat = (a.lat - b.lat) * METERS_PER_DEGREE;
    double dlon = (a.lon - b.lon) * METERS_PER_DEGREE * std::cos(mean_lat * PI / 180.0);
    return std::sqrt(dlat * dlat + dlon * dlon);
}

// ============================================================
// 【改良②】farthest-point 法で「散らばった8地点」の染色体を作る
//   ランダムな1点から始め、毎回「既選択への最小距離が最大の候補」を追加する。
//   → 互いに離れた8地点になり、f_dist が最初から小さい個体を種にできる。
//   初期の1点を個体ごとにランダムに変えるので、多様な散らばり方が得られる。
// ============================================================
Chromosome create_greedy_chromosome(int N, const std::vector<Landmark>& landmarks, RNG& rng) {
    Chromosome chrom(N + MAX_CONTROLS, 0);

    std::vector<int> chosen;
    int first = std::uniform_int_distribution<int>(0, N - 1)(rng);  // 最初の1点はランダム
    chosen.push_back(first);
    chrom[first] = 1;

    // 残りを farthest-point で選ぶ（既選択から最も遠い候補を1つずつ追加）
    while (static_cast<int>(chosen.size()) < Q_TARGET) {
        int best = -1;
        double best_mindist = -1.0;
        for (int c = 0; c < N; ++c) {
            if (chrom[c] == 1) continue;
            double mind = std::numeric_limits<double>::max();
            for (int s : chosen) {
                double d = landmark_dist(landmarks[c], landmarks[s]);
                if (d < mind) mind = d;
            }
            if (mind > best_mindist) {   // 最小距離がいちばん大きい＝いちばん離れた点
                best_mindist = mind;
                best = c;
            }
        }
        chosen.push_back(best);
        chrom[best] = 1;
    }

    // 順序パートはランダム順列
    std::vector<int> order(MAX_CONTROLS);
    std::iota(order.begin(), order.end(), 0);
    std::shuffle(order.begin(), order.end(), rng);
    for (int i = 0; i < MAX_CONTROLS; ++i) chrom[N + i] = order[i];

    return chrom;
}

// ============================================================
// 【改良①】選択数の修復：選択ビットの「1」の個数を常に Q_TARGET(=8) に保つ
//   交叉・突然変異で1の数がずれるので、個体を作るたびに呼んで8個に揃える。
//   多すぎれば余分な1をランダムに0へ、少なすぎれば0をランダムに1へ。
//   これで f_map は常に0になり、探索が「選定と巡回順」に集中する。
// ============================================================
void repair_selection(Chromosome& chrom, int N, RNG& rng) {
    std::vector<int> ones, zeros;
    for (int i = 0; i < N; ++i) {
        if (chrom[i] == 1) ones.push_back(i);
        else               zeros.push_back(i);
    }
    // 多すぎる → ランダムに 1 を 0 へ
    while (static_cast<int>(ones.size()) > Q_TARGET) {
        int k = std::uniform_int_distribution<int>(0, static_cast<int>(ones.size()) - 1)(rng);
        chrom[ones[k]] = 0;
        ones.erase(ones.begin() + k);
    }
    // 少なすぎる → ランダムに 0 を 1 へ
    while (static_cast<int>(ones.size()) < Q_TARGET) {
        int k = std::uniform_int_distribution<int>(0, static_cast<int>(zeros.size()) - 1)(rng);
        chrom[zeros[k]] = 1;
        ones.push_back(zeros[k]);
        zeros.erase(zeros.begin() + k);
    }
}

// ============================================================
// トーナメント選択
// ============================================================
const Chromosome& tournament_select(
    const std::vector<Chromosome>& population,
    const std::vector<double>&     fitnesses,
    RNG&                           rng)
{
    std::uniform_int_distribution<int> dist(0, static_cast<int>(population.size()) - 1);
    int    best     = -1;
    double best_fit = std::numeric_limits<double>::max();
    for (int i = 0; i < TOURNAMENT_SIZE; ++i) {
        int idx = dist(rng);
        if (fitnesses[idx] <= best_fit) {
            best_fit = fitnesses[idx];
            best     = idx;
        }
    }
    return population[best];
}

// ============================================================
// 交叉
//   選択パート：一様交叉
//   順序パート：OX（順序交叉／順列を保存）
// ============================================================
std::pair<Chromosome, Chromosome> crossover(
    const Chromosome& parent1,
    const Chromosome& parent2,
    int               N,
    RNG&              rng)
{
    Chromosome c1(N + MAX_CONTROLS);
    Chromosome c2(N + MAX_CONTROLS);

    std::uniform_real_distribution<double> ureal(0.0, 1.0);

    // 選択パート：各ビットを 50% で入れ替え
    for (int i = 0; i < N; ++i) {
        if (ureal(rng) < 0.5) {
            c1[i] = parent1[i];
            c2[i] = parent2[i];
        } else {
            c1[i] = parent2[i];
            c2[i] = parent1[i];
        }
    }

    // 順序パート：OX（順序交叉）
    std::uniform_int_distribution<int> dist_cut(0, MAX_CONTROLS);
    int a = dist_cut(rng);
    int b = dist_cut(rng);
    if (a > b) std::swap(a, b);

    auto ox_fill = [&](Chromosome&       child,
                       const Chromosome& parent_donor,
                       const Chromosome& parent_filler) {
        std::vector<char> used(MAX_CONTROLS, 0);

        for (int i = a; i < b; ++i) {
            int v         = parent_donor[N + i];
            child[N + i]  = v;
            used[v]       = 1;
        }

        int pos = b % MAX_CONTROLS;
        for (int k = 0; k < MAX_CONTROLS; ++k) {
            int v = parent_filler[N + (b + k) % MAX_CONTROLS];
            if (used[v]) continue;
            child[N + pos] = v;
            used[v]        = 1;
            pos = (pos + 1) % MAX_CONTROLS;
        }
    };

    ox_fill(c1, parent1, parent2);
    ox_fill(c2, parent2, parent1);

    return {c1, c2};
}

// ============================================================
// 突然変異
//   選択パート：各ビットを PROB_BIT の確率で反転
//   順序パート：PROB_SWAP の確率で2点をスワップ
// ============================================================
void mutate(Chromosome& chromosome, int N, RNG& rng) {
    std::uniform_real_distribution<double> ureal(0.0, 1.0);

    // ビット反転
    for (int i = 0; i < N; ++i) {
        if (ureal(rng) < PROB_BIT) {
            chromosome[i] = 1 - chromosome[i];
        }
    }

    // 順序パートのスワップ
    if (ureal(rng) < PROB_SWAP) {
        std::uniform_int_distribution<int> dist(N, N + MAX_CONTROLS - 1);
        int a = dist(rng);
        int b = dist(rng);
        while (b == a) b = dist(rng);
        std::swap(chromosome[a], chromosome[b]);
    }
}

// ============================================================
// 【改良③】局所探索（山登り法・メメティックGA）
//   2種類の入れ替えを試し、fitness が一番下がる手を採用。改善が止まるまで繰り返す。
//    (1) 選択の交換：選ばれている1地点を未選択の候補と入れ替える → f_dist に効く
//    (2) 巡回順の入れ替え：巡る順番の2地点を入れ替える → f_time（時間のずれ）に効く
//   「どの8個を選ぶか」だけでなく「どの順で回るか」も磨くので floor に近づける。
//   ※交換は1対1なので選択数は8のまま保たれる。
// ============================================================
void local_search(Chromosome& chrom, const std::vector<Landmark>& landmarks,
                  const PathCache& path_cache, long long gate_node) {
    const int N = static_cast<int>(landmarks.size());
    double current = evaluate(chrom, landmarks, path_cache, gate_node).fitness;

    for (int iter = 0; iter < LS_MAX_ITER; ++iter) {
        int    best_kind = -1;            // 0:選択の交換 / 1:巡回順の入れ替え
        int    best_a = -1, best_b = -1;
        double best_fit = current;

        // (1) 選択の交換：選ばれている out を未選択の in と入れ替える（f_dist に効く）
        for (int out = 0; out < N; ++out) {
            if (chrom[out] != 1) continue;
            for (int in = 0; in < N; ++in) {
                if (chrom[in] != 0) continue;
                chrom[out] = 0; chrom[in] = 1;    // 仮に交換
                double f = evaluate(chrom, landmarks, path_cache, gate_node).fitness;
                chrom[out] = 1; chrom[in] = 0;    // 元に戻す
                if (f < best_fit) { best_fit = f; best_kind = 0; best_a = out; best_b = in; }
            }
        }

        // (2) 巡回順の入れ替え：順序パート先頭 Q_TARGET 個の2点を入れ替える（f_time に効く）
        for (int a = N; a < N + Q_TARGET; ++a) {
            for (int b = a + 1; b < N + Q_TARGET; ++b) {
                std::swap(chrom[a], chrom[b]);    // 仮に巡回順を入れ替え
                double f = evaluate(chrom, landmarks, path_cache, gate_node).fitness;
                std::swap(chrom[a], chrom[b]);    // 元に戻す
                if (f < best_fit) { best_fit = f; best_kind = 1; best_a = a; best_b = b; }
            }
        }

        if (best_kind < 0) break;                 // これ以上良くならない → 終了
        if (best_kind == 0) { chrom[best_a] = 0; chrom[best_b] = 1; }  // 選択の交換を確定
        else                { std::swap(chrom[best_a], chrom[best_b]); }  // 巡回順の入れ替えを確定
        current = best_fit;
    }
}

// ============================================================
// GA メインループ
// ============================================================
GAResult run_ga(
    const std::vector<Landmark>& landmarks,
    const PathCache&             path_cache,
    long long                    gate_node,
    RNG&                         rng)
{
    const int N = static_cast<int>(landmarks.size());

    // 初期個体群
    //   【改良②】一部を farthest-point で「散らばった8地点」にして種を撒き、
    //   残りは従来どおりランダム。良い遺伝子の注入と多様性の両立を狙う。
    std::vector<Chromosome> population;
    population.reserve(POP_SIZE);
    const int n_greedy = static_cast<int>(POP_SIZE * GREEDY_INIT_RATIO);
    for (int i = 0; i < POP_SIZE; ++i) {
        if (i < n_greedy) {
            population.push_back(create_greedy_chromosome(N, landmarks, rng));
        } else {
            population.push_back(create_random_chromosome(N, rng));
        }
    }

    // 初期評価
    std::vector<double> fitnesses(POP_SIZE);
    for (int i = 0; i < POP_SIZE; ++i) {
        fitnesses[i] = evaluate(population[i], landmarks, path_cache, gate_node).fitness;
    }

    std::vector<double> best_history;
    best_history.reserve(N_GEN);

    for (int gen = 1; gen <= N_GEN; ++gen) {
        std::vector<Chromosome> next_pop;
        next_pop.reserve(POP_SIZE);

        // エリート保存：最良個体を1つそのまま次世代へ
        int elite_idx = static_cast<int>(
            std::min_element(fitnesses.begin(), fitnesses.end()) - fitnesses.begin());
        next_pop.push_back(population[elite_idx]);

        // 残りは選択・交叉・突然変異で生成
        while (static_cast<int>(next_pop.size()) < POP_SIZE) {
            const Chromosome& p1 = tournament_select(population, fitnesses, rng);
            const Chromosome& p2 = tournament_select(population, fitnesses, rng);
            auto children = crossover(p1, p2, N, rng);
            mutate(children.first,  N, rng);
            mutate(children.second, N, rng);
            repair_selection(children.first,  N, rng);   // 【改良①】常に8個に揃える
            repair_selection(children.second, N, rng);
            next_pop.push_back(std::move(children.first));
            if (static_cast<int>(next_pop.size()) < POP_SIZE) {
                next_pop.push_back(std::move(children.second));
            }
        }

        population = std::move(next_pop);

        // 新世代を評価
        for (int i = 0; i < POP_SIZE; ++i) {
            fitnesses[i] = evaluate(population[i], landmarks, path_cache, gate_node).fitness;
        }

        // 【改良③】上位 LS_TOP_K 個体を局所探索で仕上げる（メメティックGA）
        std::vector<int> idx(POP_SIZE);
        std::iota(idx.begin(), idx.end(), 0);
        std::partial_sort(idx.begin(), idx.begin() + LS_TOP_K, idx.end(),
            [&](int a, int b) { return fitnesses[a] < fitnesses[b]; });
        for (int t = 0; t < LS_TOP_K; ++t) {
            int i = idx[t];
            local_search(population[i], landmarks, path_cache, gate_node);
            fitnesses[i] = evaluate(population[i], landmarks, path_cache, gate_node).fitness;
        }

        double best = *std::min_element(fitnesses.begin(), fitnesses.end());
        best_history.push_back(best);

        std::cout << "  [世代 " << gen << "]  best_fitness = " << best << std::endl;
    }

    int best_idx = static_cast<int>(
        std::min_element(fitnesses.begin(), fitnesses.end()) - fitnesses.begin());

    GAResult result;
    result.best_chromosome      = population[best_idx];
    result.best_eval            = evaluate(result.best_chromosome, landmarks, path_cache, gate_node);
    result.best_fitness_history = std::move(best_history);
    return result;
}

// ============================================================
// 【改良④】多スタート：GAを n_restarts 回まわし、一番良い結果を返す。
//   1回ごとに異なる種(base_seed由来)で走らせ、当たり外れを吸収して
//   安定して良い解を得る。種は固定なので結果は決定的
//   （＝採点で10回実行してもすべて同一 → ばらつき SD = 0）。
// ============================================================
GAResult run_ga_best(
    const std::vector<Landmark>& landmarks,
    const PathCache&             path_cache,
    long long                    gate_node,
    unsigned int                 base_seed,
    int                          n_restarts)
{
    GAResult best;
    double   best_fit = std::numeric_limits<double>::max();
    for (int r = 0; r < n_restarts; ++r) {
        RNG rng(base_seed * 1000u + static_cast<unsigned int>(r));
        GAResult res = run_ga(landmarks, path_cache, gate_node, rng);
        if (res.best_eval.fitness < best_fit) {
            best_fit = res.best_eval.fitness;
            best     = res;
        }
    }
    return best;
}

} // namespace orienteering
