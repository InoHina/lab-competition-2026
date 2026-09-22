// ============================================================
// Step 5: 遺伝的アルゴリズム（GA） C++版
// Python版（step5_ga.py）と同じロジックを実装
// ============================================================

#include "const.h"
#include "csv_loader.h"
#include "ga.h"
#include "graph.h"
#include "evaluate.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <set>
#include <string>

#ifdef _WIN32
#include <windows.h>  // Windows のみ：コンソールを UTF-8 表示にするため
#endif

using namespace orienteering;

namespace {

// 結果を JSON 形式で書き出す（Python版 best_course.json と同じ構造）
void write_best_course_json(
    const std::string&            path,
    const GAResult&               result,
    const std::vector<Landmark>&  landmarks)
{
    const auto& ev = result.best_eval;
    const auto& d  = ev.decoded;
    double t_estimated = (ev.total_distance / WALK_SPEED
                        + ev.total_gain     / CLIMB_SPEED) * 60.0;

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        throw std::runtime_error("JSON ファイルが書き込めません: " + path);
    }

    auto fmt = [](double v, int p) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.*f", p, v);
        return std::string(buf);
    };

    ofs << "{\n";
    ofs << "  \"n_controls\": "         << d.selected_indices.size() << ",\n";
    ofs << "  \"total_distance_m\": "   << fmt(ev.total_distance, 1) << ",\n";
    ofs << "  \"total_gain_m\": "       << fmt(ev.total_gain, 1)     << ",\n";
    ofs << "  \"estimated_time_min\": " << fmt(t_estimated, 1)       << ",\n";
    ofs << "  \"fitness\": "            << fmt(ev.fitness, 6)        << ",\n";

    ofs << "  \"objectives\": {\n";
    ofs << "    \"f_map\": "   << fmt(ev.objectives.f_map,   3) << ",\n";
    ofs << "    \"f_dist\": "  << fmt(ev.objectives.f_dist,  3) << ",\n";
    ofs << "    \"f_time\": "  << fmt(ev.objectives.f_time,  3) << ",\n";
    ofs << "    \"f_route\": " << fmt(ev.objectives.f_route, 3) << "\n";
    ofs << "  },\n";

    ofs << "  \"weights\": {\n";
    ofs << "    \"f_map\": "   << fmt(W_MAP,   2) << ",\n";
    ofs << "    \"f_dist\": "  << fmt(W_DIST,  2) << ",\n";
    ofs << "    \"f_time\": "  << fmt(W_TIME,  2) << ",\n";
    ofs << "    \"f_route\": " << fmt(W_ROUTE, 2) << "\n";
    ofs << "  },\n";

    ofs << "  \"controls\": [\n";
    for (size_t i = 0; i < d.selected_indices.size(); ++i) {
        const auto& lm = landmarks[d.selected_indices[i]];
        ofs << "    {";
        ofs << "\"name\": \""    << lm.name                 << "\", ";
        ofs << "\"feature\": \"" << lm.feature              << "\", ";
        ofs << "\"lat\": "       << fmt(lm.lat, 7)          << ", ";
        ofs << "\"lon\": "       << fmt(lm.lon, 7);
        ofs << "}";
        if (i + 1 < d.selected_indices.size()) ofs << ",";
        ofs << "\n";
    }
    ofs << "  ],\n";

    ofs << "  \"course_nodes\": [";
    for (size_t i = 0; i < d.course_nodes.size(); ++i) {
        ofs << d.course_nodes[i];
        if (i + 1 < d.course_nodes.size()) ofs << ", ";
    }
    ofs << "]\n";
    ofs << "}\n";
}

void write_fitness_history_csv(
    const std::string&         path,
    const std::vector<double>& history)
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        throw std::runtime_error("CSV ファイルが書き込めません: " + path);
    }
    ofs << "generation,best_fitness\n";
    ofs << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < history.size(); ++i) {
        ofs << (i + 1) << "," << history[i] << "\n";
    }
}

} // namespace

int main() {
#ifdef _WIN32
    // Windows コンソールを UTF-8 表示にする（日本語の文字化け防止）。
    // ソースは UTF-8 で記述しているため、出力側もコードページを UTF-8 に揃える。
    SetConsoleOutputCP(CP_UTF8);
#endif
    try {
        std::cout << "データを読み込み中..." << std::endl;

        auto landmarks = load_landmarks(DATA_DIR + "/landmarks.csv");
        auto nodes     = load_nodes    (DATA_DIR + "/nodes.csv");
        auto edges     = load_edges    (DATA_DIR + "/edges.csv");
        auto gate      = load_gate     (DATA_DIR + "/seimon.csv");

        const int N = static_cast<int>(landmarks.size());
        std::cout << "  候補数: " << N
                  << "件 / ノード: " << nodes.size()
                  << "件 / エッジ: " << edges.size()
                  << "件" << std::endl;

        Graph     graph(nodes, edges);
        long long gate_node = graph.find_nearest_node(gate.lat, gate.lon);

        // 最短経路の事前計算（ゲート + 全ランドマークの nearest_node を起点に）
        std::cout << "  最短経路を事前計算中..." << std::endl;
        std::set<long long> source_set;
        source_set.insert(gate_node);
        for (const auto& lm : landmarks) source_set.insert(lm.nearest_node);
        std::vector<long long> sources(source_set.begin(), source_set.end());

        PathCache path_cache(graph, sources);

        std::cout << "\n遺伝的アルゴリズムを実行中..." << std::endl;
        std::cout << "  個体数: " << POP_SIZE
                  << " / 世代数: " << N_GEN << std::endl;

        auto t_start = std::chrono::high_resolution_clock::now();
        auto result  = run_ga_best(landmarks, path_cache, gate_node, RANDOM_SEED, N_RESTARTS);
        auto t_end   = std::chrono::high_resolution_clock::now();
        double elapsed = std::chrono::duration<double>(t_end - t_start).count();

        // 結果の出力
        std::filesystem::create_directories(OUTPUT_DIR);
        write_best_course_json(OUTPUT_DIR + "/best_course.json", result, landmarks);
        write_fitness_history_csv(OUTPUT_DIR + "/fitness_history.csv", result.best_fitness_history);

        std::cout << "\n  → " << OUTPUT_DIR << "/best_course.json に保存しました" << std::endl;
        std::cout << "  → " << OUTPUT_DIR << "/fitness_history.csv に保存しました" << std::endl;

        const auto& ev = result.best_eval;
        double t_estimated = (ev.total_distance / WALK_SPEED
                            + ev.total_gain     / CLIMB_SPEED) * 60.0;

        std::cout << "\n========== Step 5 (C++) 完了 ==========" << std::endl;

        // 最良個体の巡回順（正門スタート → コントロール → 正門ゴール）
        std::cout << "巡回順（最良個体）:" << std::endl;
        std::cout << "  スタート : 正門" << std::endl;
        const auto& sel = ev.decoded.selected_indices;
        for (size_t i = 0; i < sel.size(); ++i) {
            const auto& lm = landmarks[sel[i]];
            std::cout << "  " << lm.id << " : " << lm.name << std::endl;
        }
        std::cout << "  ゴール   : 正門" << std::endl;

        std::cout << std::fixed;
        std::cout << "実行時間          : " << std::setprecision(2) << elapsed << " 秒" << std::endl;
        std::cout << "最良解の fitness  : " << std::setprecision(4) << ev.fitness << std::endl;
        std::cout << "コントロール数    : " << ev.decoded.selected_indices.size() << " 件" << std::endl;
        std::cout << "推定時間          : " << std::setprecision(1) << t_estimated << " 分" << std::endl;
        std::cout << "累積登り          : " << std::setprecision(1) << ev.total_gain << " m" << std::endl;
        std::cout << std::setprecision(3);
        std::cout << "f_map             : " << ev.objectives.f_map   << std::endl;
        std::cout << "f_dist            : " << ev.objectives.f_dist  << std::endl;
        std::cout << "f_time            : " << ev.objectives.f_time  << std::endl;
        std::cout << "f_route           : " << ev.objectives.f_route << std::endl;
        std::cout << "==========================================" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "エラー: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
