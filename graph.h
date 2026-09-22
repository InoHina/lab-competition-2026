#ifndef GRAPH_H
#define GRAPH_H

#include "csv_loader.h"

#include <unordered_map>
#include <vector>

namespace orienteering {

// 1本のエッジ情報
struct EdgeInfo {
    long long to;
    double    length;
    double    elevation_gain;
};

// 最短路の集計結果（距離と累積登り）
struct PathInfo {
    double length;        // 距離（m）
    double gain;          // 累積登り（m）
    bool   reachable;     // 到達可能か
};

// ============================================================
// 道路ネットワーク（有向グラフ）
// ============================================================
class Graph {
public:
    Graph(const std::vector<Node>& nodes, const std::vector<Edge>& edges);

    // src から到達可能な全ノードに対し、（最短距離・その経路上の累積登り）を計算
    // 戻り値：node_id → PathInfo の辞書
    std::unordered_map<long long, PathInfo> dijkstra_from(long long src) const;

    // ゲートの緯度経度に最も近いノードIDを返す
    long long find_nearest_node(double lat, double lon) const;

private:
    std::unordered_map<long long, std::vector<EdgeInfo>> adj_;
    std::vector<Node> node_list_;  // 最近傍検索用
};

// ============================================================
// 最短経路キャッシュ
// 興味のあるノード集合 sources × sources の (距離, 登り) を事前計算
// ============================================================
class PathCache {
public:
    PathCache(const Graph& graph, const std::vector<long long>& sources);

    PathInfo get(long long src, long long dst) const;

private:
    std::unordered_map<long long, std::unordered_map<long long, PathInfo>> cache_;
};

} // namespace orienteering

#endif // GRAPH_H
