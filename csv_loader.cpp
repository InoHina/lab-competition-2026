#include "csv_loader.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace orienteering {

namespace {

// 1行をカンマで分割する（簡易版：フィールド内のカンマは想定しない）
std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> tokens;
    std::string cur;
    for (char c : line) {
        if (c == ',') {
            tokens.push_back(cur);
            cur.clear();
        } else if (c != '\r') {
            cur += c;
        }
    }
    tokens.push_back(cur);
    return tokens;
}

// 先頭の UTF-8 BOM を取り除く
void strip_bom(std::string& s) {
    if (s.size() >= 3 &&
        static_cast<unsigned char>(s[0]) == 0xEF &&
        static_cast<unsigned char>(s[1]) == 0xBB &&
        static_cast<unsigned char>(s[2]) == 0xBF) {
        s.erase(0, 3);
    }
}

std::ifstream open_or_throw(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        throw std::runtime_error("CSV ファイルが開けません: " + path);
    }
    return ifs;
}

} // namespace

std::vector<Landmark> load_landmarks(const std::string& path) {
    std::ifstream ifs = open_or_throw(path);
    std::vector<Landmark> landmarks;

    std::string header;
    std::getline(ifs, header);  // ヘッダ行はスキップ

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        strip_bom(line);
        auto t = split_csv_line(line);
        if (t.size() < 6) continue;
        Landmark lm;
        lm.id           = std::stoi(t[0]);
        lm.name         = t[1];
        lm.feature      = t[2];
        lm.lat          = std::stod(t[3]);
        lm.lon          = std::stod(t[4]);
        lm.nearest_node = std::stoll(t[5]);
        landmarks.push_back(lm);
    }
    return landmarks;
}

std::vector<Node> load_nodes(const std::string& path) {
    std::ifstream ifs = open_or_throw(path);
    std::vector<Node> nodes;

    std::string header;
    std::getline(ifs, header);

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        strip_bom(line);
        auto t = split_csv_line(line);
        if (t.size() < 4) continue;
        Node n;
        n.node_id   = std::stoll(t[0]);
        n.lat       = std::stod(t[1]);
        n.lon       = std::stod(t[2]);
        n.elevation = std::stod(t[3]);
        nodes.push_back(n);
    }
    return nodes;
}

std::vector<Edge> load_edges(const std::string& path) {
    std::ifstream ifs = open_or_throw(path);
    std::vector<Edge> edges;

    std::string header;
    std::getline(ifs, header);

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        strip_bom(line);
        auto t = split_csv_line(line);
        if (t.size() < 5) continue;
        Edge e;
        e.from_node        = std::stoll(t[0]);
        e.to_node          = std::stoll(t[1]);
        e.length_m         = std::stod(t[2]);
        e.elevation_change = std::stod(t[3]);
        e.elevation_gain   = std::stod(t[4]);
        edges.push_back(e);
    }
    return edges;
}

Gate load_gate(const std::string& path) {
    std::ifstream ifs = open_or_throw(path);

    std::string header, line;
    std::getline(ifs, header);  // 1行目: ヘッダ（読み飛ばす）
    std::getline(ifs, line);    // 2行目: 座標

    auto t = split_csv_line(line);
    if (t.size() < 2) {
        throw std::runtime_error("正門の座標が読み込めません: " + path);
    }
    Gate g;
    g.lat = std::stod(t[0]);
    g.lon = std::stod(t[1]);
    return g;
}

} // namespace orienteering
