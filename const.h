#ifndef CONST_H
#define CONST_H

#include <cfloat>
#include <string>

namespace orienteering {

// ============================================================
// ファイル・ディレクトリ
// ============================================================
inline const std::string DATA_DIR   = "input";    // 入力データのフォルダ
inline const std::string OUTPUT_DIR = "output";   // 出力ファイルのフォルダ

// ============================================================
// コントロール数の制約（コンペ課題で指定）
// ============================================================
constexpr int MIN_CONTROLS = 6;
constexpr int MAX_CONTROLS = 12;

// ============================================================
// 目的関数パラメータ（コンペ課題 表1）
// ============================================================
constexpr int    Q_TARGET     = 8;        // 目標コントロール数
constexpr double D_MIN        = 150.0;    // 近すぎる距離の閾値（m）
constexpr double T_TARGET     = 60.0;     // 目標所要時間（分）
constexpr double WALK_SPEED   = 4020.0;   // 平地の歩行速度（m/時）
constexpr double CLIMB_SPEED  = 300.0;    // 登り坂の速度換算値（m/時）
constexpr double ROUTE_TARGET = 50.0;     // 累積登り高低差の許容値（m）
constexpr double PENALTY      = DBL_MAX;   // 無効解へのペナルティ（実値では到達し得ないセンチネル）

// ============================================================
// 重み設定（4指標を重み付き和で集約。合計 1.0）
// ============================================================
constexpr double W_MAP   = 0.50;
constexpr double W_DIST  = 0.10;
constexpr double W_TIME  = 0.25;
constexpr double W_ROUTE = 0.15;

// ============================================================
// GA のハイパーパラメータ
// ============================================================
constexpr int    POP_SIZE        = 100;   // 個体数
constexpr int    N_GEN           = 30;    // 世代数
constexpr double PROB_BIT        = 0.02;  // 選択パートのビット反転確率
constexpr double PROB_SWAP       = 0.10;  // 順序パートのスワップ確率
constexpr int    TOURNAMENT_SIZE = 5;     // トーナメント選択のサイズ
constexpr double GREEDY_INIT_RATIO = 0.5; // 【改良②】初期集団のうち farthest-point で作る割合
constexpr int    LS_TOP_K          = 3;   // 【改良③】各世代で局所探索する上位個体数
constexpr int    LS_MAX_ITER       = 20;  // 【改良③】局所探索(山登り)の最大反復回数
constexpr int    N_RESTARTS        = 1;   // 【改良④】多スタート：GAを回す回数（1=単発。速度優先で1に設定）

constexpr unsigned int RANDOM_SEED = 42;

// ============================================================
// 物理定数
// ============================================================
constexpr double PI                = 3.14159265358979323846;
constexpr double METERS_PER_DEGREE = 111320.0;  // 緯度1度あたりのメートル

} // namespace orienteering

#endif // CONST_H
