# 進化計算コンペ2026 — 提出プログラム（メメティックGA）

## 手法の概要（配布サンプルからの改良点）

配布サンプル（GA）の適応度の内訳を分析すると、そのほぼ全てが
**f_dist（コントロールが 150m 以内に近接するペナルティ）** だった。
そこで「支配項 f_dist を狙い撃つ」方針で、**探索部（`ga.cpp`）だけ**を改良した。
評価関数 `evaluate.cpp` と最短経路 `graph.cpp` は採点基準そのものなので変更していない。

| 版 | 改良内容 | 20回平均の fitness |
|---|---|---|
| ベースライン | 配布サンプルそのまま | 1.45 |
| ① コントロール数を8に固定 | 探索を「どの8個・どの順で回るか」に集中（f_map=0 を保証） | 1.06 |
| ② farthest-point 初期化 | 散らばった8地点を初期集団に注入 | 1.07 |
| ③ 局所探索（メメティックGA） | 近すぎる地点をより良い候補へ交換して仕上げる山登り | 0.12 |
| ④ 多スタート | GAを数回まわし一番良い結果を採用（seed固定で決定的） | **0.08** |

- ベースライン比で平均 **1.45 → 0.08（約94%改善）**、処理時間は約2秒
- seed を固定しているため、10回実行しても結果は同一（ばらつき **SD = 0**）
- 主な改良は `ga.cpp` と `const.h`（GAパラメータ）にある

## ファイル構成

```
orienteering-cpp-test/
├── const.h            # 共通定数（コントロール数の制約・目標値・GAパラメータ等）
├── csv_loader.h/cpp   # CSV ロード（landmarks/nodes/edges）
├── graph.h/cpp        # 道路ネットワーク + Dijkstra + 経路キャッシュ
├── evaluate.h/cpp     # 染色体デコード + 4目的関数 + 評価関数
├── ga.h/cpp           # GA 操作（選択・交叉・突然変異・メインループ）
├── main.cpp           # エントリポイント・入出力
├── Makefile           # ビルド設定（コマンドラインから make で利用）
├── input/             # 入力 CSV ファイル
│   ├── landmarks.csv
│   ├── nodes.csv
│   ├── edges.csv
│   └── seimon.csv
├── output/            # 実行結果出力先
│   ├── best_course.json
│   └── fitness_history.csv
└── README.md
```

### 入力データの形式（`input/`）
**`landmarks.csv`** … コントロール候補となる地点の一覧

| 列 | 意味 |
|---|---|
| `id` | 候補の通し番号 |
| `name` | 地点名 |
| `feature` | 種別（例：`post_office`） |
| `lat`, `lon` | 緯度・経度 |
| `nearest_node` | 最寄りの道路ノード ID（`nodes.csv` の `node_id` と対応） |

**`nodes.csv`** … 道路ネットワークの地点（交差点など）

| 列 | 意味 |
|---|---|
| `node_id` | ノードの一意な ID |
| `lat`, `lon` | 緯度・経度 |
| `elevation` | 標高（m） |

**`edges.csv`** … ノード間を結ぶ道（有向エッジ）

| 列 | 意味 |
|---|---|
| `from_node`, `to_node` | 始点・終点のノード ID |
| `length_m` | 道の長さ（m） |
| `elevation_change` | 標高差（+ 登り / − 下り） |
| `elevation_gain` | 累積登り（m） |

**`seimon.csv`** … スタート・ゴール地点（正門）の座標

| 列 | 意味 |
|---|---|
| `lat`, `lon` | 緯度・経度 |

先頭の有効な 1 行のみを使用する。

---

### 出力データ（`output/`）
- 中身は空
- 実行後、best_course.jsonとfitness_history.csvが作られる。

## 環境構築 + ビルド方法

### リポジトリの取得（git clone）※ macOS / Windows 共通

1. **作業用フォルダをローカルに作る**

   プロジェクトを置きたい場所に、任意の名前でフォルダを作る（Finder / エクスプローラーで作ってもよい）。
   ターミナル（macOS）/ コマンドプロンプト（Windows）で作る場合：
   ```bash
   mkdir competition2026      # 好きな名前・場所でよい
   cd competition2026         # 作ったフォルダに移動
   ```

2. **リポジトリをクローンする**

   上で移動した `competition2026` フォルダの中で：
   ```bash
   git clone https://github.com/nhjnhjnhj/orienteering-cpp-test.git
   ```

### macOS（VSCode + C/C++拡張）

1. **必要なツール**
   - Visual Studio Code
   - VSCode 拡張機能：
     - `C/C++`（Microsoft 製、IntelliSense 用）
     - `C/C++ Extension Pack`（推奨）
   > 注：Gitを使えない人はコンパイラのインストールが必要

2. **プロジェクトを開く**
   - **方法A：ドラッグ&ドロップ**
     Finder で `orienteering-cpp-test` フォルダを、VSCode のウィンドウにドラッグ&ドロップする。
   - **方法B：メニューから開く**
     VSCode のメニュー `File` →「`Open Folder...`（フォルダを開く）」→ `orienteering-cpp-test ` フォルダを選択する。

3. **ビルド & 実行（ターミナル経由）**
   - VSCode 統合ターミナル（`Ctrl + ` ` で開く）で：
   ```bash
   make run        # ビルドして実行
   ```

### Windows（Visual Studio）

1. **必要なもの**
   - Visual Studio
   - インストール時に **「C++ によるデスクトップ開発」** ワークロードを選択

2. **新規プロジェクト作成**
   - `ファイル` → `新規作成` → `プロジェクト`
   - **空のプロジェクト（C++）** を選択
   - プロジェクト名：`competition2026`（任意）
   - 場所：`orienteering-cpp-test` の親フォルダなどわかりやすいところ

3. **ソースファイルの追加**
   - ソリューションエクスプローラで「ソースファイル」を右クリック → `追加` → `既存の項目`
   - 以下を追加：
     - `csv_loader.cpp`
     - `graph.cpp`
     - `evaluate.cpp`
     - `ga.cpp`
     - `main.cpp`
   - 同様に「ヘッダーファイル」へ `.h` を全て追加：
     - `const.h`, `csv_loader.h`, `graph.h`, `evaluate.h`, `ga.h`

4. **C++ 言語標準の設定**
   - プロジェクト右クリック → `プロパティ`
   - `構成プロパティ` → `全般` → `C++ 言語標準` → **ISO C++17 標準 (/std:c++17)** に設定
   - 構成は `すべての構成` を選択して Debug/Release 両方に適用

5. **文字コードの設定（`/utf-8`）**
   - プロパティ → `構成プロパティ` → `C/C++` → `コマンドライン` → 「追加のオプション」に **`/utf-8`** を記載。
   - 左上の構成を`すべての構成`に変更し、「OK」を押す。

6. **input / output フォルダの扱い（重要）**
     - プロパティ → `構成プロパティ` → `デバッグ` → `作業ディレクトリ` を、
       `input/` のあるフォルダに設定する（例：`$(ProjectDir)..\` など、プロジェクトの作成場所に応じて調整）。

7. **ビルド & 実行**
   - `ビルド` → `ソリューションのビルド`（または `Ctrl + Shift + B`）
   - `デバッグ` → `デバッグなしで開始`（`Ctrl + F5`）

---

## トラブルシューティング

### 1. コード1で終了 / CSV ファイルが開けません: input/landmarks.csv

実行時に上記エラーで終了する場合、**作業ディレクトリの指定ミス**が原因（Visual Studio で頻発）。

- プログラムは `input/landmarks.csv` のように **相対パス**で開く。基準は**実行時の作業ディレクトリ**。
- ありがちな間違い：作業ディレクトリに `input` フォルダ**そのもの**を指定してしまう。
  ```
  作業ディレクトリ = C:\orienteering\input     ← ✗
   → 探しに行くのは C:\orienteering\input\input\landmarks.csv（input が二重）→ 失敗
  ```
- 正しくは、`input` フォルダの**親（input を含むフォルダ）**を指定する。
  ```
  作業ディレクトリ = C:\orienteering           ← ◯
   → C:\orienteering\input\landmarks.csv に正しく到達
  ```
- Visual Studio：プロパティ → `構成プロパティ` → `デバッグ` → 「作業ディレクトリ」を上記に修正。
  反映されない場合は「構成（Debug/Release）」「プラットフォーム（x64/Win32）」が実行中のものと一致しているか確認する。

### 2. 日本語が文字化けする（Windows コンソール）

`SetConsoleOutputCP(CP_UTF8)` で対策済み。それでも化ける場合は、実行前にコンソールで `chcp 65001` を実行する。
