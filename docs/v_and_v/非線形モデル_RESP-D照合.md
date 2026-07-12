# 非線形モデル（RESP-D「05 非線形モデル」）照合

**原典:** RESP-D 操作・計算マニュアル 計算編「05. 非線形モデル」（ユーザー提供資料、
2026-07-12 照合）。本ドキュメントは同マニュアルとの照合で追加実装した項目を記録する。

## 実装した項目

### 1. RC 耐震壁のせん断非線形特性（トリリニア Qc/βu/Qu）

**対象:** `squid-n-design-jp/src/rc/wall_nonlinear.rs`（新規）。
非線形解析のせん断ばね骨格に用いるトリリニア（ひび割れ・降伏剛性低下・終局）を算定する。
従来は許容応力度検定（RC規準18条、`rc/wall.rs`）のみで、非線形骨格は未実装だった。

| 諸元 | 式 | 出典 |
|---|---|---|
| せん断ひび割れ強度 Qc | `(0.043·pg+0.051)·√Fc·Aw`（工学単位系 kgf/cm²・cm² で評価し N へ換算） | 技術基準解説書 P.635-637 |
| せん断降伏時剛性低下率 βu | `0.46·pw·σy/Fc+0.14`（σy/Fc は比のため単位非依存） | 同上 |
| 終局せん断強度 Qu | `{k·pte^0.23·(Fc+18)/(M/QD+0.12)+0.85·√(σwh·pwh)+0.1·σ0}·te·j·r`、k=0.053/0.068 | 荒川mean式系・技術基準解説書 P.281-282,638-639 |
| 開口低減率 r | `1−max(r0, l0/lw, h0/h)`、`r0=√(h0·l0/(h·lw))` | RC終局強度設計資料 P.132 |

**配線:** `joint_wiring::collect_joint_checks_with_long` が Wall 要素（RcWall 形状）＋付帯柱
から入力を組み立て、`wall_shear_trilinear` を評価。結果は `joint_checks` に
「耐震壁(RC)せん断非線形」ラベル（Qu 検定比＋Qc/βu/Qu/r を detail 表示）として追加され、
アプリ設計タブ（`design_view.rs`「接合部・耐震壁の検定」）に表示される。

**検証:** `rc/wall_nonlinear.rs` 手計算照合テスト 10 件（Qc・βu・Qu・開口低減・単位換算・
クランプ）＋`joint_wiring/tests.rs::wall_with_side_columns_emits_nonlinear_shear_trilinear`。

### 2. プッシュオーバーのヒンジ判定を実スケルトン化＋部材塑性率3方式

**対象:** `squid-n-solver/src/nonlinear/pushover/mod.rs`。

- **ヒンジ閾値の実スケルトン化:** 従来の粗い仮値（My=σy·Z弾性、Mc=My/3、Mu=My·1.2）を、
  RC=ひび割れ `Mc=κ·Fc·Ze`（κ=0.56）・降伏 `My=0.9·at·σy·j`、鉄骨=全塑性 `Mp=Zp·σy`
  （H/箱/パイプは閉形式 Zp）へ置換（`member_moment_thresholds`）。
- **部材塑性率（ductility）3方式:** RESP-D の 3 方式を実装（`DuctilityMethod`）。
  1. 塑性率基点歪み（RC: 引張0.01/圧縮0.005、鉄骨0.01）
  2. 重み付け平均塑性率 Jm=Σσref·A·|ε|·μi/Σσref·A·|ε|≥1
  3. 降伏発生時（塑性率1超）
  ファイバー要素（`FiberBeam::ductility_probe`）が危険断面の曲率・ひずみを集約し、
  塑性率基点曲率と最大応答曲率から μ=最大応答曲率/基点曲率を算定。降伏後 μ≥`ULTIMATE_DUCTILITY`
  （既定4.0）のヒンジを終局と分類。`HingeEvent.ductility` が実塑性率を持つ。

**配線:** 材料に `reference_stress/reference_strain`（`squid-n-material`）、要素に
`ductility_probe`（`squid-n-element`）を追加。アプリ解析タブに塑性率方式の選択 UI、
結果タブ（プッシュオーバー）に方式・最大部材塑性率 μmax・ヒンジ別 μ を表示。

**検証:** `pushover/tests.rs::test_pushover_computes_member_ductility`（降伏後 μ≥1）・
`test_pushover_ductility_method_selection_changes_reference`（3方式とも妥当な μ を算定）。
既存プッシュオーバー・段階的耐力喪失テストは全て緑（回帰なし）。

## 原典照合が必要な埋め込み値（技術リード確認用）

`specs/原典照合リスト.md`「RESP-D 計算編 05 非線形モデル」節を参照。
主要な埋め込み値: κ=0.56（ひび割れ）、Qc/βu/Qu の各係数、開口低減式、
塑性率基点歪み（0.01/0.005）、ULTIMATE_DUCTILITY=4.0、壁筋 σy/σwh 既定 295。

## 未実装（本フェーズのスコープ外・別途要検討）

RESP-D「05 非線形モデル」との照合で確認したが今回未着手の主な差異
（ユーザー選択により壁せん断・ヒンジ/塑性率を優先実装）:

- コンクリート構成則 NewRC モデル（現状は放物線モデル、`ec0/ecu` ハードコード）
- ファイバー断面の対数分割（RC 9 分割）・鉄骨 16/7 分割（現状は等分割）
- 鉄骨梁の横座屈モーメント Mcr、SRC 梁のせん断終局（3式・非充腹）
- 梁曲げトリリニアの菅野 αy 式による骨格（現状はファイバー積分ベース）
- ファイバー要素の鉄筋分離（現状は均質コンクリート断面）・第1折点 1.1倍/α=1/1000
