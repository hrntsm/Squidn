# 非線形解析（動的解析）（RESP-D「07 非線形解析（動的解析）」）照合

**原典:** RESP-D 操作・計算マニュアル 計算編「07. 非線形解析（動的解析）」
（ユーザー提供資料、2026-07-12 照合）。本ドキュメントは同マニュアル「履歴特性」
および「立体解析モデルの非線形特性（既定の非線形特性）」との照合で追加・是正した
項目を記録する。

## 実装した項目

### 1. 履歴特性（履歴則）の拡充 — 逆行型・標準型・最大点指向型

**対象:** `squid-n-material/src/hysteresis.rs`（`HysteresisRule` / `HysteresisMaterial`）。

従来は武田型（`Takeda`）・原点指向型（`OriginOriented`）・スリップ型（`Slip`）のみで、
マニュアル「履歴特性」の以下の名前付き履歴則が欠落していた。

| 履歴則 | 追加 variant | 挙動（原典） |
|---|---|---|
| 逆行型 | `Retrograde` | 常にスケルトンカーブ上を進む（除荷・再載荷を可逆に辿り履歴ループを生じない） |
| 標準型 | `Standard` | 除荷は Masing 則（相似則）。除荷開始剛性=初期剛性 K1、除荷後の第2/第3剛性は骨格の剛性低下率と同様 |
| 最大点指向型 | `MaxPointOriented` | \|δ\|<δy1 は原点勾配 K1。降伏後は戻り点から反対側の最大経験変形点を直線で指向（Clough 系） |

- `Retrograde`: `evaluate` 冒頭で常に `Branch::Skeleton` に短絡。
- `Standard`: 反転時に `Branch::Masing` へ遷移。`Q(θ)=Qr − 2·sgn(θr−θ)·g(|θr−θ|/2)`
  （`g`=スケルトン力）で除荷開始勾配 `g'(0)=K1`、反射点（反対側 \|θ\|≥\|反転点\|）で
  スケルトンへ復帰する。
- `MaxPointOriented`: 反転時（降伏後）に `Branch::PeakOriented` へ遷移。戻り点から
  反対側の最大経験点への直線を辿り、到達でスケルトンへ復帰。

**検証:** `hysteresis.rs` 手計算照合テスト（逆行型のスケルトン可逆性、標準型の除荷開始
剛性=K1・反射点復帰・θ=0 で履歴枝上、最大点指向型のピーク指向補間、新規則の
スケルトン一致）。

### 2. 武田型の除荷剛性式を原典へ是正

**対象:** `squid-n-material/src/hysteresis.rs::unloading_stiffness`。

原典「履歴特性 武田型」は `Kd+ = K0·|δmax/δy2|^(−ν)`（K0=初期勾配、δy2=降伏変形、
ν=除荷指数）を規定するが、従来実装は基準を降伏割線 `Ky=My/θy` としていた（誤り）。
初期勾配 `K0 = Mc/θc` を基準に是正した（既存 `alpha` を指数 ν として流用）。
武田型ループのスナップショット（除荷ゼロ交差点）が是正値に更新される（骨格の折れ点は
不変）。

### 3. 既定の非線形特性表 → 材端曲げバネへの配線

**対象:** `squid-n-core`（`HysteresisModel`・`default_member_hysteresis`・
`Model::member_hysteresis_attrs`）、`squid-n-element/src/factory/mod.rs`、
`squid-n-edit`（`SetMemberHysteresis`）、`squid-n-app`（部材表・設計ビュー）。

原典「立体解析モデルの非線形特性」の既定表（梁の曲げ: RC/SRC/CFT=武田型、S=標準型）を
実装し、材端集中バネ（`ConcentratedSpringBeam`）へ配線した。従来は全ての梁が kinematic
バイリニアで、RC 梁が武田型（剛性低下トリリニア）にならない不整合があった。

- **core:** `HysteresisModel{Auto,Retrograde,Standard,OriginOriented,MaxPointOriented,Takeda}`、
  部材個別指定の側テーブル `member_hysteresis_attrs`（`isolator_attrs` と同方式）、
  既定表 `default_member_hysteresis(rc_like)`。
- **element/factory:** `resolve_member_hysteresis`（個別指定→断面形状による既定表）で
  履歴則を解決し、`build_flexural_springs` が武田型/逆行型/最大点指向型は
  `HysteresisMaterial`（トリリニア）、原点指向型はバイリニア、標準型は従来 kinematic
  バイリニアの材端バネを構築。トリリニア折れ点は Mc=0.56·Fc·Ze（RC）/My/3、
  My=規準の曲げ終局、θy=My/(αy·k_rot)（αy=0.3 既定）、Mu=1.1·My、θu=4·θy。
  武田型等の履歴材料は N-M 相関（`set_yield`）非対応のため相関はバイリニア時のみ適用。
- **edit/UI:** `SetMemberHysteresis`（undo 可）、部材表に「履歴則」列（自動/逆行型/
  標準型/原点指向型/最大点指向型/武田型。梁のみ）、設計タブに材端履歴則の集計表示。

**検証:** `factory/tests.rs`（既定表の解決・RC=武田型/S=標準型・個別指定 override・
RC 材端バネが実際に武田型の除荷剛性低下を示す実挙動テスト）、`edit/tests.rs`
（`SetMemberHysteresis` の undo/redo・不存在部材の Noop）、`core/model/tests.rs`
（既定表・側テーブル round-trip）。既存のプッシュオーバー/時刻歴/段階的耐力喪失
テストは全て緑（回帰なし）。

### 4. 辻・山田モデル（β 混合硬化）

**対象:** `squid-n-material/src/hysteresis.rs`（`TsujiYamada`）、`squid-n-core`
（`HysteresisModel::TsujiYamada`）、`squid-n-element/src/factory/mod.rs`。

バイリニア骨格 + β による等方硬化/移動硬化の混合硬化則。塑性増分応力 Δσ を
等方硬化 `Δσ̄=β|Δσ|`（降伏幅膨張）と移動硬化 `Δᾱ=(1−β)|Δσ|`（中心移動）へ配分。
β=1 等方（降伏耐力が正負同時に膨張）、β=0 移動（バウシンガー効果）。`UniaxialMaterial`
として実装し、`set_yield` 対応のため材端バネでは N-M 相関も適用。部材表・設計ビューの
履歴則選択に「辻・山田型」を追加。

**検証:** 単調バイリニア・β=1 等方膨張・β=0 バウシンガー・commit/revert の4テスト。

### 5. 鉄骨梁端部の累積損傷度（レインフロー法）

**対象:** `squid-n-solver/src/damage.rs`（新規）、`squid-n-app/src/time_history_view.rs`。

RESP-D「その他の解析機能」の累積損傷度計算。ASTM E1049-85 3 点レインフロー計数
（`rainflow_cycles`）、レインフロー法 `D=ΣNei/Nfi`（`Nf=(μ/C)^(−1/β)`、片振幅
μ=振れ幅/2）、累積塑性変形倍率（最大振幅）法 `D=η/4·(μmax−1)·(μmax/C)^(1/β)`。
時刻歴ビューに代表応答のレインフロー等価繰返し数・最大振れ幅を参考表示。

**検証:** 折返し点抽出・入れ子サイクル・手計算照合の6テスト。

### 6. 免震支承材の装置別ひずみ依存モデル

**対象:** `squid-n-design-jp/src/isolator.rs`、`squid-n-app/src/design_view.rs`。

- **LRB 統一型（歪依存バイリニア）:** 降伏後剛性のひずみ依存 `CKd(γ)`、切片荷重の
  ひずみ依存 `CQd(γ)`、温度換算 `Kd(t0)/Qd(t0)`、等価水平剛性 `keq=Qd/δ+Kd`、
  等価粘性減衰定数 `Heq=(2/π)Qd(δ−Qd/((β−1)Kd))/(keq·δ²)`。
- **転がり支承:** `μ=(1.2+7.8·Pv/Po)/1000`、`Q1=μ·Pv`。
- **球面すべり支承:** 面圧依存 `μ0`（MN/LN）、MN 速度依存 `μ=μ0·(1−0.55·e^(−0.019|V|))`。
- **高減衰ゴム系（ブリヂストン E6/E4）:** `Geq/Heq/U` の歪 γ 多項式。
- 設計ビューの免震支承材一覧に積層ゴム系の keq/Heq（δ=200mm 参考）を表示。

**検証:** LRB ひずみ依存・温度換算・等価特性・転がり・球面すべり・高減衰ゴムの
手計算照合8テスト。

### 7. 減衰マトリクスの拡充（モード別減衰・接線剛性比例 α1/h1 一定）

**対象:** `squid-n-solver/src/dynamic/damping.rs`、`squid-n-solver/src/dynamic/timehistory/mod.rs`、
`squid-n-app`（`ThDampingModel`・減衰セレクタ・減衰構成）。

RESP-D「07」減衰マトリクスの減衰タイプを拡充した（従来は剛性比例・Rayleigh のみ）。

- **モード別減衰:** `C = Σ_i 2·h_i·ω_i·(Mφ_i)(Mφ_i)ᵀ`（質量正規化モード φᵀMφ=1）。
  各モードに独立の減衰比 h_i を与える（`φ_jᵀCφ_j=2h_jω_j`、モード間直交）。
  固有ベクトルは既存の固有値解析（縮約空間・質量正規化）から取得。
- **剛性変更に伴う減衰項の変更（接線剛性比例）:** 非線形時刻歴で瞬間（接線）剛性から
  C を毎ステップ再構成する。**α1 一定**（`C=2h/ω1e·Kt`、`StiffnessProportional{Tangent}`
  を実効化）、**h1 一定**（`ω1=ω1e·√(uᵀKt u/uᵀKe u)` を毎ステップ更新し `C=2h1/ω1·Kt`）。
  Newton ループは元より毎反復で有効剛性を再分解するため、C 再構成の追加コストは小さい。
- **累積型/非累積型:** 減衰力の評価方式を選択（`DampingAccumulation`）。**非累積型**は
  瞬間 C×速度（`{Cn}=[Cn]{ẋn}`、既定）、**累積型**は増分減衰力の積分
  （`{Cn}={Cn−1}+[Cn]{Δẋn}`）。C が変化する接線比例・モード別で両者は異なり、C 一定なら
  一致する。非線形時刻歴 `nonlinear_time_history_analysis` に方式引数を追加。
- **配線:** アプリ時刻歴の減衰セレクタに「モード別／接線(α1一定)／接線(h1一定)」を追加。

**検証:** モード別 C が目標減衰比を再現・モード直交、接線 α1/h1 の C 再構成の手計算照合、
非線形時刻歴で 3 モデルとも収束し有限応答を返す end-to-end テスト、累積型/非累積型が
C 一定で一致し接線比例でも収束する検証（計8件）。

### 8. 制振要素（マクスウェルダンパー）

**対象:** `squid-n-core`（`ElementKind::Damper`・`DamperKind::Maxwell`・`DamperProps`・
`Model::damper_attrs`）、`squid-n-element/src/springs/damper.rs`（`MaxwellDamperElement`）、
`squid-n-element/src/common/behavior.rs`（`set_time_step`）、`squid-n-solver`（時刻歴配線）、
`squid-n-app/src/design_view.rs`（諸元表示）。

RESP-D「07」制振要素の**マクスウェル要素**（バネ `Kd` と粘性ダッシュポットの直列）を実装した。
「減衰要素の要素力は節点力として運動方程式へ与えられる」規定に対応する。

- **マクスウェル要素:** 2 節点軸方向要素。連結点変位 `Ud` を後退 Euler で毎ステップ更新し、
  要素力 `N=Kd(Uij−Ud)` を評価。線形（α=1）は閉形式 `Ud=(C0·Ud_前+Δt·Kd·Uij)/(C0+Δt·Kd)`、
  非線形（α≠1、`Fc=C0·sign(V)·|V|^α`）はスカラー Newton。収束用に整合接線
  `∂N/∂Uij=Kd·C'/(Δt·Kd+C')` を接線剛性へ与える（収束解は要素力の釣合いに一致し原典と等価）。
- **時刻歴配線:** `ElementBehavior::set_time_step(Δt)`（既定 no-op）を追加し、非線形時刻歴が
  build 後に全要素へ Δt を通知。`Δt<=0`（静的・線形）ではダンパーは不活性（力・剛性 0）。
  ダンパー要素は減衰・剛性マトリクス（初期 K/C）へは加算されず、要素力・接線として残差・
  接線側に入る。
- **表示:** 設計ビューに制振ダンパー諸元（種別・Kd・C0・α・緩和時間 τ=C0/Kd）を表示。

**検証:** 要素の不活性（Δt<=0）・高速載荷でのロック→緩和・整合接線の有限差分一致・非線形 α の
釣合い・軸方向力の DOF 配置（要素5件）、SDOF+ダンパー非線形時刻歴で自由振動応答が低減する
end-to-end 検証（1件）。

## 原典照合が必要な埋め込み値（技術リード確認用）

- 武田型トリリニアの材端骨格の簡略値: ひび割れ κ=0.56、降伏時剛性低下率 αy=0.3
  （既定）、終局倍率 1.1、塑性率 4。RESP-D 準拠の菅野 αy 精算はファイバ/skeleton 側
  （`build_rc_member_skeleton`）で行う方針であり、材端集中バネは簡略骨格を用いる。
- 武田型除荷指数 ν（`alpha`）既定 0.4。辻・山田型の既定 β=0.5・K2=0.01·k_rot。
- 累積損傷度の疲労特性 C・β（暫定既定 C=20, β=0.5。鋼種・接合形式で要照合）。

## 未実装項目（今後の課題／資料スコープ外）

RESP-D「07 非線形解析（動的解析）」との照合で確認したが**なお未着手**の主な差異
（規模が大きく、それぞれ別スライスとして対応するのが妥当）:

- **免震支承材の残装置・要素配線:** オイレス Tri-Linear/修正 H-D、錫プラグ、eRB、
  U 型ダンパー等の残装置。§6 は design-jp 純関数として算定式を実装したが、要素
  （`IsolatorElement`）は依然バイリニア/摩擦の汎用モデルで、γ 依存を毎ステップ反映する
  配線と、装置種別・ゴム総厚 H 等の入力（`IsolatorKind`/`IsolatorProps` 拡張）は未実装。
- **制振要素の残装置:** §8 でマクスウェル要素（`ElementKind::Damper`）と時刻歴配線を実装。
  残るは各社ダンパーの製品固有則（SUB、アンボンドブレース、TRC、ユニットゴム修正 HD、
  RDT、筒状流体/粘性体、GRAST 3 要素）。多くは §8 のマクスウェル・§4 の辻山田・
  バイリニア/トリリニア骨格の組合せで表現でき、製品係数を `DamperProps` に拡張して対応する。
  JFE LY トリリニアもここに含む。なお制振ダンパーの GUI 作成フォームは未実装（諸元表示のみ。
  現状は API／IO 経由で作成）。
- **質点系（串団子）モデル:** 3D プッシュオーバー結果を層ごとのトリリニア stick へ
  縮約する新ステージ（等価せん断/曲げせん断型、P-δ、スウェイ・ロッキング、免震考慮）。
- **鉄骨大梁の座屈考慮履歴:** 局部/横/連成座屈の Mu 式（Λc'・h0・qκ・r・αΛ）と RO 除荷
  （γ=5,Φ=0.5）を持つ SteelBuckling 履歴則。
- **位相差入力解析:** せん断波速度・入射角から位相遅れ時間を算定しねじれ加振を生成。
- **累積損傷度の梁端 μ 収集配線:** §5 のアルゴリズムは実装済み。時刻歴ソルバで梁端
  曲げ塑性率 μ の時刻歴を要素状態から収集し `cumulative_ductility` を実値化する配線が残る。
