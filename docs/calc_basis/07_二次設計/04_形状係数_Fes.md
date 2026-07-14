# 7.4 形状係数 Fes（剛性率 Fs・偏心率 Fe）

## 7.4.1 剛性率 Rs・Fs

建築基準法施行令 第82条の6 と告示第1792号に基づき、剛性率 Rs（規定は \\( R_s \ge 0.6 \\)）から Fs を算定する。

**算定式**：\\( K_s = h/\delta \\)、\\( R_s = K_s/\mathrm{mean}(K_s) \\)。\\( F_s = \begin{cases} 1.0 & (R_s \ge 0.6) \\\\ 2.0 - R_s/0.6 & (R_s < 0.6) \end{cases} \\)。

**実装**：`holding_capacity::{stiffness_ratios, fs}` が算定する。剛性率用の層間変位には重心変位 δg
（`secondary::stiffness_ratio::cog_story_drifts`）を用いる。

**整合性**：Fs 式は[原典照合リスト](https://github.com/hrntsm/squid-n/blob/main/specs/原典照合リスト.md)に告示1792号として登録している（☐）。

## 7.4.2 偏心率 Re・Fe

建築基準法施行令 第82条の6 と告示第1792号に基づき、偏心率 Re（規定は \\( R_e \le 0.15 \\)）から Fe を算定する。
剛心と弾力半径は武藤 D 値法で求める。

**算定式**
- D 値 \\( D = a \cdot K_{c0} \\)、\\( K_{c0} = 12EI_c/h^3 \\)、\\( a = \bar{k}/(2+\bar{k}) \\)（一般階）
- 剛心 \\( X_s = \sum(D_y \cdot x)/\sum D_y \\)、偏心距離 \\( e_x = |X_g - X_s| \\)
- 弾力半径 \\( r_{ex} = \sqrt{K_R/\sum D_x} \\)、\\( K_R = \sum(D_x \cdot \bar{y}^2) + \sum(D_y \cdot \bar{x}^2) \\)
- \\( R_e = e/r_e \\)、\\( F_e = \begin{cases} 1.0 & (R_e \le 0.15) \\\\ \min(1.0 + 0.5(R_e - 0.15)/0.15, 1.5) & (R_e > 0.15) \end{cases} \\)

**実装**：`secondary::eccentricity`（D 値法略算）と `secondary::eccentricity_analysis`（応力解析結果からの精算層。剛心は解析剛性、重心は長期軸力による）、および `holding_capacity::fe` が算定する。

**整合性**：doc に令82条の6、告示1792号、武藤 D 値法を明記している。
Fe の上限を 1.5 で頭打ちにする扱いは[原典照合リスト 優先度3](https://github.com/hrntsm/squid-n/blob/main/specs/原典照合リスト.md#優先度3fes形状係数の頭打ち挙動)で要照合である（☐）。
偏心率は弾性剛性（弾性解析結果）に基づいて算定する（令82条の6 の運用に基づく実務的取扱い）。

## 7.4.3 形状係数 Fes

形状係数 Fes は、剛性率による Fs と偏心率による Fe の積として算定する。

**算定式**：\\( F_{es} = F_s \cdot F_e \\)。

**実装**：`holding_capacity::fes` が算定する。
