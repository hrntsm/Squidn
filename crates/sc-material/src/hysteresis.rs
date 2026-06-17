/// 部材レベルの履歴則。
/// ファイバ要素（P5）は一軸材料（uniaxial）の積分で履歴を作るので、
/// こちらは集中ばね（one/two-component）系で使う。
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum HysteresisRule {
    /// 武田モデル（剛性低下型トリリニア）。
    Takeda {
        /// ひび割れ点 (Mc, θc)
        crack: (f64, f64),
        /// 降伏点 (My, θy)
        yield_point: (f64, f64),
        /// 終局点 (Mu, θu)
        ultimate: (f64, f64),
        /// 除荷剛性低下指数 α（代表 0.4〜0.5）
        alpha: f64,
    },
    /// 武田モデル劣化版。
    TakedaDegrading {
        crack: (f64, f64),
        yield_point: (f64, f64),
        ultimate: (f64, f64),
        alpha: f64,
        /// 劣化率
        degradation: f64,
    },
    /// 原点指向型（せん断）。
    OriginOriented {
        /// 降伏点 (Qy, δy)
        yield_point: (f64, f64),
        /// 終局点 (Qu, δu)
        ultimate: (f64, f64),
    },
    /// スリップ型（せん断）。
    Slip {
        yield_point: (f64, f64),
        ultimate: (f64, f64),
        /// スリップ量係数
        slip_factor: f64,
    },
}

impl HysteresisRule {
    /// 除荷剛性（降伏後）を計算する。
    /// Ku = Ky · (θm / θy)^(−α)
    /// Ky = 降伏点割線剛性 = My / θy
    /// θm = 最大経験変形
    pub fn unloading_stiffness(&self, max_deformation: f64) -> Option<f64> {
        match *self {
            HysteresisRule::Takeda {
                yield_point: (my, theta_y),
                alpha,
                ..
            }
            | HysteresisRule::TakedaDegrading {
                yield_point: (my, theta_y),
                alpha,
                ..
            } => {
                if theta_y.abs() < 1e-15 {
                    return None;
                }
                let ky = my / theta_y;
                let ratio = (max_deformation.abs() / theta_y).max(1.0);
                Some(ky * ratio.powf(-alpha))
            }
            HysteresisRule::OriginOriented { .. } | HysteresisRule::Slip { .. } => None,
        }
    }
}
