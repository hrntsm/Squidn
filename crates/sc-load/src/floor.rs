use sc_core::ids::ElemId;
use sc_core::model::{DistributionMethod, Model, Slab};

pub enum LoadShape {
    Uniform { w: f64 },
    Trapezoid { w0: f64, a: f64, b: f64 },
    Triangle { w0: f64 },
    Point { p: f64, x: f64 },
}

pub struct Cmq {
    pub c_i: f64,
    pub c_j: f64,
    pub q_i: f64,
    pub q_j: f64,
}

pub struct BeamLoad {
    pub elem: ElemId,
    pub shape: LoadShape,
    pub cmq: Cmq,
}

/// スラブの面荷重を大梁へ分配する。
/// 正方形スラブの場合、4辺すべてに三角形分配（45°法）。
/// 矩形スラブの場合、短辺方向に三角形、長辺方向に台形。
/// ※ slab.boundary は節点IDの並び。サポート梁の特定にはモデル参照が必要だが、
///   ここでは分配された荷重強度とCMQを計算し、呼び出し側が節点→部材を対応づける。
pub fn distribute_slab(model: &Model, slab: &Slab) -> Vec<BeamLoad> {
    let mut loads = Vec::new();
    let (lx, ly) = slab_dimensions(model, slab);
    if lx <= 0.0 || ly <= 0.0 {
        return loads;
    }

    for area_load in &slab.loads {
        let w = area_load.value;

        match slab.method {
            DistributionMethod::TriTrapezoid => {
                let is_square = (lx - ly).abs() < 1e-6;
                if is_square {
                    let w0 = w * lx / 2.0;
                    for i in 0..4 {
                        let l = if i % 2 == 0 { lx } else { ly };
                        loads.push(BeamLoad {
                            elem: ElemId(i as u32),
                            shape: LoadShape::Triangle { w0 },
                            cmq: fem_triangle(w0, l),
                        });
                    }
                } else {
                    let short = lx.min(ly);
                    let long = lx.max(ly);
                    let w0 = w * short / 2.0;
                    let a = short / 2.0;
                    let b = long - 2.0 * a;

                    for i in 0..4 {
                        let l = if i % 2 == 0 { lx } else { ly };
                        let is_short_side = (l - short).abs() < 1e-6;
                        if is_short_side {
                            loads.push(BeamLoad {
                                elem: ElemId(i as u32),
                                shape: LoadShape::Triangle { w0 },
                                cmq: fem_triangle(w0, l),
                            });
                        } else {
                            loads.push(BeamLoad {
                                elem: ElemId(i as u32),
                                shape: LoadShape::Trapezoid { w0, a, b },
                                cmq: fem_trapezoid(w0, a, b, l),
                            });
                        }
                    }
                }
            }
            DistributionMethod::OneWay => {
                // 一方向スラブ: ly 方向に架け、長さ lx の 2 本の大梁が等分に負担する。
                // 各梁の線荷重 = w·(負担幅 ly/2)。総和 2·(w·ly/2)·lx = w·lx·ly（保存）。
                let w_line = w * ly / 2.0;
                for i in 0..4 {
                    let l = if i % 2 == 0 { lx } else { ly };
                    if (l - lx).abs() < 1e-6 {
                        loads.push(BeamLoad {
                            elem: ElemId(i as u32),
                            shape: LoadShape::Uniform { w: w_line },
                            cmq: fem_uniform(w_line, l),
                        });
                    }
                }
            }
            DistributionMethod::TributaryArea => {
                // 45°負担面積を等価等分布へ換算（総和保存）。
                // 短辺側梁: 三角形負担（面積 S²/4）→ UDL = w·S/4。
                // 長辺側梁: 台形負担（面積 S·Lg/2 − S²/4）→ UDL = w·(…)/Lg。
                let short = lx.min(ly);
                let long = lx.max(ly);
                for i in 0..4 {
                    let l = if i % 2 == 0 { lx } else { ly };
                    let is_short_side = (l - short).abs() <= (l - long).abs();
                    let w_line = if is_short_side {
                        w * short / 4.0
                    } else {
                        w * (short * long / 2.0 - short * short / 4.0) / long
                    };
                    loads.push(BeamLoad {
                        elem: ElemId(i as u32),
                        shape: LoadShape::Uniform { w: w_line },
                        cmq: fem_uniform(w_line, l),
                    });
                }
            }
        }
    }

    loads
}

fn slab_dimensions(model: &Model, slab: &Slab) -> (f64, f64) {
    if slab.boundary.len() < 4 {
        return (0.0, 0.0);
    }
    let p0 = model.nodes.get(slab.boundary[0].index()).map(|n| n.coord);
    let p1 = model.nodes.get(slab.boundary[1].index()).map(|n| n.coord);
    let p3 = model.nodes.get(slab.boundary[3].index()).map(|n| n.coord);
    match (p0, p1, p3) {
        (Some(c0), Some(c1), Some(c3)) => {
            let lx = ((c1[0] - c0[0]).powi(2) + (c1[1] - c0[1]).powi(2) + (c1[2] - c0[2]).powi(2))
                .sqrt();
            let ly = ((c3[0] - c0[0]).powi(2) + (c3[1] - c0[1]).powi(2) + (c3[2] - c0[2]).powi(2))
                .sqrt();
            (lx, ly)
        }
        _ => (0.0, 0.0),
    }
}

fn fem_uniform(w: f64, l: f64) -> Cmq {
    Cmq {
        c_i: w * l * l / 12.0,
        c_j: -w * l * l / 12.0,
        q_i: w * l / 2.0,
        q_j: w * l / 2.0,
    }
}

fn fem_triangle(w0: f64, l: f64) -> Cmq {
    Cmq {
        c_i: 5.0 * w0 * l * l / 96.0,
        c_j: -5.0 * w0 * l * l / 96.0,
        q_i: w0 * l / 4.0,
        q_j: w0 * l / 4.0,
    }
}

/// 対称台形荷重（両端 a 区間で 0→w0 に線形立上り、中央 L−2a 区間は等高 w0）の
/// 両端固定梁の固定端モーメント・せん断。
/// 固定端モーメントは閉形式 FEM = (1/L²)∫₀ᴸ w(x)·x·(L−x)² dx を評価して求める。
/// 検算: a→L/2 で対称三角形 5w0L²/96、a→0 で等分布 w0L²/12 に一致する。
#[allow(unused_variables)]
fn fem_trapezoid(w0: f64, a: f64, b: f64, l: f64) -> Cmq {
    // ∫ x(L-x)² dx の不定積分
    let g = |x: f64| l * l * x * x / 2.0 - 2.0 * l * x * x * x / 3.0 + x.powi(4) / 4.0;
    // 両端の三角形立上り区間（[0,a] と [L-a,L]）の寄与（/a を約分済みの閉形式）
    let i_ends = w0 * l * a * a * (l / 3.0 - a / 4.0);
    // 中央の等分布区間 [a, L-a] の寄与
    let i_mid = w0 * (g(l - a) - g(a));
    let fem = (i_ends + i_mid) / (l * l);
    // 総荷重 = 台形面積（単位幅あたり）= w0·(L−a)。せん断は対称なので両端で W/2。
    let total = w0 * (l - a);
    Cmq {
        c_i: fem,
        c_j: -fem,
        q_i: total / 2.0,
        q_j: total / 2.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fem_uniform() {
        let cmq = fem_uniform(10.0, 4000.0);
        let expected = 10.0 * 4000.0_f64.powi(2) / 12.0;
        assert!((cmq.c_i - expected).abs() < 1e-6);
        assert_eq!(cmq.q_i, 10.0 * 4000.0 / 2.0);
    }

    #[test]
    fn test_fem_triangle_spec() {
        let w0 = 10.0_f64;
        let l = 4000.0_f64;
        let cmq = fem_triangle(w0, l);
        let expected = 5.0 * w0 * l.powi(2) / 96.0;
        assert!(
            (cmq.c_i - expected).abs() < 1e-3,
            "FEM={} expected={}",
            cmq.c_i,
            expected
        );
        assert!((expected - 8.3333e6).abs() < 1.0e3, "expected={}", expected);
    }

    #[test]
    fn test_fem_trapezoid_limits() {
        let w0 = 10.0_f64;
        let l = 6000.0_f64;
        // a→L/2（中央区間消滅）→ 対称三角形 5w0L²/96
        let tri_limit = fem_trapezoid(w0, l / 2.0, 0.0, l);
        let expected_tri = 5.0 * w0 * l.powi(2) / 96.0;
        assert!(
            (tri_limit.c_i - expected_tri).abs() / expected_tri < 1e-9,
            "三角形極限 c_i={} expected={}",
            tri_limit.c_i,
            expected_tri
        );
        // a→0（立上り消滅）→ 等分布 w0L²/12
        let uni_limit = fem_trapezoid(w0, 0.0, l, l);
        let expected_uni = w0 * l.powi(2) / 12.0;
        assert!(
            (uni_limit.c_i - expected_uni).abs() / expected_uni < 1e-9,
            "等分布極限 c_i={} expected={}",
            uni_limit.c_i,
            expected_uni
        );
    }

    #[test]
    fn test_fem_trapezoid_numeric() {
        // 一般の台形を数値積分と照合: FEM = (1/L²)∫ w(x)·x·(L-x)² dx
        let w0 = 7.0_f64;
        let l = 5000.0_f64;
        let a = 1500.0_f64;
        let cmq = fem_trapezoid(w0, a, l - 2.0 * a, l);
        let n = 2_000_000;
        let dx = l / n as f64;
        let mut integral = 0.0;
        let mut total = 0.0;
        for k in 0..n {
            let x = (k as f64 + 0.5) * dx;
            let wx = if x < a {
                w0 * x / a
            } else if x > l - a {
                w0 * (l - x) / a
            } else {
                w0
            };
            integral += wx * x * (l - x).powi(2) * dx;
            total += wx * dx;
        }
        let fem_num = integral / (l * l);
        assert!(
            (cmq.c_i - fem_num).abs() / fem_num < 1e-4,
            "c_i={} 数値積分={}",
            cmq.c_i,
            fem_num
        );
        // せん断 q_i+q_j = 総荷重
        assert!(
            (cmq.q_i + cmq.q_j - total).abs() / total < 1e-4,
            "Q合計={} 総荷重={}",
            cmq.q_i + cmq.q_j,
            total
        );
    }

    fn make_square_slab_model(side: f64, method: DistributionMethod, w: f64) -> (Model, Slab) {
        make_rect_slab_model(side, side, method, w)
    }

    fn make_rect_slab_model(lx: f64, ly: f64, method: DistributionMethod, w: f64) -> (Model, Slab) {
        use sc_core::ids::{NodeId, SlabId};
        use sc_core::model::{AreaLoad, Node};
        let mk = |id: u32, x: f64, y: f64| Node {
            id: NodeId(id),
            coord: [x, y, 0.0],
            restraint: Default::default(),
            mass: None,
            story: None,
        };
        let model = Model {
            nodes: vec![
                mk(0, 0.0, 0.0),
                mk(1, lx, 0.0),
                mk(2, lx, ly),
                mk(3, 0.0, ly),
            ],
            ..Default::default()
        };
        let slab = Slab {
            id: SlabId(0),
            boundary: vec![NodeId(0), NodeId(1), NodeId(2), NodeId(3)],
            joists: vec![],
            loads: vec![AreaLoad {
                kind: "DL".into(),
                value: w,
            }],
            method,
        };
        (model, slab)
    }

    fn total_load(loads: &[BeamLoad]) -> f64 {
        // 鉛直釣合いより、各梁の総荷重 = 端せん断の和 q_i + q_j。
        loads.iter().map(|l| l.cmq.q_i + l.cmq.q_j).sum()
    }

    #[test]
    fn test_slab_conservation_square_triangle() {
        // 設計書 §7.3: 1辺 a=4000, w=0.005 → 総和 = w·a² = 80000 N（厳密）
        let w = 0.005_f64;
        let a = 4000.0_f64;
        let (model, slab) = make_square_slab_model(a, DistributionMethod::TriTrapezoid, w);
        let loads = distribute_slab(&model, &slab);
        let expected = w * a * a;
        assert!(
            (total_load(&loads) - expected).abs() < 1e-6,
            "総和={} expected={}",
            total_load(&loads),
            expected
        );
        // 各大梁ピーク強度 w0 = w·a/2 = 10, FEM = 5·w0·a²/96
        for l in &loads {
            if let LoadShape::Triangle { w0 } = l.shape {
                assert!((w0 - 10.0).abs() < 1e-9, "w0={}", w0);
                let fem = 5.0 * w0 * a * a / 96.0;
                assert!((l.cmq.c_i - fem).abs() < 1e-3, "FEM={}", l.cmq.c_i);
            }
        }
    }

    #[test]
    fn test_slab_conservation_rect_all_methods() {
        let w = 0.005_f64;
        let (lx, ly) = (4000.0_f64, 6000.0_f64);
        let expected = w * lx * ly;
        for method in [
            DistributionMethod::TriTrapezoid,
            DistributionMethod::OneWay,
            DistributionMethod::TributaryArea,
        ] {
            let (model, slab) = make_rect_slab_model(lx, ly, method, w);
            let loads = distribute_slab(&model, &slab);
            assert!(
                (total_load(&loads) - expected).abs() / expected < 1e-9,
                "method={:?} 総和={} expected={}",
                method,
                total_load(&loads),
                expected
            );
        }
    }
}
