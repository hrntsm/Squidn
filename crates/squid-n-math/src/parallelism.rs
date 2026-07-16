//! 解析全体の並列度設定（プロセスグローバル）。
//!
//! 設計書 §5.3 / R28 の決定性保証（CPU・単一スレッドでビット一致）を既定としつつ、
//! 速度優先の利用者向けに faer の行列分解・ソルバ内部のスレッド並列と、
//! 荷重ケース単位のバッチ並列（`squid-n-solver` の batch API）を有効化できる。
//!
//! - 既定は [`Parallelism::Deterministic`]（従来どおり単一スレッド・ビット一致）。
//! - 並列モードでは浮動小数の加算順序が変わり得るため、結果は単一スレッドと
//!   ビット一致しない（値としてはほぼ一致する）。決定性が必要な検証・テストは
//!   `Deterministic` を明示すること。
//!
//! ソルバの各エントリポイント（`Analysis::prepare` 等）は計算開始前に
//! [`apply_to_faer`] を呼び、ここで設定された並列度を faer に反映する。

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::{Arc, Mutex, OnceLock};

/// 解析に使うスレッド数の指定。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Parallelism {
    /// 単一スレッド（既定）。同一入力でビット一致の再現性を保証する（R28）。
    Deterministic,
    /// 使用可能な全コアを使う。
    Auto,
    /// スレッド数を明示指定する（0 は `Auto`、1 は `Deterministic` と同義）。
    Threads(usize),
}

impl Parallelism {
    /// GUI 等の「スレッド数」整数入力から変換する（0=自動、1=単一、n=固定）。
    pub fn from_threads(n: usize) -> Self {
        match n {
            0 => Parallelism::Auto,
            1 => Parallelism::Deterministic,
            n => Parallelism::Threads(n),
        }
    }
}

/// 内部表現: 1=Deterministic（既定）、0=Auto、n>1=Threads(n)。
static PAR: AtomicUsize = AtomicUsize::new(1);

/// `Threads(n)` 用に構築した rayon プールのキャッシュ（スレッド数, プール）。
/// 設定変更時のみ再構築する。`Auto` は rayon グローバルプールを使うため不要。
#[allow(clippy::type_complexity)]
static POOL: OnceLock<Mutex<Option<(usize, Arc<rayon::ThreadPool>)>>> = OnceLock::new();

/// 並列度を設定し、faer のグローバル並列設定へ即時反映する。
pub fn set_parallelism(p: Parallelism) {
    let v = match p {
        Parallelism::Deterministic => 1,
        Parallelism::Auto => 0,
        Parallelism::Threads(n) => n, // 0/1 はそのまま Auto/Deterministic 扱い
    };
    PAR.store(v, Ordering::Relaxed);
    apply_to_faer();
}

/// 現在の並列度設定を返す。
pub fn parallelism() -> Parallelism {
    match PAR.load(Ordering::Relaxed) {
        1 => Parallelism::Deterministic,
        0 => Parallelism::Auto,
        n => Parallelism::Threads(n),
    }
}

/// 並列実行が有効か（バッチ API が rayon 経路を使うかどうかの判定）。
pub fn is_parallel() -> bool {
    !matches!(parallelism(), Parallelism::Deterministic)
}

/// 現在の設定で使うスレッド数の実効値を返す。
/// `Auto` は使用可能な並列数（`available_parallelism`）に解決する。
/// バッチ API がケース並列とソルバ内部並列へスレッドを配分する際の総枠になる。
pub fn effective_threads() -> usize {
    match parallelism() {
        Parallelism::Deterministic => 1,
        Parallelism::Auto => std::thread::available_parallelism()
            .map(|n| n.get())
            .unwrap_or(1),
        Parallelism::Threads(n) => n.max(1),
    }
}

/// 現在の設定を faer のグローバル並列設定へ反映する。
/// ソルバの各エントリポイントの先頭で呼ぶ（従来の
/// `faer::set_global_parallelism(faer::Par::Seq)` 固定呼び出しの置き換え）。
pub fn apply_to_faer() {
    let par = match parallelism() {
        Parallelism::Deterministic => faer::Par::Seq,
        // Par::rayon(0) は rayon グローバルプールの全スレッドを使う。
        Parallelism::Auto => faer::Par::rayon(0),
        Parallelism::Threads(n) => faer::Par::rayon(n),
    };
    faer::set_global_parallelism(par);
}

/// 設定された並列度のスレッドプール上でクロージャを実行する。
/// バッチ API（荷重ケース単位の並列）が `par_iter` をこの中で呼ぶことで、
/// `Threads(n)` 指定時にケース並列の同時実行数も n に制限される。
/// `Deterministic` では呼び出し元スレッドでそのまま実行する（並列化しない）。
pub fn run<R: Send>(f: impl FnOnce() -> R + Send) -> R {
    match parallelism() {
        Parallelism::Deterministic | Parallelism::Auto => f(),
        Parallelism::Threads(n) => {
            let mutex = POOL.get_or_init(|| Mutex::new(None));
            let pool = {
                let mut guard = mutex.lock().expect("並列プールのロックに失敗");
                match guard.as_ref() {
                    Some((threads, pool)) if *threads == n => Arc::clone(pool),
                    _ => {
                        let pool = Arc::new(
                            rayon::ThreadPoolBuilder::new()
                                .num_threads(n)
                                .build()
                                .expect("rayon スレッドプールの構築に失敗"),
                        );
                        *guard = Some((n, Arc::clone(&pool)));
                        pool
                    }
                }
            };
            pool.install(f)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 並列度設定はプロセスグローバルのため、順序依存を避けて1テストに集約する。
    #[test]
    fn test_parallelism_roundtrip_and_run() {
        // 既定は Deterministic
        assert_eq!(parallelism(), Parallelism::Deterministic);
        assert!(!is_parallel());

        set_parallelism(Parallelism::Auto);
        assert_eq!(parallelism(), Parallelism::Auto);
        assert!(is_parallel());

        set_parallelism(Parallelism::Threads(2));
        assert_eq!(parallelism(), Parallelism::Threads(2));
        // Threads(n) の run は n スレッドのプールで実行される
        let n = run(rayon::current_num_threads);
        assert_eq!(n, 2);

        // 0/1 は Auto / Deterministic に正規化される
        assert_eq!(Parallelism::from_threads(0), Parallelism::Auto);
        assert_eq!(Parallelism::from_threads(1), Parallelism::Deterministic);
        assert_eq!(Parallelism::from_threads(4), Parallelism::Threads(4));
        set_parallelism(Parallelism::Threads(1));
        assert_eq!(parallelism(), Parallelism::Deterministic);

        // 後続テストの決定性を壊さないよう既定へ戻す
        set_parallelism(Parallelism::Deterministic);
        assert!(!is_parallel());
    }
}
