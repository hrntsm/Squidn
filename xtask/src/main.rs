use std::collections::HashMap;
use std::path::Path;

fn main() -> anyhow::Result<()> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 || args[1] != "check-deps" {
        eprintln!("Usage: cargo run -p xtask -- check-deps");
        std::process::exit(1);
    }

    // レイヤ順（下が下流＝依存される側）。実依存グラフ（DAG）に一致させる。
    // 上位→下位（dep_layer < layer_idx）のみ許可。同層・下位→上位は不許可。
    let layers: &[&[&str]] = &[
        &["sc-core", "sc-math", "sc-material", "sc-ml"],
        &["sc-edit", "sc-section", "sc-load", "sc-gpu"],
        &["sc-skeleton"],
        &["sc-element"],
        &["sc-solver", "sc-io"],
        &["sc-design-jp"],
        &["sc-mcp", "sc-app"],
    ];

    let layer_map: HashMap<&str, usize> = layers
        .iter()
        .enumerate()
        .flat_map(|(i, names)| names.iter().map(move |&n| (n, i)))
        .collect();

    let workspace_root = Path::new(env!("CARGO_MANIFEST_DIR")).parent().unwrap();
    let crate_root = workspace_root.join("crates");

    let mut errors = Vec::new();

    for (name, &layer_idx) in &layer_map {
        // Only check crates/ subdir
        if *name == "xtask" {
            continue;
        }
        let cargo_toml = crate_root.join(name).join("Cargo.toml");
        if !cargo_toml.exists() {
            continue;
        }
        let content = std::fs::read_to_string(&cargo_toml)?;
        let parsed: toml::Value = content.parse()?;

        if let Some(deps) = parsed.get("dependencies").and_then(|d| d.as_table()) {
            for (dep_name, _) in deps {
                if let Some(&dep_layer) = layer_map.get(dep_name.as_str()) {
                    if dep_layer < layer_idx {
                        errors.push(format!(
                            "OK: {} (layer {}) depends on {} (layer {})",
                            name, layer_idx, dep_name, dep_layer
                        ));
                    } else {
                        errors.push(format!(
                            "VIOLATION: {} (layer {}) depends on DOWNSTREAM {} (layer {})",
                            name, layer_idx, dep_name, dep_layer
                        ));
                    }
                }
            }
        }

        if let Some(deps) = parsed.get("dev-dependencies").and_then(|d| d.as_table()) {
            for (dep_name, _) in deps {
                if let Some(&dep_layer) = layer_map.get(dep_name.as_str()) {
                    if dep_layer < layer_idx {
                        errors.push(format!(
                            "OK: {} (layer {}) dev-depends on {} (layer {})",
                            name, layer_idx, dep_name, dep_layer
                        ));
                    } else {
                        errors.push(format!(
                            "VIOLATION: {} (layer {}) dev-depends on DOWNSTREAM {} (layer {})",
                            name, layer_idx, dep_name, dep_layer
                        ));
                    }
                }
            }
        }
    }

    let violations: Vec<_> = errors
        .iter()
        .filter(|e| e.starts_with("VIOLATION"))
        .collect();
    if !violations.is_empty() {
        for v in &violations {
            eprintln!("{}", v);
        }
        anyhow::bail!(
            "Dependency direction check failed with {} violation(s)",
            violations.len()
        );
    }

    println!(
        "All dependency directions OK ({} upstream checks)",
        errors.len()
    );
    Ok(())
}
