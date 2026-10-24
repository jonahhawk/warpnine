use std::path::PathBuf;
fn main() -> anyhow::Result<()> {
    let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../fixtures/rhino-m2");
    std::fs::create_dir_all(&root)?;
    for (name, variant, version) in [
        ("parts-mm-v8", 0, 8),
        ("parts-inch-v6", 1, 6),
        ("solids-v8", 4, 8),
    ] {
        let path = root.join(format!("{name}.3dm"));
        warp_import_3dm::write_acceptance_fixture(&path, variant, version)?;
        println!("{}", path.display());
    }
    Ok(())
}
