use std::fmt::Write as _;
use std::path::Path;

fn append_assets(directory: &Path, root: &Path, generated: &mut String) -> std::io::Result<()> {
    let mut entries = std::fs::read_dir(directory)?.collect::<std::io::Result<Vec<_>>>()?;
    entries.sort_by_key(std::fs::DirEntry::file_name);
    for entry in entries {
        let path = entry.path();
        let file_type = entry.file_type()?;
        if file_type.is_dir() {
            append_assets(&path, root, generated)?;
        } else if file_type.is_file() {
            let relative = path
                .strip_prefix(root)
                .map_err(std::io::Error::other)?
                .to_string_lossy()
                .replace('\\', "/");
            writeln!(
                generated,
                "    ({relative:?}, include_bytes!({:?})),",
                path.to_string_lossy()
            )
            .map_err(std::io::Error::other)?;
        }
    }
    Ok(())
}

fn main() -> std::io::Result<()> {
    let manifest = std::env::var("CARGO_MANIFEST_DIR").map_err(std::io::Error::other)?;
    let web = Path::new(&manifest).join("../src/so101_setup/assets/web");
    println!("cargo:rerun-if-changed={}", web.display());
    if !web.join("index.html").is_file() {
        return Err(std::io::Error::other(
            "Build the bundled UI first with pnpm install and pnpm build",
        ));
    }
    let mut generated = String::from("const ASSETS: &[(&str, &[u8])] = &[\n");
    let web = web.canonicalize()?;
    append_assets(&web, &web, &mut generated)?;
    generated.push_str("];\n");
    let output = std::env::var("OUT_DIR").map_err(std::io::Error::other)?;
    std::fs::write(Path::new(&output).join("assets.rs"), generated)
}
