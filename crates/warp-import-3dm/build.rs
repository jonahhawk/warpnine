use std::{env, path::PathBuf, process::Command};
fn run(c: &mut Command) {
    assert!(
        c.status()
            .expect("Could not start native build tool")
            .success(),
        "Native openNURBS build failed"
    );
}
fn main() {
    let root = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let sdk = root.join("../../vendor/opennurbs");
    assert!(
        sdk.join("opennurbs.h").exists(),
        "Run scripts/fetch-opennurbs.ps1 before building"
    );
    let pin = std::fs::read_to_string(root.join("../../vendor/opennurbs-pin.txt")).unwrap();
    let head = Command::new("git")
        .arg("-C")
        .arg(&sdk)
        .args(["rev-parse", "HEAD"])
        .output()
        .expect("git required to verify SDK pin");
    assert!(
        head.status.success() && String::from_utf8_lossy(&head.stdout).trim() == pin.trim(),
        "openNURBS revision mismatch; run scripts/fetch-opennurbs.ps1"
    );
    let out = PathBuf::from(env::var("OUT_DIR").unwrap()).join("native");
    let cmake = env::var("CMAKE").unwrap_or_else(|_| {
        let vs =
            PathBuf::from(r"C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe");
        if let Ok(o) = Command::new(vs)
            .args(["-latest", "-products", "*", "-property", "installationPath"])
            .output()
        {
            let p = PathBuf::from(String::from_utf8_lossy(&o.stdout).trim())
                .join("Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe");
            if p.exists() {
                return p.to_string_lossy().into_owned();
            }
        }
        "cmake".into()
    });
    run(Command::new(&cmake)
        .arg("-S")
        .arg(root.join("native"))
        .arg("-B")
        .arg(&out)
        .arg(format!(
            "-DOPENNURBS_SOURCE={}",
            sdk.canonicalize()
                .unwrap()
                .to_string_lossy()
                .trim_start_matches(r"\\?\")
                .replace("\\", "/")
        ))
        .arg("-DCMAKE_BUILD_TYPE=Release"));
    run(Command::new(&cmake).arg("--build").arg(&out).args([
        "--config",
        "Release",
        "--target",
        "warp3dm",
        "--parallel",
        "4",
    ]));
    for p in [
        out.join("Release"),
        out.clone(),
        out.join("sdk/Release"),
        out.join("sdk"),
        out.join("sdk/zlib/Release"),
        out.join("sdk/zlib"),
    ] {
        println!("cargo:rustc-link-search=native={}", p.display());
    }
    for l in ["warp3dm", "opennurbsStatic", "zlib"] {
        println!("cargo:rustc-link-lib=static={l}");
    }
    if env::var("CARGO_CFG_TARGET_OS").unwrap() == "windows" {
        for l in [
            "shell32", "ole32", "shlwapi", "usp10", "rpcrt4", "gdi32", "user32", "advapi32",
        ] {
            println!("cargo:rustc-link-lib={l}");
        }
    } else {
        panic!("M2 native build is currently qualified for Windows/MSVC only");
    }
    println!("cargo:rerun-if-changed=native");
    println!("cargo:rerun-if-changed=../../vendor/opennurbs-pin.txt");
    println!("cargo:rerun-if-env-changed=CMAKE");
}
