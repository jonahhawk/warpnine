use std::{env, path::PathBuf, process::Command};
fn run(c: &mut Command) {
    assert!(
        c.status()
            .expect("Could not start native build tool")
            .success(),
        "Native Open CASCADE bridge build failed"
    );
}
fn main() {
    let root = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let install = root.join("../../vendor/occt-install");
    let pin = std::fs::read_to_string(root.join("../../vendor/occt-pin.txt")).unwrap();
    let built = std::fs::read_to_string(install.join("warpnine-occt-pin.txt")).unwrap_or_default();
    assert!(
        install.join("inc/STEPControl_Reader.hxx").exists() && built.trim() == pin.trim(),
        "Run scripts/fetch-occt.ps1 before building (pinned OCCT {} is not installed)",
        pin.trim()
    );
    let lib_dir = install.join("win64/vc14/lib");
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
    let clean = |p: &PathBuf| {
        p.canonicalize()
            .unwrap()
            .to_string_lossy()
            .trim_start_matches(r"\\?\")
            .replace('\\', "/")
    };
    run(Command::new(&cmake)
        .arg("-S")
        .arg(root.join("native"))
        .arg("-B")
        .arg(&out)
        .arg(format!("-DOCCT_INCLUDE={}", clean(&install.join("inc"))))
        .arg("-DCMAKE_BUILD_TYPE=Release"));
    run(Command::new(&cmake).arg("--build").arg(&out).args([
        "--config",
        "Release",
        "--target",
        "warpstep",
        "--parallel",
        "4",
    ]));
    for p in [out.join("Release"), out.clone(), lib_dir.clone()] {
        println!("cargo:rustc-link-search=native={}", p.display());
    }
    println!("cargo:rustc-link-lib=static=warpstep");
    let mut libs: Vec<String> = std::fs::read_dir(&lib_dir)
        .expect("OCCT lib dir")
        .filter_map(|e| e.ok())
        .map(|e| e.file_name().to_string_lossy().into_owned())
        .filter(|n| n.starts_with("TK") && n.ends_with(".lib"))
        .map(|n| n.trim_end_matches(".lib").to_string())
        .collect();
    libs.sort();
    for l in libs {
        println!("cargo:rustc-link-lib=static={l}");
    }
    if env::var("CARGO_CFG_TARGET_OS").unwrap() == "windows" {
        for l in ["user32", "advapi32", "gdi32", "ole32", "shell32", "ws2_32"] {
            println!("cargo:rustc-link-lib={l}");
        }
    } else {
        panic!("The STEP native build is currently qualified for Windows/MSVC only");
    }
    println!("cargo:rerun-if-changed=native");
    println!("cargo:rerun-if-changed=../../vendor/occt-pin.txt");
    println!("cargo:rerun-if-env-changed=CMAKE");
}
