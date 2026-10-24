//! STEP (AP203/AP214/AP242) importer on a pinned static Open CASCADE build.
//!
//! Unlike the Rhino path, WarpNine meshes the exact B-rep itself at the requested
//! tolerance and measures the result, so a STEP solid arrives already represented by a
//! qualified machining mesh. All native model allocations stay inside the bridge.
use anyhow::{bail, ensure, Context, Result};
use std::{
    ffi::{CStr, CString},
    os::raw::{c_char, c_int},
    path::Path,
};
use warp_core::interchange::*;
extern "C" {
    fn warpstep_read(
        path: *const c_char,
        tolerance: f64,
        max_triangles: f64,
        cancel: *const u8,
        ok: *mut c_int,
    ) -> *mut c_char;
    fn warpstep_free(p: *mut c_char);
}
/// Open CASCADE keeps global reader/writer state (Interface_Static, the STEP controller),
/// so native calls are serialised process-wide.
static NATIVE: std::sync::Mutex<()> = std::sync::Mutex::new(());
fn native_guard() -> std::sync::MutexGuard<'static, ()> {
    NATIVE.lock().unwrap_or_else(|e| e.into_inner())
}
struct NativeBuffer(*mut c_char);
impl Drop for NativeBuffer {
    fn drop(&mut self) {
        unsafe { warpstep_free(self.0) }
    }
}
pub struct StepImporter;
impl GeometryImporter for StepImporter {
    fn read(&self, path: &Path, options: &ImportOptions) -> Result<ImportResult> {
        options.validate()?;
        let source_file = path
            .canonicalize()
            .context("STEP file not found")?
            .to_string_lossy()
            .into_owned();
        let path = CString::new(source_file.as_str())?;
        let mut ok = 0;
        let _guard = native_guard();
        let buffer = NativeBuffer(unsafe {
            warpstep_read(
                path.as_ptr(),
                options.tolerance_mm,
                options.max_triangles as f64,
                options.cancel_flag(),
                &mut ok,
            )
        });
        ensure!(!buffer.0.is_null(), "Native reader allocation failed");
        let bytes = unsafe { CStr::from_ptr(buffer.0) }.to_bytes();
        if ok == 0 {
            bail!("{}", String::from_utf8_lossy(bytes));
        }
        let raw: NativeScene =
            serde_json::from_slice(bytes).context("Invalid native reader response")?;
        let name = match raw.file_version {
            0 => "STEP".to_string(),
            v => format!("STEP AP{v}"),
        };
        finish_native_import(
            raw,
            options,
            source_file,
            ImportFormat {
                key: "step",
                name,
                remesh_hint: "re-import the STEP file with a tighter import tolerance".into(),
            },
        )
    }
}
/// Writes synthetic acceptance geometry; used by tests.
#[doc(hidden)]
pub fn write_acceptance_fixture(path: &Path, variant: i32) -> Result<()> {
    extern "C" {
        fn warpstep_write_fixture(path: *const c_char, variant: c_int) -> c_int;
    }
    ensure!((0..=1).contains(&variant), "Unsupported fixture variant");
    let path = CString::new(path.to_string_lossy().as_bytes())?;
    let _guard = native_guard();
    ensure!(
        unsafe { warpstep_write_fixture(path.as_ptr(), variant) } == 1,
        "Native fixture writer failed"
    );
    Ok(())
}
