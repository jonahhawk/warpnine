//! Pinned openNURBS reader. All native model allocations stay inside the bridge.
use anyhow::{bail, ensure, Context, Result};
use std::{
    ffi::{CStr, CString},
    os::raw::{c_char, c_int},
    path::Path,
};
use warp_core::interchange::*;
extern "C" {
    fn warp3dm_read(
        path: *const c_char,
        tolerance: f64,
        max_triangles: f64,
        cancel: *const u8,
        ok: *mut c_int,
    ) -> *mut c_char;
    fn warp3dm_free(p: *mut c_char);
}
struct NativeBuffer(*mut c_char);
impl Drop for NativeBuffer {
    fn drop(&mut self) {
        unsafe { warp3dm_free(self.0) }
    }
}
pub struct OpenNurbsImporter;
impl GeometryImporter for OpenNurbsImporter {
    fn read(&self, path: &Path, options: &ImportOptions) -> Result<ImportResult> {
        options.validate()?;
        let source_file = path
            .canonicalize()
            .context("Rhino file not found")?
            .to_string_lossy()
            .into_owned();
        let path = CString::new(source_file.as_str())?;
        let mut ok = 0;
        let buffer = NativeBuffer(unsafe {
            warp3dm_read(
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
        let version = if raw.file_version >= 50 {
            raw.file_version / 10
        } else {
            raw.file_version
        };
        finish_native_import(
            raw,
            options,
            source_file,
            ImportFormat {
                key: "3dm",
                name: format!("Rhino {version}"),
                remesh_hint: "re-mesh the source in Rhino with a tighter tolerance".into(),
            },
        )
    }
}
/// Writes only synthetic acceptance geometry; used by tests and the fixture generator.
#[doc(hidden)]
pub fn write_acceptance_fixture(path: &Path, variant: i32, version: i32) -> Result<()> {
    extern "C" {
        fn warp3dm_write_fixture(path: *const c_char, variant: c_int, version: c_int) -> c_int;
    }
    ensure!(
        (0..=10).contains(&variant) && [2, 3, 4, 5, 6, 7, 8].contains(&version),
        "Unsupported fixture variant/version"
    );
    let path = CString::new(path.to_string_lossy().as_bytes())?;
    ensure!(
        unsafe { warp3dm_write_fixture(path.as_ptr(), variant, version) } == 1,
        "Native fixture writer failed"
    );
    Ok(())
}
