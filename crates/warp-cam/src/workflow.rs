//! Bounded generation and document history, independently testable without a GUI.
use crate::{Operation, Project, Toolpath};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc, Arc,
};
use warp_core::scene::OperationId;
use warp_tools::ToolLibrary;
thread_local! { static ACTIVE_CANCEL: std::cell::RefCell<Option<Arc<AtomicBool>>> = const {std::cell::RefCell::new(None)}; }
pub(crate) fn check_cancel() -> anyhow::Result<()> {
    if ACTIVE_CANCEL.with(|c| {
        c.borrow()
            .as_ref()
            .is_some_and(|c| c.load(Ordering::Relaxed))
    }) {
        anyhow::bail!("Cancelled");
    }
    Ok(())
}

pub fn input_key(
    op: &Operation,
    project: &Project,
    tools: &ToolLibrary,
) -> anyhow::Result<Vec<u8>> {
    // Only selected geometry and this setup/tool affect this operation. Display
    // visibility, object order and unrelated geometry are deliberately excluded.
    let (p, mut o) = project.resolve_operation(op)?;
    o.common.curves = (0..p.curves.len()).collect();
    let setup = project.setups.iter().find(|s| s.id == op.common.setup);
    Ok(serde_json::to_vec(&(
        o,
        p.curves,
        p.mesh,
        p.stock,
        p.execution_boundary,
        p.execution_avoid,
        p.execution_fixtures,
        p.checks,
        tools
            .tool(op.common.tool)
            .and_then(|t| t.holder.as_ref())
            .and_then(|id| tools.holder(id)),
        tools.tool(op.common.tool),
        setup.map(|s| s.wcs),
    ))?)
}

pub struct Job {
    pub id: OperationId,
    pub epoch: u64,
    pub key: Vec<u8>,
    pub op: Operation,
    pub project: Project,
    pub tools: ToolLibrary,
    pub cancelled: Arc<AtomicBool>,
}
pub struct JobResult {
    pub id: OperationId,
    pub epoch: u64,
    pub key: Vec<u8>,
    pub result: Result<Toolpath, String>,
    /// Wall-clock time the generation took.
    pub elapsed: std::time::Duration,
}
pub struct JobQueue {
    tx: mpsc::SyncSender<Job>,
    pub rx: mpsc::Receiver<JobResult>,
}
impl Default for JobQueue {
    fn default() -> Self {
        let (tx, rx) = mpsc::sync_channel::<Job>(32);
        let (out, result) = mpsc::channel();
        std::thread::spawn(move || {
            while let Ok(job) = rx.recv() {
                ACTIVE_CANCEL.with(|c| *c.borrow_mut() = Some(job.cancelled.clone()));
                let started = std::time::Instant::now();
                let r = if job.cancelled.load(Ordering::Relaxed) {
                    Err("Cancelled".into())
                } else {
                    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        crate::generate(&job.op, &job.project, &job.tools)
                    }))
                    .map_err(|_| "Generation failed internally".to_string())
                    .and_then(|r| r.map_err(|e| format!("{e:#}")))
                };
                let r = if job.cancelled.load(Ordering::Relaxed) {
                    Err("Cancelled".into())
                } else {
                    r
                };
                ACTIVE_CANCEL.with(|c| *c.borrow_mut() = None);
                if out
                    .send(JobResult {
                        id: job.id,
                        epoch: job.epoch,
                        key: job.key,
                        result: r,
                        elapsed: started.elapsed(),
                    })
                    .is_err()
                {
                    break;
                }
            }
        });
        Self { tx, rx: result }
    }
}
impl JobQueue {
    pub fn submit(&self, job: Job) -> Result<(), String> {
        self.tx
            .try_send(job)
            .map_err(|_| "Generation queue is full; wait or cancel pending work".into())
    }
}

#[derive(Default)]
pub struct History {
    undo: Vec<String>,
    redo: Vec<String>,
}
impl History {
    pub fn record(&mut self, before: String) {
        if self.undo.last() != Some(&before) {
            self.undo.push(before);
        }
        while self.undo.len() > 32
            || (self.undo.len() > 1
                && self.undo.iter().map(String::len).sum::<usize>() > 64 * 1024 * 1024)
        {
            self.undo.remove(0);
        }
        self.redo.clear();
    }
    pub fn undo(&mut self, current: String) -> Option<String> {
        let p = self.undo.pop()?;
        self.redo.push(current);
        Some(p)
    }
    pub fn redo(&mut self, current: String) -> Option<String> {
        let p = self.redo.pop()?;
        self.undo.push(current);
        Some(p)
    }
    pub fn can_undo(&self) -> bool {
        !self.undo.is_empty()
    }
    pub fn can_redo(&self) -> bool {
        !self.redo.is_empty()
    }
}
