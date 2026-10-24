//! Background, cancellable nesting runs. A result is only meaningful if its input key
//! still matches the job it was started from; the caller checks that before applying.

use crate::footprint::Footprint;
use crate::solver::{solve, Layout};
use crate::{NestJob, PartId};
use std::collections::HashMap;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    mpsc, Arc,
};

/// Everything that influences a solve: parts, sheets, constraints, locked placements and
/// the footprints. Unlocked placements are deliberately excluded (they are replaced).
pub fn input_key(job: &NestJob, footprints: &HashMap<PartId, Footprint>) -> Vec<u8> {
    let mut locked: Vec<_> = job.placements.iter().filter(|p| p.locked).collect();
    locked.sort_by(|a, b| a.instance.cmp(&b.instance));
    let mut fps: Vec<_> = footprints
        .iter()
        .map(|(id, f)| (id.0.as_str(), &f.outline.pts, f.source_offset, f.source_z))
        .collect();
    fps.sort_by(|a, b| a.0.cmp(b.0));
    let mut out = Vec::new();
    // serde_json is not a dependency; a stable Debug rendering of plain data suffices.
    out.extend_from_slice(
        format!("{:?}", (&job.parts, &job.sheets, &job.constraints, locked)).as_bytes(),
    );
    out.extend_from_slice(format!("{fps:?}").as_bytes());
    out
}

pub struct NestTask {
    pub epoch: u64,
    pub key: Vec<u8>,
    pub job: NestJob,
    pub footprints: HashMap<PartId, Footprint>,
    pub cancelled: Arc<AtomicBool>,
}
pub struct NestOutcome {
    pub epoch: u64,
    pub key: Vec<u8>,
    pub layout: Layout,
}
pub struct NestQueue {
    tx: mpsc::SyncSender<NestTask>,
    pub rx: mpsc::Receiver<NestOutcome>,
}
impl Default for NestQueue {
    fn default() -> Self {
        let (tx, rx) = mpsc::sync_channel::<NestTask>(4);
        let (out, result) = mpsc::channel();
        std::thread::spawn(move || {
            while let Ok(task) = rx.recv() {
                let cancelled = task.cancelled.clone();
                let layout = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    solve(&task.job, &task.footprints, &|| {
                        cancelled.load(Ordering::Relaxed)
                    })
                }))
                .unwrap_or_else(|_| Layout {
                    placements: task
                        .job
                        .placements
                        .iter()
                        .filter(|p| p.locked)
                        .cloned()
                        .collect(),
                    unplaced: task
                        .job
                        .instances()
                        .into_iter()
                        .filter(|i| {
                            !task
                                .job
                                .placements
                                .iter()
                                .any(|p| p.locked && p.instance == i.id)
                        })
                        .map(|i| crate::solver::Unplaced {
                            instance: i.id,
                            reason: "nesting failed internally".into(),
                        })
                        .collect(),
                    score: Default::default(),
                    cancelled: false,
                });
                if out
                    .send(NestOutcome {
                        epoch: task.epoch,
                        key: task.key,
                        layout,
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
impl NestQueue {
    pub fn submit(&self, task: NestTask) -> Result<(), String> {
        self.tx
            .try_send(task)
            .map_err(|_| "Nesting queue is full; wait or cancel the running nest".into())
    }
}
