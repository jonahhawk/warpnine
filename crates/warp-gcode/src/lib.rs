//! G-code parsing and interpretation.
//!
//! [`parse`] turns text into [`Block`]s; [`Interpreter`] runs blocks through a modal state machine
//! and produces machine-coordinate [`Motion`]s that both the simulator and the controller consume.

mod interp;
mod parse;

pub use interp::{CoolantState, Interpreter, MachineState, Motion, MotionKind, Plane, Program, SpindleState};
pub use parse::{parse, parse_line, Block, Word};

#[derive(Debug, thiserror::Error, Clone, PartialEq)]
pub enum GcodeError {
    #[error("line {line}: {msg}")]
    Parse { line: usize, msg: String },
    #[error("line {line}: {msg}")]
    Semantic { line: usize, msg: String },
}
