use crate::config::{MachineConfig, AXES};

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DeviceStatus {
    /// Machine coordinates (mm, degrees for A).
    pub pos: [f64; AXES],
    pub moving: bool,
    /// Free slots in the device motion buffer.
    pub buffer_free: usize,
    /// Id of the move being executed (as passed to `queue_linear`), 0 if idle.
    pub current_id: u64,
    pub estop: bool,
    pub inputs: u64,
}

/// A motion controller board (UC100, simulator, …). Moves are queued; the device plans
/// acceleration and executes them in order.
pub trait MotionDevice: Send {
    fn name(&self) -> String;
    fn connect(&mut self, cfg: &MachineConfig) -> anyhow::Result<()>;
    fn disconnect(&mut self);
    /// Hardware boards come up in reset and need an explicit E-stop reset before moving.
    fn starts_in_estop(&self) -> bool {
        false
    }
    fn poll(&mut self) -> anyhow::Result<DeviceStatus>;
    /// Queue a coordinated linear move to `target` (machine coords). `feed` in mm/min.
    fn queue_linear(&mut self, target: [f64; AXES], feed: f64, rapid: bool, id: u64) -> anyhow::Result<()>;
    /// Stop motion immediately and flush the buffer.
    fn stop(&mut self) -> anyhow::Result<()>;
    fn feed_hold(&mut self, hold: bool) -> anyhow::Result<()>;
    /// Continuous jog; `velocity` in mm/min, signed. 0 stops.
    fn jog(&mut self, axis: usize, velocity: f64) -> anyhow::Result<()>;
    fn set_estop(&mut self, on: bool) -> anyhow::Result<()>;
    fn set_output(&mut self, pin: u32, on: bool) -> anyhow::Result<()>;
    /// Redefine the current machine position (homing / machine zero).
    fn set_position(&mut self, pos: [f64; AXES]) -> anyhow::Result<()>;
}

/// Software device that executes moves in real time at constant velocity.
pub struct SimDevice {
    pos: [f64; AXES],
    queue: std::collections::VecDeque<([f64; AXES], f64, u64)>,
    current: Option<([f64; AXES], f64, u64)>,
    last: std::time::Instant,
    hold: bool,
    estop: bool,
    jog: [f64; AXES],
    rapid: f64,
    capacity: usize,
}

impl Default for SimDevice {
    fn default() -> Self {
        SimDevice {
            pos: [0.0; AXES],
            queue: Default::default(),
            current: None,
            last: std::time::Instant::now(),
            hold: false,
            estop: false,
            jog: [0.0; AXES],
            rapid: 2500.0,
            capacity: 200,
        }
    }
}

impl MotionDevice for SimDevice {
    fn name(&self) -> String {
        "Simulator".into()
    }

    fn connect(&mut self, cfg: &MachineConfig) -> anyhow::Result<()> {
        self.rapid = cfg.rapid_rate;
        self.last = std::time::Instant::now();
        Ok(())
    }

    fn disconnect(&mut self) {}

    fn poll(&mut self) -> anyhow::Result<DeviceStatus> {
        let now = std::time::Instant::now();
        let dt = (now - self.last).as_secs_f64();
        self.last = now;
        if !self.estop && !self.hold {
            for (p, v) in self.pos.iter_mut().zip(self.jog) {
                *p += v / 60.0 * dt;
            }
            let mut budget = dt;
            while budget > 0.0 {
                if self.current.is_none() {
                    self.current = self.queue.pop_front();
                }
                let Some((t, f, _)) = self.current else { break };
                let d: f64 = (0..AXES).map(|i| (t[i] - self.pos[i]).powi(2)).sum::<f64>().sqrt();
                let v = f.max(1.0) / 60.0;
                let need = d / v;
                if need <= budget {
                    self.pos = t;
                    budget -= need;
                    self.current = None;
                } else {
                    let k = budget * v / d;
                    for i in 0..AXES {
                        self.pos[i] += (t[i] - self.pos[i]) * k;
                    }
                    budget = 0.0;
                }
            }
        }
        Ok(DeviceStatus {
            pos: self.pos,
            moving: self.current.is_some() || !self.queue.is_empty() || self.jog.iter().any(|v| *v != 0.0),
            buffer_free: self.capacity.saturating_sub(self.queue.len()),
            current_id: self.current.map(|c| c.2).unwrap_or(0),
            estop: self.estop,
            inputs: 0,
        })
    }

    fn queue_linear(&mut self, target: [f64; AXES], feed: f64, rapid: bool, id: u64) -> anyhow::Result<()> {
        anyhow::ensure!(!self.estop, "E-stop active");
        self.queue.push_back((target, if rapid { self.rapid } else { feed }, id));
        Ok(())
    }

    fn stop(&mut self) -> anyhow::Result<()> {
        self.queue.clear();
        self.current = None;
        self.jog = [0.0; AXES];
        Ok(())
    }

    fn feed_hold(&mut self, hold: bool) -> anyhow::Result<()> {
        self.hold = hold;
        Ok(())
    }

    fn jog(&mut self, axis: usize, velocity: f64) -> anyhow::Result<()> {
        anyhow::ensure!(!self.estop, "E-stop active");
        self.jog[axis] = velocity;
        Ok(())
    }

    fn set_estop(&mut self, on: bool) -> anyhow::Result<()> {
        self.estop = on;
        if on {
            self.stop()?;
        }
        Ok(())
    }

    fn set_output(&mut self, _pin: u32, _on: bool) -> anyhow::Result<()> {
        Ok(())
    }

    fn set_position(&mut self, pos: [f64; AXES]) -> anyhow::Result<()> {
        self.pos = pos;
        Ok(())
    }
}
