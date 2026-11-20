//! Deterministic state hash (FNV-1a 64) for replay and lockstep desync checks.
//!
//! Self-contained, std only. Callers feed their own fields in a documented, fixed order
//! (for example: tick, then each node's position x,y,z, then each velocity x,y,z).
//! Floats are hashed bit-exact via `to_bits()` little-endian, so `0.0` and `-0.0` hash
//! differently by design. NaN is refused: NaN means the simulation already failed.

const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const PRIME: u64 = 0x0000_0100_0000_01b3;

/// Error from hashing a value that must never reach a state hash.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HashError {
    /// A NaN was fed to the hasher.
    Nan,
}

impl std::fmt::Display for HashError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HashError::Nan => write!(f, "NaN in state hash input (the simulation has failed)"),
        }
    }
}

impl std::error::Error for HashError {}

/// Incremental FNV-1a 64 hasher.
#[derive(Debug, Clone)]
pub struct Hasher {
    state: u64,
}

impl Default for Hasher {
    fn default() -> Self {
        Self::new()
    }
}

impl Hasher {
    pub fn new() -> Self {
        Hasher { state: OFFSET }
    }

    /// Hash raw bytes.
    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.state ^= u64::from(b);
            self.state = self.state.wrapping_mul(PRIME);
        }
    }

    /// Hash a u64 as 8 little-endian bytes.
    pub fn write_u64(&mut self, v: u64) {
        self.write_bytes(&v.to_le_bytes());
    }

    /// Hash an f64's bit pattern (little-endian). Errors on NaN, leaving the hasher unchanged.
    pub fn write_f64(&mut self, v: f64) -> Result<(), HashError> {
        if v.is_nan() {
            return Err(HashError::Nan);
        }
        self.write_u64(v.to_bits());
        Ok(())
    }

    /// Hash each value in order. If a NaN is found, nothing from the slice is hashed.
    pub fn write_slice_f64(&mut self, vs: &[f64]) -> Result<(), HashError> {
        if vs.iter().any(|v| v.is_nan()) {
            return Err(HashError::Nan);
        }
        for &v in vs {
            self.write_u64(v.to_bits());
        }
        Ok(())
    }

    pub fn finish(&self) -> u64 {
        self.state
    }

    /// The hash as 16 lower-case hex digits.
    pub fn hex(&self) -> String {
        format!("{:016x}", self.state)
    }
}
