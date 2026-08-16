//! Deterministic pseudo-noise shared by the HIL sensor sims.
//!
//! Pure functions of an integer seed — no RNG state, no `rand` crate — so a
//! given build replays the exact same noise sequence, keeping HIL runs
//! reproducible. The baro and IMU sims draw from separate seed streams; the
//! avalanche hash decorrelates distinct seeds, so no channel's "sensor noise"
//! is secretly another channel's.

/// Deterministic per-sample pseudo-noise with ~unit variance, roughly Gaussian
/// in [-3, 3].
///
/// Sums three decorrelated uniform[-1,1] hashed draws: by the CLT this is
/// Gaussian-ish with std = sqrt(3) * (1/sqrt(3)) = 1.0, so a caller's sigma
/// constant reads directly as sigma.
pub fn hash_noise(seed: u32) -> f32 {
    let a = hash_unit(seed.wrapping_mul(0x9E37_79B1));
    let b = hash_unit(seed.wrapping_mul(0x85EB_CA77).wrapping_add(0x1656_67B1));
    let c = hash_unit(seed.wrapping_mul(0xC2B2_AE3D).wrapping_add(0x27D4_EB2F));
    a + b + c
}

/// Integer avalanche hash → f32 in [-1, 1].
fn hash_unit(mut x: u32) -> f32 {
    x ^= x >> 16;
    x = x.wrapping_mul(0x7FEB_352D);
    x ^= x >> 15;
    x = x.wrapping_mul(0x846C_A68B);
    x ^= x >> 16;
    (x as f32 / u32::MAX as f32) * 2.0 - 1.0
}
