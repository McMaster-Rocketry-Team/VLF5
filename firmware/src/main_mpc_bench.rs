//! MPC solver micro-benchmark (not flight firmware).
//!
//! The airbrakes MPC measured 10.6 ms mean / 20.9 ms max per solve in the
//! 2026-08-17 `hil-dual` run — ~4000 cycles per derivative evaluation at
//! 480 MHz, which is far more than the handful of flops in
//! `calculate_state_derivatives` should cost. This bench splits that number
//! into its parts on the real silicon:
//!
//! * how much of it is `libm::powf` inside `approximate_air_density`,
//! * how much a transcendental-free polynomial density would save,
//! * how much is the `opt-level = "z"` size build rather than the math.
//!
//! The RK2 walk below is a faithful copy of
//! `air-brakes-controller-core/src/controller/rocket_dynamics.rs` so the
//! density function can be swapped at monomorphization time. `mpc-real`
//! measures the actual `AirBrakesMPC::update`, and `copy-libm` should land on
//! top of it — that agreement is what makes `copy-poly` trustworthy.
//!
//! Run:
//! ```text
//! cargo run --release --bin mpc_bench -- --probe 0483:374b:<vlf5>
//! ```

#![no_std]
#![no_main]
#![feature(core_intrinsics)]
#![allow(internal_features)]

use {defmt_rtt_pipe as _, panic_probe as _};

use core::hint::black_box;
use cortex_m_rt::entry;
use defmt::info;
use embassy_time::Instant;
use nalgebra::Vector2;

use air_brakes_controller_core::{AirBrakesMPC, RocketParameters};

mod clock_config;

const SYSCLK_MHZ: u64 = 480;

// --- density candidates ------------------------------------------------------

/// Current flight code, verbatim.
#[inline(always)]
fn density_libm_i(altitude_asl: f32) -> f32 {
    1.225 * libm::powf((1.0 - 2.25577e-5 * altitude_asl).max(0.0), 4.256)
}

/// Degree-4 least-squares fit of the same ISA curve in the dimensionless
/// u = 2.25577e-5*h, valid over h in [-1000, 12000] m. Max relative error
/// 1.6e-6 against the exact f64 formula (f32 Horner included), i.e. f32
/// round-off. No transcendentals, no branches, no table walk.
#[inline(always)]
fn density_poly_i(altitude_asl: f32) -> f32 {
    let u = (2.25577e-5f32 * altitude_asl).clamp(-0.03, 0.28);
    ((((1.936_849_5f32) * u - 6.368_716) * u + 8.486_722) * u - 5.213_590) * u + 1.225_000_4
}

#[inline(never)]
fn density_libm(h: f32) -> f32 {
    density_libm_i(h)
}
#[inline(never)]
fn density_poly(h: f32) -> f32 {
    density_poly_i(h)
}
#[inline(never)]
fn density_noop(h: f32) -> f32 {
    h
}
#[inline(never)]
fn sqrtf_probe(h: f32) -> f32 {
    libm::sqrtf(h)
}

/// The FPv5-SP unit on this M7 (target features `+vfp4d16sp`) has a
/// single-instruction square root; `libm::sqrtf` is a software implementation
/// that never reaches it. `core` exposes the instruction as an intrinsic.
#[inline(always)]
fn sqrt_hw_i(x: f32) -> f32 {
    unsafe { core::intrinsics::sqrtf32(x) }
}

#[inline(never)]
fn sqrt_hw(h: f32) -> f32 {
    sqrt_hw_i(h)
}

// --- faithful copy of rocket_dynamics, generic over the density fn -----------

const DT: f32 = 0.1;
const MAX_APOGEE_STEPS: usize = 2000;

#[derive(Clone)]
struct St {
    altitude_asl: f32,
    velocity: Vector2<f32>,
}

/// A kernel is the two swappable pieces of the inner loop: the density model,
/// and how the drag acceleration vector is formed. The second matters because
/// `-v.normalize() * (k * v²)` costs a software sqrt plus two divides, while
/// the algebraically identical `v * (-k * |v|)` costs one hardware sqrt and a
/// multiply.
trait Kernel {
    fn rho(h: f32) -> f32;
    fn drag_accel(v: Vector2<f32>, speed_sq: f32, k: f32) -> Vector2<f32>;
}

/// Exactly what flies today.
struct KCurrent;
impl Kernel for KCurrent {
    #[inline(always)]
    fn rho(h: f32) -> f32 {
        density_libm_i(h)
    }
    #[inline(always)]
    fn drag_accel(v: Vector2<f32>, speed_sq: f32, k: f32) -> Vector2<f32> {
        -v.normalize() * (k * speed_sq)
    }
}

/// Polynomial density only — isolates the `powf` win.
struct KPoly;
impl Kernel for KPoly {
    #[inline(always)]
    fn rho(h: f32) -> f32 {
        density_poly_i(h)
    }
    #[inline(always)]
    fn drag_accel(v: Vector2<f32>, speed_sq: f32, k: f32) -> Vector2<f32> {
        -v.normalize() * (k * speed_sq)
    }
}

/// Polynomial density + hardware sqrt + no divides.
struct KFast;
impl Kernel for KFast {
    #[inline(always)]
    fn rho(h: f32) -> f32 {
        density_poly_i(h)
    }
    #[inline(always)]
    fn drag_accel(v: Vector2<f32>, speed_sq: f32, k: f32) -> Vector2<f32> {
        v * (-k * sqrt_hw_i(speed_sq))
    }
}

#[inline(always)]
fn lerp2(t: f32, values: &[f32]) -> f32 {
    let len = values.len();
    let spacing = 1.0f32 / ((len - 1) as f32);
    let mut i = (t / spacing) as usize;
    if i > len - 2 {
        i = len - 2;
    }
    let t = (t - spacing * (i as f32)) * (len - 1) as f32;
    (1.0 - t) * values[i] + t * values[i + 1]
}

#[inline(always)]
fn cd_from_dp(p: &RocketParameters, drag_percentage: f32) -> f32 {
    lerp2(
        (drag_percentage + 1.0) / 2.0,
        &[p.cd[0], p.cd[p.cd.len() - 1]],
    )
}

#[inline(always)]
fn derivatives<K: Kernel>(dp: f32, state: &St, p: &RocketParameters) -> St {
    let air_density = K::rho(state.altitude_asl);
    let speed_squared = state.velocity.magnitude_squared();
    let cd = cd_from_dp(p, dp);
    let k = 0.5 * cd * air_density * p.reference_area / p.burnout_mass;
    let mut acceleration = if speed_squared > 1e-6 {
        K::drag_accel(state.velocity, speed_squared, k)
    } else {
        Vector2::zeros()
    };
    acceleration.y -= 9.81;
    St {
        altitude_asl: state.velocity.y,
        velocity: acceleration,
    }
}

fn simulate_apogee<K: Kernel>(first_dp: f32, initial: &St, p: &RocketParameters) -> f32 {
    if !initial.altitude_asl.is_finite()
        || !initial.velocity.x.is_finite()
        || !initial.velocity.y.is_finite()
    {
        return 0.0;
    }
    if initial.velocity.y <= 0.0 {
        return initial.altitude_asl;
    }
    let mut state = initial.clone();
    for step_index in 0..MAX_APOGEE_STEPS {
        let dp = match step_index {
            0 => first_dp,
            1 => first_dp / 2.0,
            _ => 0.0,
        };
        let k1 = derivatives::<K>(dp, &state, p);
        let mid = St {
            altitude_asl: state.altitude_asl + k1.altitude_asl * (0.5 * DT),
            velocity: state.velocity + k1.velocity * (0.5 * DT),
        };
        let k2 = derivatives::<K>(dp, &mid, p);
        let next = St {
            altitude_asl: state.altitude_asl + k2.altitude_asl * DT,
            velocity: state.velocity + k2.velocity * DT,
        };
        if !next.altitude_asl.is_finite()
            || !next.velocity.x.is_finite()
            || !next.velocity.y.is_finite()
        {
            return initial.altitude_asl;
        }
        let vy0 = state.velocity.y;
        let vy1 = next.velocity.y;
        if vy1 <= 0.0 {
            let denom = vy1 - vy0;
            if denom.abs() < f32::EPSILON {
                return next.altitude_asl.max(state.altitude_asl);
            }
            let t_zero = DT * (-vy0) / denom;
            let delta_alt = vy0 * t_zero + 0.5 * (denom / DT) * t_zero * t_zero;
            return state.altitude_asl + delta_alt;
        }
        state = next;
    }
    initial.altitude_asl
}

/// The same 6-simulation shape as `AirBrakesMPC::update`.
fn solve<K: Kernel>(alt: f32, vel: Vector2<f32>, p: &RocketParameters, target: f32) -> f32 {
    let initial = St {
        altitude_asl: alt,
        velocity: vel,
    };
    let mut low = -1.0f32;
    let mut high = 1.0f32;
    let mut ap_low = simulate_apogee::<K>(low, &initial, p);
    let mut ap_high = simulate_apogee::<K>(high, &initial, p);
    for _ in 0..3 {
        let mid = 0.5 * (low + high);
        let ap_mid = simulate_apogee::<K>(mid, &initial, p);
        if ap_mid > target {
            low = mid;
            ap_low = ap_mid;
        } else {
            high = mid;
            ap_high = ap_mid;
        }
    }
    let denom = ap_low - ap_high;
    let t = if denom.abs() < 1e-6 {
        0.5
    } else {
        ((target - ap_high) / denom).clamp(0.0, 1.0)
    };
    let dp = high + t * (low - high);
    simulate_apogee::<K>(dp.clamp(0.0, 1.0), &initial, p)
}

// --- harness -----------------------------------------------------------------

fn report(name: &str, iters: u32, us: u64, baseline_us: u64) {
    let net = us.saturating_sub(baseline_us);
    let ns_per = (net * 1000) / iters as u64;
    let cyc_per = (net * SYSCLK_MHZ) / iters as u64;
    info!(
        "BENCH {} iters={} total={}us net={}us => {}ns/call {}cyc/call",
        name, iters, us, net, ns_per, cyc_per
    );
}

fn time_fn(f: fn(f32) -> f32, iters: u32) -> u64 {
    let t0 = Instant::now();
    let mut acc = 0.0f32;
    for i in 0..iters {
        let h = black_box(i as f32 * 0.1);
        acc += f(h);
    }
    black_box(acc);
    Instant::now().saturating_duration_since(t0).as_micros()
}

#[entry]
fn main() -> ! {
    let _p = embassy_stm32::init(clock_config::vlf5_clock_config());

    info!("=== MPC solver bench, sysclk {} MHz, opt-level from profile ===", SYSCLK_MHZ);

    // 1. Isolated scalar functions.
    const N: u32 = 100_000;
    let base = time_fn(density_noop, N);
    info!("BENCH baseline(loop only) iters={} total={}us", N, base);
    report("density-libm-powf", N, time_fn(density_libm, N), base);
    report("density-poly4", N, time_fn(density_poly, N), base);
    report("sqrtf-libm", N, time_fn(sqrtf_probe, N), base);
    report("sqrt-hw-vsqrt", N, time_fn(sqrt_hw, N), base);

    // 2. Accuracy cross-check on the real domain, so a fast wrong answer is
    //    caught here rather than in flight.
    let mut worst_rel = 0.0f32;
    let mut worst_h = 0.0f32;
    let mut h = -1000.0f32;
    while h <= 12000.0 {
        let a = density_libm(h);
        let b = density_poly(h);
        let rel = ((a - b) / a).abs();
        if rel > worst_rel {
            worst_rel = rel;
            worst_h = h;
        }
        h += 10.0;
    }
    info!(
        "ACCURACY poly vs libm: worst rel err {} at h={}m (libm={}, poly={})",
        worst_rel,
        worst_h,
        density_libm(worst_h),
        density_poly(worst_h)
    );

    // 3. Full solves at flight-representative states.
    let params = RocketParameters {
        burnout_mass: 18.696,
        cd: [0.61365, 0.69816, 0.8084, 0.96641, 1.12441],
        reference_area: 0.009854945,
    };
    // Pad -63.5 m ASL, SD-persisted target 9448 m AGL.
    let target_asl = 9448.0 - 63.5;
    let mpc = AirBrakesMPC::new(params.clone(), target_asl);

    let states: [(&str, f32, Vector2<f32>); 4] = [
        ("birth vy=229", 6967.0, Vector2::new(0.0, 229.0)),
        ("gate-max vy=250", 6967.0, Vector2::new(0.0, 250.0)),
        ("mid vy=120", 8500.0, Vector2::new(0.0, 120.0)),
        ("near-apogee vy=30", 9300.0, Vector2::new(0.0, 30.0)),
    ];

    for (name, alt, vel) in states.iter() {
        const M: u32 = 20;

        let t0 = Instant::now();
        for _ in 0..M {
            black_box(mpc.update(black_box(*alt), black_box(*vel)));
        }
        let real = Instant::now().saturating_duration_since(t0).as_micros();

        let t0 = Instant::now();
        for _ in 0..M {
            black_box(solve::<KCurrent>(
                black_box(*alt),
                black_box(*vel),
                &params,
                target_asl,
            ));
        }
        let copy_libm = Instant::now().saturating_duration_since(t0).as_micros();

        let t0 = Instant::now();
        for _ in 0..M {
            black_box(solve::<KPoly>(
                black_box(*alt),
                black_box(*vel),
                &params,
                target_asl,
            ));
        }
        let copy_poly = Instant::now().saturating_duration_since(t0).as_micros();

        let t0 = Instant::now();
        for _ in 0..M {
            black_box(solve::<KFast>(
                black_box(*alt),
                black_box(*vel),
                &params,
                target_asl,
            ));
        }
        let copy_fast = Instant::now().saturating_duration_since(t0).as_micros();

        info!(
            "SOLVE [{}] mpc-real={}us copy-current={}us copy-poly={}us copy-fast={}us",
            name,
            real / M as u64,
            copy_libm / M as u64,
            copy_poly / M as u64,
            copy_fast / M as u64,
        );
        // Agreement matters more than speed: a faster wrong apogee is useless.
        info!(
            "  predicted apogee asl: real={}m current={}m poly={}m fast={}m",
            mpc.update(*alt, *vel).predicted_apogee_asl,
            solve::<KCurrent>(*alt, *vel, &params, target_asl),
            solve::<KPoly>(*alt, *vel, &params, target_asl),
            solve::<KFast>(*alt, *vel, &params, target_asl),
        );
    }

    info!("=== done ===");
    loop {
        cortex_m::asm::wfi();
    }
}
