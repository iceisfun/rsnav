//! Minimal scoped-thread parallel primitives, dependency-free.
//!
//! [`par_map_indexed`]: apply a function to every element of a slice
//! across `std::thread::scope` workers and return the results in input
//! order. Workers pull indices from a shared atomic counter, so wildly
//! uneven per-item cost (one huge region among a hundred small ones)
//! still balances. Results are scattered back by index after the join —
//! output order is deterministic by construction, regardless of which
//! worker finished what.
//!
//! [`par_bands_mut`]: the scatter-in-place counterpart, for phases shaped
//! "read the whole input, write disjoint bands of an output buffer".
//! `par_map_indexed` can only express those by allocating a per-band
//! result and concatenating — an extra full-buffer copy per phase, which
//! for a multi-pass grid transform is the dominant cost. Both helpers
//! tolerate spawn failure and both are thread-count invariant.

use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

/// Resolve a thread-count request: `0` means "use available parallelism",
/// anything else is returned literally and unclamped — callers cap the
/// result themselves (e.g. `.min(items.len())`; [`par_map_indexed`]
/// already clamps to its item count internally).
pub fn resolve_threads(requested: usize) -> usize {
    if requested == 0 {
        std::thread::available_parallelism().map_or(1, |n| n.get())
    } else {
        requested
    }
}

/// Map `f` over `items` on up to `threads` scoped workers, returning
/// results in input order.
///
/// `f` receives `(index, &item)`. With `threads <= 1` or fewer than two
/// items this runs serially on the caller's thread with no spawn at all.
/// A panic in `f` propagates to the caller once the scope unwinds.
pub fn par_map_indexed<I, O, F>(items: &[I], threads: usize, f: F) -> Vec<O>
where
    I: Sync,
    O: Send,
    F: Fn(usize, &I) -> O + Sync,
{
    let threads = threads.min(items.len());
    if threads <= 1 || items.len() < 2 {
        return items.iter().enumerate().map(|(i, it)| f(i, it)).collect();
    }

    let next = AtomicUsize::new(0);
    let mut buckets: Vec<Vec<(usize, O)>> = Vec::new();
    let work = || {
        let mut out: Vec<(usize, O)> = Vec::new();
        loop {
            let i = next.fetch_add(1, Ordering::Relaxed);
            if i >= items.len() {
                break;
            }
            out.push((i, f(i, &items[i])));
        }
        out
    };
    std::thread::scope(|s| {
        // Spawn failure (EAGAIN, pid limits) is tolerated, not fatal: the
        // shared counter makes worker count irrelevant to correctness, so
        // a failed spawn just degrades parallelism — worst case the
        // caller, which always works the queue too, runs everything.
        let handles: Vec<_> = (1..threads)
            .filter_map(|_| std::thread::Builder::new().spawn_scoped(s, work).ok())
            .collect();
        buckets.push(work());
        for h in handles {
            // join() only errs if the worker panicked; resume the panic
            // on the caller so behavior matches the serial path.
            match h.join() {
                Ok(v) => buckets.push(v),
                Err(p) => std::panic::resume_unwind(p),
            }
        }
    });

    let mut slots: Vec<Option<O>> = Vec::with_capacity(items.len());
    slots.resize_with(items.len(), || None);
    for (i, v) in buckets.into_iter().flatten() {
        slots[i] = Some(v);
    }
    slots
        .into_iter()
        .map(|s| s.expect("every index visited exactly once"))
        .collect()
}

/// Apply `f(band_index, band)` to the disjoint `band_len`-sized chunks of
/// `out` across scoped workers, writing in place.
///
/// `out.chunks_mut(band_len)` defines the bands; the final band is short
/// when `band_len` does not divide `out.len()`, exactly as `chunks_mut`
/// yields it, and `f` must handle that. `band_index` is the chunk's
/// ordinal, so `f` can recover the band's absolute offset as
/// `band_index * band_len`.
///
/// **Determinism.** Every band is written by exactly one worker and `f`
/// sees only that band plus shared immutable state, so the bytes written
/// to `out` do not depend on `threads` — there is no reduction, no
/// accumulation across bands, and no reassociation. Which *worker* runs a
/// band varies (bands are pulled from a shared counter, so uneven bands
/// still balance); which *bytes* land where does not.
///
/// With `threads <= 1`, fewer than two bands, or `band_len == 0` this runs
/// serially on the caller's thread with no spawn. A panic in `f`
/// propagates once the scope unwinds.
pub fn par_bands_mut<T: Send, F>(out: &mut [T], band_len: usize, threads: usize, f: F)
where
    F: Fn(usize, &mut [T]) + Sync,
{
    if band_len == 0 {
        // Degenerate request: treat the whole buffer as one band rather
        // than looping forever on zero-length chunks.
        if !out.is_empty() {
            f(0, out);
        }
        return;
    }
    let bands: Vec<&mut [T]> = out.chunks_mut(band_len).collect();
    let threads = threads.min(bands.len());
    if threads <= 1 || bands.len() < 2 {
        for (i, b) in bands.into_iter().enumerate() {
            f(i, b);
        }
        return;
    }

    // Each band lives in its own slot; a worker `take()`s the band it
    // claimed from the counter, so a band is handed out exactly once and
    // the `&mut` never aliases. The Mutex is uncontended by construction
    // (the counter already serialized the claim) and is only here to give
    // shared access to a `&mut` without unsafe.
    let slots: Vec<Mutex<Option<&mut [T]>>> =
        bands.into_iter().map(|b| Mutex::new(Some(b))).collect();
    let next = AtomicUsize::new(0);
    let work = || loop {
        let i = next.fetch_add(1, Ordering::Relaxed);
        if i >= slots.len() {
            break;
        }
        let band = slots[i]
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take()
            .expect("each band index is claimed exactly once");
        f(i, band);
    };
    std::thread::scope(|s| {
        // Spawn failure (EAGAIN, pid limits) degrades parallelism, not
        // correctness: the caller works the same queue, so any band a
        // missing worker would have taken is picked up here.
        let handles: Vec<_> = (1..threads)
            .filter_map(|_| std::thread::Builder::new().spawn_scoped(s, work).ok())
            .collect();
        work();
        for h in handles {
            if let Err(p) = h.join() {
                std::panic::resume_unwind(p);
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn preserves_input_order() {
        let items: Vec<usize> = (0..1000).collect();
        let out = par_map_indexed(&items, 8, |i, &x| {
            assert_eq!(i, x);
            x * 2
        });
        assert_eq!(out, (0..1000).map(|x| x * 2).collect::<Vec<_>>());
    }

    #[test]
    fn serial_and_parallel_agree() {
        let items: Vec<u64> = (0..137).map(|x| x * 31 % 97).collect();
        let serial = par_map_indexed(&items, 1, |_, &x| x * x);
        let par = par_map_indexed(&items, 16, |_, &x| x * x);
        assert_eq!(serial, par);
    }

    #[test]
    fn empty_and_single() {
        let none: Vec<u32> = vec![];
        assert!(par_map_indexed(&none, 8, |_, &x| x).is_empty());
        assert_eq!(par_map_indexed(&[41u32], 8, |_, &x| x + 1), vec![42]);
    }

    #[test]
    fn resolve_zero_is_available_parallelism() {
        assert!(resolve_threads(0) >= 1);
        assert_eq!(resolve_threads(3), 3);
    }

    #[test]
    fn bands_cover_every_element_exactly_once() {
        for &threads in &[1usize, 2, 4, 8, 16] {
            // 1000 is deliberately not a multiple of 64: the last band is short.
            let mut out = vec![0u64; 1000];
            par_bands_mut(&mut out, 64, threads, |bi, band| {
                for (j, slot) in band.iter_mut().enumerate() {
                    *slot = (bi * 64 + j) as u64;
                }
            });
            assert_eq!(out, (0..1000u64).collect::<Vec<_>>(), "threads={threads}");
        }
    }

    #[test]
    fn bands_are_thread_count_invariant() {
        let src: Vec<u64> = (0..4096).map(|x: u64| x.wrapping_mul(2654435761)).collect();
        let run = |threads: usize| {
            let mut out = vec![0u64; src.len()];
            par_bands_mut(&mut out, 37, threads, |bi, band| {
                for (j, slot) in band.iter_mut().enumerate() {
                    *slot = src[bi * 37 + j] ^ (bi as u64);
                }
            });
            out
        };
        let base = run(1);
        for &t in &[2usize, 3, 8, 16, 64] {
            assert_eq!(run(t), base, "threads={t} changed the output");
        }
    }

    #[test]
    fn bands_degenerate_inputs() {
        let mut empty: Vec<u8> = vec![];
        par_bands_mut(&mut empty, 8, 4, |_, _| panic!("no bands to visit"));
        // band_len == 0 falls back to a single whole-buffer band.
        let mut one = vec![0u8; 3];
        par_bands_mut(&mut one, 0, 4, |bi, b| {
            assert_eq!(bi, 0);
            b.fill(7);
        });
        assert_eq!(one, vec![7, 7, 7]);
    }

    #[test]
    fn bands_worker_panic_propagates() {
        let r = std::panic::catch_unwind(|| {
            let mut out = vec![0u32; 512];
            par_bands_mut(&mut out, 16, 4, |bi, _| {
                if bi == 9 {
                    panic!("boom");
                }
            });
        });
        assert!(r.is_err());
    }

    #[test]
    fn worker_panic_propagates() {
        let items: Vec<u32> = (0..64).collect();
        let r = std::panic::catch_unwind(|| {
            par_map_indexed(&items, 4, |_, &x| {
                if x == 33 {
                    panic!("boom");
                }
                x
            })
        });
        assert!(r.is_err());
    }
}
