//! Minimal scoped-thread parallel map, dependency-free.
//!
//! One helper, [`par_map_indexed`]: apply a function to every element of a
//! slice across `std::thread::scope` workers and return the results in
//! input order. Workers pull indices from a shared atomic counter, so
//! wildly uneven per-item cost (one huge region among a hundred small
//! ones) still balances. Results are scattered back by index after the
//! join — output order is deterministic by construction, regardless of
//! which worker finished what.

use std::sync::atomic::{AtomicUsize, Ordering};

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
