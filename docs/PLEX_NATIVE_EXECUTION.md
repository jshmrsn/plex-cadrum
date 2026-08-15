# Plex native execution policy

`plex-cadrum` is Plex's thread-confined OCCT execution layer. Raw OCCT handles stay on the
worker that created them; Rust-owned archives, topology facts, diagnostics, and presentation
buffers are the only data that may cross worker boundaries.

## Operation safety matrix

| Operation family | Independent worker lanes | OCCT inner parallelism | Cancellation boundary |
| --- | --- | --- | --- |
| primitives, transforms, exact queries | Allowed | Disabled | before/after operation |
| Boolean cells builder | Allowed for independent shapes | Disabled | OCCT progress plus stage checks |
| extrusion, sweep, fillet, chamfer | Allowed for independent shapes | Disabled | OCCT progress where exposed plus stage checks |
| shell and offset | Allowed for independent shapes | Disabled | safe stage boundaries |
| surface meshing | Allowed for detached presentation shapes | Disabled in Plex after native benchmarks showed regressions at 6 and 150 faces | OCCT progress plus per-face checks |
| B-rep archive/read | Allowed for independent shapes | Not applicable | between archive stages |
| loft | Serialized by `LOFT_LOCK` | Disabled | before/after operation |

The loft lock remains deliberately narrow. Independent Boolean, meshing, archive, query, and
builder pipelines are covered by the concurrency stress suite. Removing the loft lock requires a
repeatable sanitizer-clean stress result against the pinned OCCT build; type-level appearances are
not sufficient evidence.

## Native sanitizer gate

On the macOS development host, run:

```sh
scripts/run-native-sanitizers.sh
```

This rebuilds the C++ bridge with AddressSanitizer and UndefinedBehaviorSanitizer and runs the
malformed-input, cancellation, concurrency, and identity sweep corpus. The prebuilt OCCT archive
is not itself instrumented, so this gate primarily covers bridge ownership, buffers, exception
barriers, and calls into OCCT. A source-built fully instrumented OCCT remains an optional deeper
release qualification step.

## Crash isolation decision

A helper process is not used on the normal interactive path: it would force B-rep serialization
and rehydration into every request and defeat live prepared sessions. The API boundary remains
compatible with adding an opt-in helper for untrusted imports or an operation proven to crash
inside OCCT despite valid bridge usage. Such a lane must exchange only B-rep plus Rust-owned
request/result data and must never become the default direct-manipulation lane.

## Release optimization decision

The measured costs are dominated by OCCT algorithms and avoidable integration work rather than
Rust dispatch. Link-time optimization or PGO is therefore not enabled as a substitute for the
algorithmic gates. Either may be adopted only with repeatable end-to-end benchmark evidence and
the identity/robustness corpus still passing.
