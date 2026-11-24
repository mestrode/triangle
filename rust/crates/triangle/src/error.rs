//! The public error taxonomy and the error funnel: Error/Result distilled
//! from the triexit sites and TRI_* status codes, plus the mirrored
//! InternalStatus enum (exact C discriminants) and the single funnel
//! translation (From<InternalStatus> for Error) at the api boundary.
//!
//! Internal codes stay mirrored until idiomatization rung 2 (ADR 0001).
//! ADR 0004 section 6; deviation D8 in docs/deviation-register.md.
