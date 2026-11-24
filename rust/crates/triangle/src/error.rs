//! The public error taxonomy and the error funnel (ADR 0004, section 6):
//! 'Error' distilled from the remaining triexit sites and the TRI_* status
//! codes, plus the mirrored internal 'InternalStatus' enum (the exact C
//! discriminants of triangle_config.h:28-40) and the single funnel
//! translation ('From<InternalStatus> for Error') at the api boundary.
//!
//! Internal codes stay mirrored until idiomatization rung 2 (ADR 0001).
//! C sites: triexit/trimalloc/trifree/internalerror (triangle.c:313-354),
//! the TRI_* codes (triangle_config.h:28-40). Deviation D8: TRI_NULL and
//! the null-context checks dissolve (owned API); D11: no process exits.
//!
//! Stage-1 note: the funnel extensions below (Oom, InternalError,
//! NotImplemented) have no C number because C exits the process on those
//! conditions instead of returning. The blanket silences ahead-of-use
//! dead_code (the I/O and segment-recovery paths that construct several
//! variants are not landed yet) and is removed with them.
#![allow(dead_code)]

use std::fmt;

/// The mirrored segment-recovery failure codes the segment machinery
/// propagates through C '*status' out-params today (TRI_FIND_DIRECTION,
/// TRI_SEG_SPLIT, TRI_SEG_INTERSECT, TRI_SEG_INSERT, TRI_SEG_SCOUT,
/// TRI_CYCLE). The kind is the exact C code; no public information is
/// lost. A PSLG whose segments cross or resist recovery fails here.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentRecoveryKind {
    FindDirection,
    Split,
    Intersect,
    Insert,
    Scout,
    Cycle,
}

/// The public error taxonomy (ADR 0004, section 6). The C fatal- site
/// messages survive verbatim: 'OutOfMemory' renders C's exact
/// "Error:  Out of memory." (triangle.c:318-324) and 'Internal' carries
/// C's internalerror() text payload (triangle.c:349-354, rendered by
/// Display).
#[derive(Debug)]
pub enum Error {
    /// TRI_OPTIONS: invalid or conflicting switches.
    Options(String),
    /// TRI_INPUT and the TRI_FAILUREs of input validation: bad
    /// coordinates, indices, counts.
    Input(String),
    /// The mirrored internal codes of the segment-insertion failure
    /// family (*status propagation in C).
    SegmentRecovery(SegmentRecoveryKind),
    /// C prints "Error:  Out of memory." and exits 1 (trimalloc,
    /// triangle.c:318-324). The pool path returns this instead.
    OutOfMemory,
    /// internalerror() (triangle.c:349-354): a port bug, 5 call sites in
    /// C. The payload is the C stdout text.
    Internal(String),
    /// TRI_FILE_OPEN / TRI_FILE_READ. Dormant until the I/O phase fills
    /// the file functions.
    Io(std::io::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            // C's triexit(1) site at trimalloc prints exactly this.
            Error::OutOfMemory => write!(f, "Error:  Out of memory."),
            Error::Options(msg) => write!(f, "invalid or conflicting switches: {}", msg),
            Error::Input(msg) => write!(f, "invalid input: {}", msg),
            Error::SegmentRecovery(kind) => write!(
                f,
                "segment recovery failed ({:?}, a PSLG whose segments cross or resist recovery)",
                kind
            ),
            Error::Internal(msg) => write!(f, "Triangle internal error: {}", msg),
            Error::Io(e) => write!(f, "io failure: {}", e),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Io(e) => Some(e),
            _ => None,
        }
    }
}

/// The crate's result alias for the public surface.
pub type Result<T> = std::result::Result<T, Error>;

/// The mirrored internal status codes (funnel item 1, ADR 0004 section
/// 6): the exact C discriminants of the TRI_* codes, plus the funnel
/// extensions 'Oom' and 'InternalError' (C exits the process on those
/// two instead of returning) and 'NotImplemented', the stage-1
/// scaffolding marker for entry points whose core modules have not
/// landed yet. Extensions get synthetic negative values that no C code
/// ever produced; the C-valued discriminants are pinned by a test.
///
/// Until rung 2, every transliterated core function that returns a
/// status in C returns this, and every C 'int *status' out-param
/// becomes '&mut InternalStatus' with identical propagation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(i32)]
pub(crate) enum InternalStatus {
    Ok = 0,
    Failure = -1,
    Options = -2,
    Input = -3,
    FindDirection = -4,
    SegSplit = -5,
    SegIntersect = -6,
    SegInsert = -7,
    SegScout = -8,
    FileOpen = -9,
    FileRead = -10,
    Cycle = -11,
    /// Synthetic (no C number): C exits at the trimalloc site instead.
    Oom = -12,
    /// Synthetic (no C number): C exits at the internalerror site.
    InternalError = -13,
    /// Synthetic (no C number): stage-1 shell scaffolding marker for
    /// entry points awaiting their transliterated core.
    NotImplemented = -14,
}

/// C internalerror (triangle.c:349-354). C's noreturn-by-exit becomes
/// noreturn-by-construction: every C call site reads 'return
/// internalerror();'. The C stdout text rides the funnel as the
/// Error::Internal payload.
pub(crate) fn internalerror() -> InternalStatus {
    InternalStatus::InternalError
}

/// The OOM bridge from the pool layer (ADR 0004 section 6, item 4):
/// the pools' primitive fallible allocation returns pool::Oom; at the
/// api boundary it funnels to Error::OutOfMemory exactly as C's fatal
/// "Error:  Out of memory." site would have ended the process.
impl From<crate::pool::Oom> for Error {
    fn from(_: crate::pool::Oom) -> Error {
        Error::OutOfMemory
    }
}

impl From<crate::pool::Oom> for InternalStatus {
    fn from(_: crate::pool::Oom) -> InternalStatus {
        InternalStatus::Oom
    }
}

/// Funnel item 5 (ADR 0004 section 6): the single translation point,
/// used only at the api.rs boundary exactly where triangle_api.c
/// inspects returned statuses. The generic payloads carry the C code
/// names; sites with richer knowledge (input validation, options
/// validation) build their Error directly with the specific message.
/// FileOpen/FileRead are dormant until the I/O phase.
impl From<InternalStatus> for Error {
    fn from(s: InternalStatus) -> Error {
        match s {
            InternalStatus::Ok => Error::Internal(
                "funnel misuse: an Ok status reached the translation point".to_string(),
            ),
            InternalStatus::Failure => {
                Error::Input("invalid input (TRI_FAILURE)".to_string())
            }
            InternalStatus::Options => {
                Error::Options("invalid or conflicting switches (TRI_OPTIONS)".to_string())
            }
            InternalStatus::Input => Error::Input("invalid input (TRI_INPUT)".to_string()),
            InternalStatus::FindDirection => {
                Error::SegmentRecovery(SegmentRecoveryKind::FindDirection)
            }
            InternalStatus::SegSplit => Error::SegmentRecovery(SegmentRecoveryKind::Split),
            InternalStatus::SegIntersect => {
                Error::SegmentRecovery(SegmentRecoveryKind::Intersect)
            }
            InternalStatus::SegInsert => Error::SegmentRecovery(SegmentRecoveryKind::Insert),
            InternalStatus::SegScout => Error::SegmentRecovery(SegmentRecoveryKind::Scout),
            InternalStatus::FileOpen => Error::Io(std::io::Error::other(
                "file open failure (TRI_FILE_OPEN; dormant until the I/O phase)",
            )),
            InternalStatus::FileRead => Error::Io(std::io::Error::other(
                "file read failure (TRI_FILE_READ; dormant until the I/O phase)",
            )),
            InternalStatus::Cycle => Error::SegmentRecovery(SegmentRecoveryKind::Cycle),
            InternalStatus::Oom => Error::OutOfMemory,
            // The verbatim C stdout text of internalerror (triangle.c:349-354).
            InternalStatus::InternalError => Error::Internal(
                "  Please report this bug to jrs@cs.berkeley.edu\n  Include the message above, your input data set, and the exact\n    command line you used to run Triangle.".to_string(),
            ),
            InternalStatus::NotImplemented => Error::Internal(
                "not implemented: this entry point awaits its stage-1 core module (scaffolding)".to_string(),
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Funnel item 1: the C-valued discriminants are pinned exactly
    /// (triangle_config.h:28-40).
    #[test]
    fn internal_status_c_discriminants() {
        assert_eq!(InternalStatus::Ok as i32, 0);
        assert_eq!(InternalStatus::Failure as i32, -1);
        assert_eq!(InternalStatus::Options as i32, -2);
        assert_eq!(InternalStatus::Input as i32, -3);
        assert_eq!(InternalStatus::FindDirection as i32, -4);
        assert_eq!(InternalStatus::SegSplit as i32, -5);
        assert_eq!(InternalStatus::SegIntersect as i32, -6);
        assert_eq!(InternalStatus::SegInsert as i32, -7);
        assert_eq!(InternalStatus::SegScout as i32, -8);
        assert_eq!(InternalStatus::FileOpen as i32, -9);
        assert_eq!(InternalStatus::FileRead as i32, -10);
        assert_eq!(InternalStatus::Cycle as i32, -11);
    }

    /// Funnel item 5: every negative code funnels to its public variant
    /// (the extensions included).
    #[test]
    fn funnel_maps_every_negative_code() {
        let e = Error::from(InternalStatus::Failure);
        assert!(matches!(e, Error::Input(_)));
        let e = Error::from(InternalStatus::Options);
        assert!(matches!(e, Error::Options(_)));
        let e = Error::from(InternalStatus::Input);
        assert!(matches!(e, Error::Input(_)));
        assert!(matches!(
            Error::from(InternalStatus::FindDirection),
            Error::SegmentRecovery(SegmentRecoveryKind::FindDirection)
        ));
        assert!(matches!(
            Error::from(InternalStatus::SegSplit),
            Error::SegmentRecovery(SegmentRecoveryKind::Split)
        ));
        assert!(matches!(
            Error::from(InternalStatus::SegIntersect),
            Error::SegmentRecovery(SegmentRecoveryKind::Intersect)
        ));
        assert!(matches!(
            Error::from(InternalStatus::SegInsert),
            Error::SegmentRecovery(SegmentRecoveryKind::Insert)
        ));
        assert!(matches!(
            Error::from(InternalStatus::SegScout),
            Error::SegmentRecovery(SegmentRecoveryKind::Scout)
        ));
        assert!(matches!(
            Error::from(InternalStatus::Cycle),
            Error::SegmentRecovery(SegmentRecoveryKind::Cycle)
        ));
        assert!(matches!(
            Error::from(InternalStatus::Oom),
            Error::OutOfMemory
        ));
        assert!(matches!(
            Error::from(InternalStatus::InternalError),
            Error::Internal(_)
        ));
        assert!(matches!(
            Error::from(InternalStatus::NotImplemented),
            Error::Internal(_)
        ));
        // Dormant but funnelled IO codes.
        assert!(matches!(
            Error::from(InternalStatus::FileOpen),
            Error::Io(_)
        ));
        assert!(matches!(
            Error::from(InternalStatus::FileRead),
            Error::Io(_)
        ));
    }

    /// C's fatal messages render verbatim (ADR 0004 section 6, item 7).
    #[test]
    fn fatal_messages_are_verbatim() {
        assert_eq!(Error::OutOfMemory.to_string(), "Error:  Out of memory.");
        let e = Error::from(internalerror());
        let msg = e.to_string();
        assert!(msg.contains("Please report this bug to jrs@cs.berkeley.edu"));
        assert!(msg.contains("command line you used to run Triangle"));
    }

    #[test]
    fn oom_bridge_from_pool() {
        let e: Error = crate::pool::Oom.into();
        assert!(matches!(e, Error::OutOfMemory));
    }
}
