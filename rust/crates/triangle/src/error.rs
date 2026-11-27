//! The public error taxonomy (ADR 0004, section 6): 'Error' distilled
//! from the remaining triexit sites and the TRI_* status codes. Since
//! idiomatization rung 2 the core raises these variants at the
//! raising sites (ADR 0001); the stage-1 mirrored 'InternalStatus'
//! enum and its single funnel translation are dissolved (D8's
//! rung-2 resolution note, decided in the rung-2 PR; deviation
//! register).
//!
//! C sites: triexit/trimalloc/trifree/internalerror
//! (triangle.c:313-354), the TRI_* codes (triangle_config.h:28-40).
//! Deviation D8: TRI_NULL and the null-context checks dissolve (the
//! owned API carries no null context); D11: no process exits. The
//! pool seam keeps its own primitive: pool::Oom converts into
//! Error::OutOfMemory through the From impl below (D26) wherever pool
//! allocation propagates.

use std::fmt;

/// The mirrored segment-recovery failure codes the segment machinery
/// raised through C '*status' out-params in the stage-1 funnel
/// (TRI_FIND_DIRECTION, TRI_SEG_SPLIT, TRI_SEG_INTERSECT,
/// TRI_SEG_INSERT, TRI_SEG_SCOUT, TRI_CYCLE). The kind is the exact C
/// code; no public information is lost. A PSLG whose segments cross or
/// resist recovery fails here.
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
    /// TRI_FILE_OPEN / TRI_FILE_READ (the read-side failures) and the
    /// writers I/O failures: the readers raise it at the seam carrying
    /// the io::Error (the register writer-seam row D31).
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

/// The OOM bridge from the pool layer (ADR 0004 section 6, item 4):
/// the pools' primitive fallible allocation returns pool::Oom; call
/// sites convert it to Error::OutOfMemory exactly as C's fatal
/// "Error:  Out of memory." site would have ended the process.
impl From<crate::pool::Oom> for Error {
    fn from(_: crate::pool::Oom) -> Error {
        Error::OutOfMemory
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// C's fatal messages render verbatim (ADR 0004 section 6, item
    /// 7). Adapted in rung 2: constructs Error::Internal directly with
    /// the verbatim internalerror payload - the same text the
    /// funnel_maps_every_negative_code arm carried and the
    /// conformingedge site's boundary conversion (construct.rs)
    /// constructs today.
    #[test]
    fn fatal_messages_are_verbatim() {
        assert_eq!(Error::OutOfMemory.to_string(), "Error:  Out of memory.");
        let e = Error::Internal(
            "  Please report this bug to jrs@cs.berkeley.edu\n  Include the message above, your input data set, and the exact\n    command line you used to run Triangle.".to_string(),
        );
        let msg = e.to_string();
        assert!(msg.contains("Please report this bug to jrs@cs.berkeley.edu"));
        assert!(msg.contains("command line you used to run Triangle"));
    }

    #[test]
    fn oom_bridge_from_pool() {
        let e: Error = crate::pool::Oom.into();
        assert!(matches!(e, Error::OutOfMemory));
    }

    /// The taxonomy Display texts stay pinned byte-for-byte (rung 2's
    /// observable-identity rule). Replaces the funnel-arm pins the
    /// funnel test carried for Options/Input/SegmentRecovery/Io; the
    /// raising sites pin the variant construction itself (the
    /// rung-2 outcome tests).
    #[test]
    fn display_texts_are_pinned() {
        assert_eq!(
            Error::Options("msg".to_string()).to_string(),
            "invalid or conflicting switches: msg"
        );
        assert_eq!(
            Error::Input("msg".to_string()).to_string(),
            "invalid input: msg"
        );
        assert_eq!(
            Error::SegmentRecovery(SegmentRecoveryKind::Scout).to_string(),
            "segment recovery failed (Scout, a PSLG whose segments cross or resist recovery)"
        );
        assert_eq!(
            Error::Io(std::io::Error::other("x")).to_string(),
            "io failure: x"
        );
    }
}
