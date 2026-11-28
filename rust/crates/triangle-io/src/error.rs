//! The crate's own error taxonomy (ADR 0006, issue #70): the two
//! failure classes the C readers raise at the read seam
//! (triangle_io.c) - the fgets NULL (TRI_FILE_READ, D31's equivalence
//! class in the port) and the raw -1 TODO failures on malformed input
//! (TRI_FAILURE payloads). Consumers convert at their seam; the
//! triangle crate maps Error::Read onto its Error::Io (the same
//! io::Error payload) and Error::Input onto its Error::Input, so its
//! public surface is unchanged.

use std::fmt;

/// The reader failure taxonomy: TRI_FILE_READ (a fgets NULL at the
/// read seam, the io::Error payload the port's funnel arm carried)
/// and the raw -1 TODO returns of C's readers (malformed input - C's
/// own TODO texts say so; the payload is the funnel arm's exact
/// TRI_FAILURE text).
#[derive(Debug)]
pub enum Error {
    /// TRI_FILE_READ: a fgets NULL at the reader seam.
    Read(std::io::Error),
    /// The raw -1 TODO returns of C's readers: malformed input,
    /// carrying the exact TRI_FAILURE payload text.
    Input(String),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Read(e) => write!(f, "file read failure (TRI_FILE_READ): {}", e),
            Error::Input(msg) => write!(f, "invalid input: {}", msg),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::Read(e) => Some(e),
            Error::Input(_) => None,
        }
    }
}

/// The crate's result alias.
pub type Result<T> = std::result::Result<T, Error>;
