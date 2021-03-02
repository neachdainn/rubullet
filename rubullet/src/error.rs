//! Error types.
//!
//! Unfortunately, PyBullet doesn't really offer much in the way of error information (most errors
//! are C-style return codes) and the underlying C API is a mess. I can't find any documentation,
//! only endless series of header files and inheritance.
//!
//! As such, this library is going to use a single opaque error type that attempts to provide as
//! much information in the display as possible.
use std::{borrow::Cow, error, fmt};

#[derive(Debug, Clone)]
pub struct Error {
    ctx: Cow<'static, str>,
}

impl Error {
    /// Creates a new error from the provided static string.
    ///
    /// This is not implemented as `From<_>` in order to keep the functionality from being exposed
    /// to users of the crate.
    pub(crate) fn new(ctx: &'static str) -> Error {
        Error {
            ctx: Cow::Borrowed(ctx),
        }
    }

    /// Creates a new error from the provided `String`.
    pub(crate) fn with(ctx: String) -> Error {
        Error {
            ctx: Cow::Owned(ctx),
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.ctx)
    }
}

impl error::Error for Error {
    fn description(&self) -> &str {
        &self.ctx
    }
}
