"""Structural replica of the ``io`` module (Python 3.14).

Class hierarchy:

    IOBase
        RawIOBase
            FileIO
        BufferedIOBase
            BytesIO
            BufferedReader
            BufferedWriter
            BufferedRandom
            BufferedRWPair
        TextIOBase
            TextIOWrapper
            StringIO
"""

from __future__ import annotations

from typing import Any, Callable, Iterable, Iterator

DEFAULT_BUFFER_SIZE: int


class UnsupportedOperation(OSError, ValueError):
    pass


class IOBase(object):
    closed: bool

    def close(self) -> None:
        # Flush and close the stream; further operations raise ValueError.
        pass

    def fileno(self) -> int:
        # Return the underlying file descriptor, if there is one.
        pass

    def flush(self) -> None:
        # Flush the write buffers of the stream, if applicable.
        pass

    def isatty(self) -> bool:
        # Return whether the stream is interactive (connected to a terminal).
        pass

    def readable(self) -> bool:
        # Return whether the stream supports read().
        pass

    def readline(self, size: int = -1) -> Any:
        # Read and return one line from the stream.
        pass

    def readlines(self, hint: int = -1) -> list[Any]:
        # Read and return a list of lines from the stream.
        pass

    def seek(self, offset: int, whence: int = 0) -> int:
        # Change the stream position and return the new absolute position.
        pass

    def seekable(self) -> bool:
        # Return whether the stream supports random access.
        pass

    def tell(self) -> int:
        # Return the current stream position.
        pass

    def truncate(self, size: int | None = None) -> int:
        # Resize the stream to the given size, or the current position if omitted.
        pass

    def writable(self) -> bool:
        # Return whether the stream supports write().
        pass

    def writelines(self, lines: Iterable[Any]) -> None:
        # Write a list of lines to the stream.
        pass

    def __del__(self) -> None:
        # Finalize the object, closing the stream if still open.
        pass

    def __enter__(self) -> "IOBase":
        # Enter the runtime context, returning self.
        pass

    def __exit__(self, *exc: Any) -> None:
        # Close the stream on context manager exit.
        pass

    def __iter__(self) -> Iterator[Any]:
        # Return self, since streams are their own line iterator.
        pass

    def __next__(self) -> Any:
        # Return the next line from the stream.
        pass


class RawIOBase(IOBase):
    def read(self, size: int = -1) -> bytes | None:
        # Read up to size bytes from the stream in a single system call.
        pass

    def readall(self) -> bytes:
        # Read and return all bytes from the stream until EOF.
        pass

    def readinto(self, b: Any) -> int | None:
        # Read bytes into a pre-allocated, writable buffer.
        pass

    def write(self, b: Any) -> int | None:
        # Write a buffer of bytes to the stream in a single system call.
        pass


class BufferedIOBase(IOBase):
    raw: RawIOBase

    def detach(self) -> RawIOBase:
        # Separate the underlying raw stream and return it.
        pass

    def read(self, size: int | None = -1) -> bytes:
        # Read and return up to size bytes.
        pass

    def read1(self, size: int = -1) -> bytes:
        # Read and return up to size bytes with at most one call to the raw stream.
        pass

    def readinto(self, b: Any) -> int:
        # Read bytes into a pre-allocated, writable buffer.
        pass

    def readinto1(self, b: Any) -> int:
        # Read bytes into a buffer with at most one call to the raw stream.
        pass

    def write(self, b: Any) -> int:
        # Write the given buffer of bytes to the stream.
        pass


class TextIOBase(IOBase):
    encoding: str
    errors: str | None
    newlines: str | tuple[str, ...] | None

    def detach(self) -> "BufferedIOBase":
        # Separate the underlying binary buffer and return it.
        pass

    def read(self, size: int | None = -1) -> str:
        # Read and return up to size characters as a string.
        pass

    def readline(self, size: int = -1) -> str:
        # Read and return one line from the stream.
        pass

    def write(self, s: str) -> int:
        # Write the given string to the stream.
        pass


class FileIO(RawIOBase):
    name: Any
    mode: str
    closefd: bool

    def __init__(self, file: Any, mode: str = "r", closefd: bool = True, opener: Callable[[Any, int], int] | None = None) -> None:
        # Open a file and wrap it as a raw byte stream.
        pass


class BytesIO(BufferedIOBase):
    def __init__(self, initial_bytes: bytes = b"") -> None:
        # Create an in-memory binary stream, optionally pre-filled with bytes.
        pass

    def getvalue(self) -> bytes:
        # Return the entire contents of the buffer as bytes.
        pass

    def getbuffer(self) -> memoryview:
        # Return a writable memoryview over the buffer's contents.
        pass


class BufferedReader(BufferedIOBase):
    def __init__(self, raw: RawIOBase, buffer_size: int = ...) -> None:
        # Wrap a readable raw stream with a read buffer.
        pass

    def peek(self, size: int = 0) -> bytes:
        # Return bytes from the buffer without advancing the position.
        pass


class BufferedWriter(BufferedIOBase):
    def __init__(self, raw: RawIOBase, buffer_size: int = ...) -> None:
        # Wrap a writable raw stream with a write buffer.
        pass


class BufferedRandom(BufferedIOBase):
    def __init__(self, raw: RawIOBase, buffer_size: int = ...) -> None:
        # Wrap a seekable raw stream with a combined read/write buffer.
        pass

    def peek(self, size: int = 0) -> bytes:
        # Return bytes from the buffer without advancing the position.
        pass


class BufferedRWPair(BufferedIOBase):
    def __init__(self, reader: RawIOBase, writer: RawIOBase, buffer_size: int = ...) -> None:
        # Combine a buffered reader and a buffered writer into one stream.
        pass


class IncrementalNewlineDecoder(object):
    def __init__(self, decoder: Any, translate: bool, errors: str = "strict") -> None:
        # Wrap a decoder to translate universal newlines incrementally.
        pass

    def decode(self, input: Any, final: bool = False) -> str:
        # Decode a chunk of input, translating newlines as configured.
        pass

    def reset(self) -> None:
        # Reset the decoder to its initial state.
        pass

    def getstate(self) -> tuple[Any, int]:
        # Return the decoder's current state, for later restoration.
        pass

    def setstate(self, state: tuple[Any, int]) -> None:
        # Restore the decoder's state from a value returned by getstate().
        pass

    @property
    def newlines(self) -> str | tuple[str, ...] | None:
        # Return the newline character(s) encountered so far.
        pass


class TextIOWrapper(TextIOBase):
    line_buffering: bool
    write_through: bool
    name: Any
    buffer: BufferedIOBase

    def __init__(
        self,
        buffer: BufferedIOBase,
        encoding: str | None = None,
        errors: str | None = None,
        newline: str | None = None,
        line_buffering: bool = False,
        write_through: bool = False,
    ) -> None:
        # Wrap a buffered binary stream to provide text I/O with encoding/decoding.
        pass

    def reconfigure(
        self,
        *,
        encoding: str | None = None,
        errors: str | None = None,
        newline: str | None = None,
        line_buffering: bool | None = None,
        write_through: bool | None = None,
    ) -> None:
        # Reconfigure the text stream's encoding, error handling or buffering.
        pass


class StringIO(TextIOBase):
    def __init__(self, initial_value: str | None = "", newline: str | None = "\n") -> None:
        # Create an in-memory text stream, optionally pre-filled with a string.
        pass

    def getvalue(self) -> str:
        # Return the entire contents of the buffer as a string.
        pass


def open(
    file: Any,
    mode: str = "r",
    buffering: int = -1,
    encoding: str | None = None,
    errors: str | None = None,
    newline: str | None = None,
    closefd: bool = True,
    opener: Callable[[Any, int], int] | None = None,
) -> Any:
    # Open a file and return a corresponding stream object.
    pass


def open_code(path: str) -> Any:
    # Open a file for reading as executable code, honoring import hooks.
    pass


def text_encoding(encoding: str | None, stacklevel: int = 2) -> str:
    # Return encoding if given, otherwise the locale's default text encoding.
    pass
