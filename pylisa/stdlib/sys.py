"""Structural replica of the ``sys`` module (Python 3.14)."""

from __future__ import annotations

from typing import Any, Callable, Iterable, TextIO, TypeVar

_T = TypeVar("_T")


# ---------------------------------------------------------------------------
# Struct-sequence-like informational types
# ---------------------------------------------------------------------------

class _flags(object):
    debug: int
    inspect: int
    interactive: int
    optimize: int
    dont_write_bytecode: int
    no_user_site: int
    no_site: int
    ignore_environment: int
    verbose: int
    bytes_warning: int
    quiet: int
    hash_randomization: int
    isolated: int
    dev_mode: bool
    utf8_mode: int
    warn_default_encoding: int
    safe_path: bool
    int_max_str_digits: int


class _float_info(object):
    max: float
    max_exp: int
    max_10_exp: int
    min: float
    min_exp: int
    min_10_exp: int
    dig: int
    mant_dig: int
    epsilon: float
    radix: int
    rounds: int


class _hash_info(object):
    width: int
    modulus: int
    inf: int
    nan: int
    imag: int
    algorithm: str
    hash_bits: int
    seed_bits: int
    cutoff: int


class _int_info(object):
    bits_per_digit: int
    sizeof_digit: int
    default_max_str_digits: int
    str_digits_check_threshold: int


class _thread_info(object):
    name: str | None
    lock: str | None
    version: str | None


class _version_info(object):
    major: int
    minor: int
    micro: int
    releaselevel: str
    serial: int


class _implementation(object):
    name: str
    cache_tag: str | None
    version: _version_info
    hexversion: int
    _multiarch: str


class UnraisableHookArgs(object):
    exc_type: type[BaseException]
    exc_value: BaseException | None
    exc_traceback: Any
    err_msg: str | None
    object: Any


# ---------------------------------------------------------------------------
# Attributes
# ---------------------------------------------------------------------------

abiflags: str
argv: list[str]
orig_argv: list[str]
base_exec_prefix: str
base_prefix: str
byteorder: str
builtin_module_names: tuple[str, ...]
copyright: str
dont_write_bytecode: bool
exec_prefix: str
executable: str
flags: _flags
float_info: _float_info
float_repr_style: str
hash_info: _hash_info
hexversion: int
implementation: _implementation
int_info: _int_info
last_type: type[BaseException] | None
last_value: BaseException | None
last_traceback: Any
maxsize: int
maxunicode: int
meta_path: list[Any]
modules: dict[str, Any]
path: list[str]
path_hooks: list[Any]
path_importer_cache: dict[str, Any]
platform: str
platlibdir: str
prefix: str
ps1: Any
ps2: Any
pycache_prefix: str | None
stdin: TextIO
stdout: TextIO
stderr: TextIO
__stdin__: TextIO
__stdout__: TextIO
__stderr__: TextIO
stdlib_module_names: frozenset[str]
thread_info: _thread_info
tracebacklimit: int | None
version: str
api_version: int
version_info: _version_info
warnoptions: list[str]
winver: str
displayhook: Callable[[Any], None]
__displayhook__: Callable[[Any], None]
excepthook: Callable[[type[BaseException], BaseException, Any], None]
__excepthook__: Callable[[type[BaseException], BaseException, Any], None]
unraisablehook: Callable[[UnraisableHookArgs], None]
__unraisablehook__: Callable[[UnraisableHookArgs], None]
__breakpointhook__: Callable[..., Any]
__interactivehook__: Callable[[], None]


# ---------------------------------------------------------------------------
# Functions
# ---------------------------------------------------------------------------

def addaudithook(hook: Callable[[str, tuple[Any, ...]], None]) -> None:
    # Append a callable to the list of active auditing hooks.
    pass

def audit(event: str, /, *args: Any) -> None:
    # Raise an auditing event, invoking any active auditing hooks.
    pass

def breakpointhook(*args: Any, **kwargs: Any) -> Any:
    # Called by the breakpoint() builtin to enter a debugger.
    pass

def _clear_type_cache() -> None:
    # Clear the internal type attribute lookup cache.
    pass

def _current_frames() -> dict[int, Any]:
    # Return the current stack frame for each running thread, keyed by thread id.
    pass

def _current_exceptions() -> dict[int, BaseException | None]:
    # Return the currently handled exception for each running thread, keyed by thread id.
    pass

def exception() -> BaseException | None:
    # Return the exception currently being handled, or None.
    pass

def exc_info() -> tuple[type[BaseException], BaseException, Any] | tuple[None, None, None]:
    # Return information about the exception currently being handled.
    pass

def exit(code: Any = None) -> None:
    # Raise SystemExit, requesting interpreter termination with the given code.
    pass

def getallocatedblocks() -> int:
    # Return the number of memory blocks currently allocated by the interpreter.
    pass

def getdefaultencoding() -> str:
    # Return the name of the current default string encoding.
    pass

def getdlopenflags() -> int:
    # Return the flags used for dlopen() calls when loading extension modules.
    pass

def getfilesystemencoding() -> str:
    # Return the encoding used to convert between Unicode filenames and bytes.
    pass

def getfilesystemencodeerrors() -> str:
    # Return the error mode used when encoding/decoding filenames.
    pass

def getrefcount(object: Any, /) -> int:
    # Return the reference count of an object, plus one for the argument itself.
    pass

def getrecursionlimit() -> int:
    # Return the current maximum interpreter recursion depth.
    pass

def getsizeof(object: Any, default: int = ...) -> int:
    # Return the size of an object in bytes.
    pass

def get_asyncgen_hooks() -> tuple[Callable[[Any], None] | None, Callable[[Any], None] | None]:
    # Return the current (firstiter, finalizer) async generator hooks.
    pass

def get_coroutine_origin_tracking_depth() -> int:
    # Return the current coroutine origin tracking depth, for debugging.
    pass

def getprofile() -> Callable[..., Any] | None:
    # Return the profiler function set by setprofile(), if any.
    pass

def gettrace() -> Callable[..., Any] | None:
    # Return the trace function set by settrace(), if any.
    pass

def getswitchinterval() -> float:
    # Return the interpreter's thread-switch interval, in seconds.
    pass

def getunicodeinternedsize() -> int:
    # Return the number of unicode strings currently interned.
    pass

def getwindowsversion() -> Any:
    # Return information about the running Windows version.
    pass

def intern(string: str, /) -> str:
    # Enter a string into the interpreter's table of interned strings.
    pass

def is_finalizing() -> bool:
    # Return whether the Python interpreter is shutting down.
    pass

def _is_gil_enabled() -> bool:
    # Return whether the Global Interpreter Lock is currently enabled.
    pass

def setdlopenflags(flags: int, /) -> None:
    # Set the flags used for dlopen() calls when loading extension modules.
    pass

def setprofile(profilefunc: Callable[..., Any] | None) -> None:
    # Set a profiling function invoked on function call/return events.
    pass

def setrecursionlimit(limit: int, /) -> None:
    # Set the maximum interpreter recursion depth.
    pass

def setswitchinterval(interval: float, /) -> None:
    # Set the interpreter's thread-switch interval, in seconds.
    pass

def settrace(tracefunc: Callable[..., Any] | None) -> None:
    # Set a trace function invoked on line/call/return/exception events.
    pass

def set_asyncgen_hooks(firstiter: Callable[[Any], None] | None = None, finalizer: Callable[[Any], None] | None = None) -> None:
    # Set the (firstiter, finalizer) hooks for asynchronous generators.
    pass

def set_coroutine_origin_tracking_depth(depth: int) -> None:
    # Set the coroutine origin tracking depth, for debugging.
    pass

def set_int_max_str_digits(maxdigits: int) -> None:
    # Set the limit on digits allowed when converting between int and str.
    pass

def get_int_max_str_digits() -> int:
    # Return the current limit on digits allowed when converting between int and str.
    pass
