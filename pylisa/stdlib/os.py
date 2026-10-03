"""Structural replica of the ``os`` module (Python 3.14).

``os.path`` is intentionally not modelled here: in real CPython it is not
part of the ``os`` package (``os`` is a plain module, not a package) but a
separately-implemented module (``posixpath``/``ntpath``) merely exposed as
the ``os.path`` attribute, and it was out of the requested scope.
"""

from __future__ import annotations

from typing import Any, Callable, Generic, Iterable, Iterator, Mapping, MutableMapping, Sequence, TypeVar

_T = TypeVar("_T")
_Path = Any  # str | bytes | PathLike[Any]


# ---------------------------------------------------------------------------
# Protocols / small value types
# ---------------------------------------------------------------------------

class PathLike(Generic[_T], object):
    def __fspath__(self) -> _T:
        # Return the file system path representation of the object.
        pass


class _Environ(MutableMapping[str, str]):
    def __init__(self, data: dict[str, str], encodekey: Callable[[str], str], decodekey: Callable[[str], str], encodevalue: Callable[[str], str], decodevalue: Callable[[str], str]) -> None:
        # Initialize the mapping wrapping the process environment.
        pass

    def copy(self) -> dict[str, str]:
        # Return a plain dict copy of the environment.
        pass

    def setdefault(self, key: str, default: str = "") -> str:
        # Return the value for key, setting it to default in the environment if absent.
        pass

    def __getitem__(self, key: str) -> str:
        # Return the value of an environment variable.
        pass

    def __setitem__(self, key: str, value: str) -> None:
        # Set an environment variable, also updating the process environment.
        pass

    def __delitem__(self, key: str) -> None:
        # Remove an environment variable, also updating the process environment.
        pass

    def __iter__(self) -> Iterator[str]:
        # Return an iterator over the environment variable names.
        pass

    def __len__(self) -> int:
        # Return the number of environment variables.
        pass


class stat_result(object):
    st_mode: int
    st_ino: int
    st_dev: int
    st_nlink: int
    st_uid: int
    st_gid: int
    st_size: int
    st_atime: float
    st_mtime: float
    st_ctime: float
    st_atime_ns: int
    st_mtime_ns: int
    st_ctime_ns: int
    st_blocks: int
    st_blksize: int
    st_rdev: int
    st_flags: int
    st_gen: int
    st_birthtime: float
    st_file_attributes: int
    st_reparse_tag: int


class statvfs_result(object):
    f_bsize: int
    f_frsize: int
    f_blocks: int
    f_bfree: int
    f_bavail: int
    f_files: int
    f_ffree: int
    f_favail: int
    f_flag: int
    f_namemax: int
    f_fsid: int


class terminal_size(object):
    columns: int
    lines: int

    def __init__(self, iterable: Iterable[int]) -> None:
        # Initialize with (columns, lines) values.
        pass


class uname_result(object):
    sysname: str
    nodename: str
    release: str
    version: str
    machine: str


class times_result(object):
    user: float
    system: float
    children_user: float
    children_system: float
    elapsed: float


class sched_param(object):
    sched_priority: int

    def __init__(self, sched_priority: int) -> None:
        # Initialize a scheduling parameter with the given priority.
        pass


class waitid_result(object):
    si_pid: int
    si_uid: int
    si_signo: int
    si_status: int
    si_code: int


class DirEntry(Generic[_T], object):
    name: _T
    path: _T

    def inode(self) -> int:
        # Return the inode number of the entry.
        pass

    def is_dir(self, *, follow_symlinks: bool = True) -> bool:
        # Return whether the entry is a directory.
        pass

    def is_file(self, *, follow_symlinks: bool = True) -> bool:
        # Return whether the entry is a regular file.
        pass

    def is_symlink(self) -> bool:
        # Return whether the entry is a symbolic link.
        pass

    def is_junction(self) -> bool:
        # Return whether the entry is a junction point (Windows).
        pass

    def stat(self, *, follow_symlinks: bool = True) -> stat_result:
        # Return a stat_result for the entry.
        pass

    def __fspath__(self) -> _T:
        # Return the file system path of the entry.
        pass


class _wrap_close(str):
    def __init__(self, stream: Any, proc: Any) -> None:
        # Wrap a popen() stream with the subprocess used to close it.
        pass

    def close(self) -> int | None:
        # Close the stream and wait for the subprocess to terminate.
        pass


# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------

name: str
environ: _Environ
environb: _Environ
supports_bytes_environ: bool
sep: str
altsep: str | None
extsep: str
pathsep: str
defpath: str
linesep: str
devnull: str
curdir: str
pardir: str
error = OSError  # noqa: this is a real alias in the actual os module


# ---------------------------------------------------------------------------
# Process parameters
# ---------------------------------------------------------------------------

def ctermid() -> str:
    # Return the filename corresponding to the controlling terminal.
    pass

def getenv(key: str, default: Any = None) -> Any:
    # Return the value of an environment variable, or default if unset.
    pass

def getenvb(key: bytes, default: Any = None) -> Any:
    # Return the value of an environment variable as bytes, or default if unset.
    pass

def putenv(key: Any, value: Any) -> None:
    # Set an environment variable directly in the process environment.
    pass

def unsetenv(key: Any) -> None:
    # Unset an environment variable directly in the process environment.
    pass

def fsencode(filename: _Path) -> bytes:
    # Encode a path-like object to bytes using the filesystem encoding.
    pass

def fsdecode(filename: _Path) -> str:
    # Decode a path-like object to str using the filesystem encoding.
    pass

def fspath(path: _Path) -> Any:
    # Return the file system path representation of a path-like object.
    pass

def getcwd() -> str:
    # Return the current working directory as a string.
    pass

def getcwdb() -> bytes:
    # Return the current working directory as bytes.
    pass

def chdir(path: _Path) -> None:
    # Change the current working directory.
    pass

def fchdir(fd: int) -> None:
    # Change the current working directory to the directory represented by fd.
    pass

def getegid() -> int:
    # Return the effective group id of the current process.
    pass

def geteuid() -> int:
    # Return the effective user id of the current process.
    pass

def getgid() -> int:
    # Return the real group id of the current process.
    pass

def getgrouplist(user: str, group: int) -> list[int]:
    # Return the list of group ids that user belongs to.
    pass

def getgroups() -> list[int]:
    # Return the list of supplemental group ids of the current process.
    pass

def getlogin() -> str:
    # Return the name of the user logged in on the controlling terminal.
    pass

def getpgid(pid: int) -> int:
    # Return the process group id of the process with the given pid.
    pass

def getpgrp() -> int:
    # Return the process group id of the current process.
    pass

def getpid() -> int:
    # Return the current process id.
    pass

def getppid() -> int:
    # Return the parent process id.
    pass

def getpriority(which: int, who: int) -> int:
    # Return the scheduling priority for a process, process group, or user.
    pass

def getresuid() -> tuple[int, int, int]:
    # Return the (real, effective, saved) user ids of the current process.
    pass

def getresgid() -> tuple[int, int, int]:
    # Return the (real, effective, saved) group ids of the current process.
    pass

def getuid() -> int:
    # Return the real user id of the current process.
    pass

def initgroups(username: str, gid: int) -> None:
    # Initialize the group access list using the system group database.
    pass

def setegid(egid: int) -> None:
    # Set the effective group id of the current process.
    pass

def seteuid(euid: int) -> None:
    # Set the effective user id of the current process.
    pass

def setgid(gid: int) -> None:
    # Set the group id of the current process.
    pass

def setgroups(groups: Sequence[int]) -> None:
    # Set the list of supplemental group ids for the current process.
    pass

def setpgrp() -> None:
    # Call the system call setpgrp() or setpgrp(0, 0), platform dependent.
    pass

def setpgid(pid: int, pgrp: int) -> None:
    # Set the process group id of the process with the given pid.
    pass

def setpriority(which: int, who: int, priority: int) -> None:
    # Set the scheduling priority for a process, process group, or user.
    pass

def setregid(rgid: int, egid: int) -> None:
    # Set the real and effective group ids of the current process.
    pass

def setresgid(rgid: int, egid: int, sgid: int) -> None:
    # Set the real, effective and saved group ids of the current process.
    pass

def setresuid(ruid: int, euid: int, suid: int) -> None:
    # Set the real, effective and saved user ids of the current process.
    pass

def setreuid(ruid: int, euid: int) -> None:
    # Set the real and effective user ids of the current process.
    pass

def setuid(uid: int) -> None:
    # Set the user id of the current process.
    pass

def strerror(code: int) -> str:
    # Return the error message corresponding to an errno value.
    pass

def umask(mask: int) -> int:
    # Set the current numeric umask and return the previous mask.
    pass

def uname() -> uname_result:
    # Return information identifying the current operating system.
    pass

def cpu_count() -> int | None:
    # Return the number of CPUs available to the process, if determinable.
    pass

def process_cpu_count() -> int | None:
    # Return the number of CPUs the current process could theoretically use.
    pass


# ---------------------------------------------------------------------------
# File Object Creation / File Descriptor Operations
# ---------------------------------------------------------------------------

def popen(cmd: str, mode: str = "r", buffering: int = -1) -> Any:
    # Open a pipe to or from a shell command, returning a file-like object.
    pass

def fdopen(fd: int, *args: Any, **kwargs: Any) -> Any:
    # Return an open file object connected to an existing file descriptor.
    pass

def close(fd: int) -> None:
    # Close a file descriptor.
    pass

def closerange(fd_low: int, fd_high: int) -> None:
    # Close all file descriptors in the given half-open range.
    pass

def device_encoding(fd: int) -> str | None:
    # Return the encoding used for the device associated with fd, if a terminal.
    pass

def dup(fd: int) -> int:
    # Return a duplicate of file descriptor fd.
    pass

def dup2(fd: int, fd2: int, inheritable: bool = True) -> int:
    # Duplicate file descriptor fd to fd2, closing fd2 first if needed.
    pass

def fchmod(fd: int, mode: int) -> None:
    # Change the mode of the file given by file descriptor fd.
    pass

def fchown(fd: int, uid: int, gid: int) -> None:
    # Change the owner and group of the file given by file descriptor fd.
    pass

def fdatasync(fd: int) -> None:
    # Force write of a file to disk, without forcing metadata updates.
    pass

def fpathconf(fd: int, name: str | int) -> int:
    # Return system configuration information for an open file.
    pass

def fstat(fd: int) -> stat_result:
    # Return status information about the file given by file descriptor fd.
    pass

def fstatvfs(fd: int) -> statvfs_result:
    # Return file system status information for an open file.
    pass

def fsync(fd: int) -> None:
    # Force write of a file to disk, including metadata.
    pass

def ftruncate(fd: int, length: int) -> None:
    # Truncate the file given by file descriptor fd to length bytes.
    pass

def get_blocking(fd: int) -> bool:
    # Return whether a file descriptor is in blocking mode.
    pass

def set_blocking(fd: int, blocking: bool) -> None:
    # Set a file descriptor's blocking mode.
    pass

def get_inheritable(fd: int) -> bool:
    # Return a file descriptor's inheritable flag.
    pass

def set_inheritable(fd: int, inheritable: bool) -> None:
    # Set a file descriptor's inheritable flag.
    pass

def get_terminal_size(fd: int = ...) -> terminal_size:
    # Return the size of the terminal associated with fd.
    pass

def isatty(fd: int) -> bool:
    # Return whether the file descriptor is open and connected to a tty device.
    pass

def lockf(fd: int, cmd: int, len: int) -> None:
    # Apply, test or remove a POSIX lock on an open file descriptor.
    pass

def lseek(fd: int, pos: int, whence: int) -> int:
    # Set the current position of file descriptor fd and return the new position.
    pass

def open(path: _Path, flags: int, mode: int = 0o777, *, dir_fd: int | None = None) -> int:
    # Open a file and return its low-level file descriptor.
    pass

def openpty() -> tuple[int, int]:
    # Open a new pseudo-terminal pair, returning (master_fd, slave_fd).
    pass

def pipe() -> tuple[int, int]:
    # Create a pipe, returning (read_fd, write_fd).
    pass

def pipe2(flags: int) -> tuple[int, int]:
    # Create a pipe with the given flags, returning (read_fd, write_fd).
    pass

def posix_fadvise(fd: int, offset: int, len: int, advice: int) -> None:
    # Announce an intention to access file data in a particular pattern.
    pass

def posix_fallocate(fd: int, offset: int, len: int) -> None:
    # Ensure that disk space is allocated for a file region.
    pass

def pread(fd: int, n: int, offset: int) -> bytes:
    # Read n bytes from file descriptor fd at the given offset, without moving the file pointer.
    pass

def preadv(fd: int, buffers: Sequence[Any], offset: int, flags: int = 0) -> int:
    # Read from fd into multiple buffers starting at offset.
    pass

def pwrite(fd: int, str: bytes, offset: int) -> int:
    # Write bytes to file descriptor fd at the given offset, without moving the file pointer.
    pass

def pwritev(fd: int, buffers: Sequence[Any], offset: int, flags: int = 0) -> int:
    # Write multiple buffers to fd starting at offset.
    pass

def read(fd: int, n: int) -> bytes:
    # Read at most n bytes from file descriptor fd.
    pass

def readv(fd: int, buffers: Sequence[Any]) -> int:
    # Read from fd into multiple buffers.
    pass

def sendfile(out_fd: int, in_fd: int, offset: int | None, count: int) -> int:
    # Copy count bytes from in_fd to out_fd, starting at offset.
    pass

def tcgetpgrp(fd: int) -> int:
    # Return the process group associated with the terminal given by fd.
    pass

def tcsetpgrp(fd: int, pg: int) -> None:
    # Set the process group associated with the terminal given by fd.
    pass

def ttyname(fd: int) -> str:
    # Return the name of the terminal device associated with fd.
    pass

def write(fd: int, str: bytes) -> int:
    # Write bytes to file descriptor fd.
    pass

def writev(fd: int, buffers: Sequence[Any]) -> int:
    # Write the contents of multiple buffers to fd.
    pass


# ---------------------------------------------------------------------------
# Files and Directories
# ---------------------------------------------------------------------------

def access(path: _Path, mode: int, *, dir_fd: int | None = None, effective_ids: bool = False, follow_symlinks: bool = True) -> bool:
    # Test whether the current process has the given access rights to path.
    pass

def chflags(path: _Path, flags: int, *, follow_symlinks: bool = True) -> None:
    # Set the flags of path to the numeric flags (BSD).
    pass

def chmod(path: _Path, mode: int, *, dir_fd: int | None = None, follow_symlinks: bool = True) -> None:
    # Change the mode of path to the numeric mode.
    pass

def chown(path: _Path, uid: int, gid: int, *, dir_fd: int | None = None, follow_symlinks: bool = True) -> None:
    # Change the owner and group id of path.
    pass

def chroot(path: _Path) -> None:
    # Change the root directory of the current process to path.
    pass

def lchflags(path: _Path, flags: int) -> None:
    # Set the flags of path to the numeric flags, without following symlinks.
    pass

def lchmod(path: _Path, mode: int) -> None:
    # Change the mode of path, without following symlinks.
    pass

def lchown(path: _Path, uid: int, gid: int) -> None:
    # Change the owner and group id of path, without following symlinks.
    pass

def link(src: _Path, dst: _Path, *, src_dir_fd: int | None = None, dst_dir_fd: int | None = None, follow_symlinks: bool = True) -> None:
    # Create a hard link pointing to src named dst.
    pass

def listdir(path: _Path = ".") -> list[Any]:
    # Return a list of entry names in the given directory.
    pass

def listdrives() -> list[str]:
    # Return a list of drives on the system (Windows).
    pass

def listmounts(volume: str) -> list[str]:
    # Return a list of all mount points for a volume (Windows).
    pass

def listxattr(path: _Path | None = None, *, follow_symlinks: bool = True) -> list[str]:
    # Return a list of extended attribute names on path.
    pass

def lstat(path: _Path, *, dir_fd: int | None = None) -> stat_result:
    # Like stat(), but does not follow symbolic links.
    pass

def getxattr(path: _Path, attribute: _Path, *, follow_symlinks: bool = True) -> bytes:
    # Return the value of an extended filesystem attribute.
    pass

def setxattr(path: _Path, attribute: _Path, value: bytes, flags: int = 0, *, follow_symlinks: bool = True) -> None:
    # Set the value of an extended filesystem attribute.
    pass

def removexattr(path: _Path, attribute: _Path, *, follow_symlinks: bool = True) -> None:
    # Remove an extended filesystem attribute from path.
    pass

def mkdir(path: _Path, mode: int = 0o777, *, dir_fd: int | None = None) -> None:
    # Create a directory named path.
    pass

def makedirs(name: _Path, mode: int = 0o777, exist_ok: bool = False) -> None:
    # Recursively create a directory and all missing intermediate directories.
    pass

def mkfifo(path: _Path, mode: int = 0o666, *, dir_fd: int | None = None) -> None:
    # Create a FIFO (named pipe) at path.
    pass

def mknod(path: _Path, mode: int = 0o600, device: int = 0, *, dir_fd: int | None = None) -> None:
    # Create a filesystem node at path.
    pass

def major(device: int) -> int:
    # Extract the major device number from a raw device number.
    pass

def minor(device: int) -> int:
    # Extract the minor device number from a raw device number.
    pass

def makedev(major: int, minor: int) -> int:
    # Compose a raw device number from major and minor device numbers.
    pass

def pathconf(path: _Path, name: str | int) -> int:
    # Return system configuration information for the given path.
    pass

def readlink(path: _Path, *, dir_fd: int | None = None) -> Any:
    # Return the target of a symbolic link.
    pass

def remove(path: _Path, *, dir_fd: int | None = None) -> None:
    # Remove (delete) a file.
    pass

def removedirs(name: _Path) -> None:
    # Remove a directory and, recursively, any now-empty parent directories.
    pass

def rename(src: _Path, dst: _Path, *, src_dir_fd: int | None = None, dst_dir_fd: int | None = None) -> None:
    # Rename src to dst.
    pass

def renames(old: _Path, new: _Path) -> None:
    # Rename old to new, creating intermediate directories and cleaning up old ones.
    pass

def replace(src: _Path, dst: _Path, *, src_dir_fd: int | None = None, dst_dir_fd: int | None = None) -> None:
    # Rename src to dst, overwriting dst if it exists.
    pass

def rmdir(path: _Path, *, dir_fd: int | None = None) -> None:
    # Remove (delete) an empty directory.
    pass

def scandir(path: _Path = ".") -> Iterator[DirEntry[Any]]:
    # Return an iterator of DirEntry objects for entries in the given directory.
    pass

def stat(path: _Path, *, dir_fd: int | None = None, follow_symlinks: bool = True) -> stat_result:
    # Return status information about a path.
    pass

def statvfs(path: _Path) -> statvfs_result:
    # Return file system status information for the file system containing path.
    pass

def symlink(src: _Path, dst: _Path, target_is_directory: bool = False, *, dir_fd: int | None = None) -> None:
    # Create a symbolic link pointing to src named dst.
    pass

def sync() -> None:
    # Force write of everything to disk.
    pass

def truncate(path: _Path, length: int) -> None:
    # Truncate the file at path to length bytes.
    pass

def unlink(path: _Path, *, dir_fd: int | None = None) -> None:
    # Remove (delete) a file.
    pass

def utime(path: _Path, times: tuple[int, int] | tuple[float, float] | None = None, *, ns: tuple[int, int] | None = None, dir_fd: int | None = None, follow_symlinks: bool = True) -> None:
    # Set the access and modified times of path.
    pass

def walk(top: _Path, topdown: bool = True, onerror: Callable[[OSError], object] | None = None, followlinks: bool = False) -> Iterator[tuple[Any, list[Any], list[Any]]]:
    # Generate (dirpath, dirnames, filenames) tuples for a directory tree.
    pass

def fwalk(top: _Path = ".", topdown: bool = True, onerror: Callable[[OSError], object] | None = None, *, follow_symlinks: bool = False, dir_fd: int | None = None) -> Iterator[tuple[Any, list[Any], list[Any], int]]:
    # Like walk(), but also yields the file descriptor of each directory.
    pass


# ---------------------------------------------------------------------------
# Process Management
# ---------------------------------------------------------------------------

def abort() -> None:
    # Generate a SIGABRT signal, typically terminating the process abnormally.
    pass

def execl(path: _Path, *args: Any) -> None:
    # Replace the current process image, passing args individually.
    pass

def execle(path: _Path, *args: Any) -> None:
    # Replace the current process image, passing args and an environment mapping.
    pass

def execlp(file: _Path, *args: Any) -> None:
    # Replace the current process image, searching PATH for file.
    pass

def execlpe(file: _Path, *args: Any) -> None:
    # Replace the current process image, searching PATH and passing an environment mapping.
    pass

def execv(path: _Path, args: Sequence[str]) -> None:
    # Replace the current process image, passing args as a sequence.
    pass

def execve(path: _Path, args: Sequence[str], env: Mapping[str, str]) -> None:
    # Replace the current process image, passing args and an explicit environment.
    pass

def execvp(file: _Path, args: Sequence[str]) -> None:
    # Replace the current process image, searching PATH for file.
    pass

def execvpe(file: _Path, args: Sequence[str], env: Mapping[str, str]) -> None:
    # Replace the current process image, searching PATH and passing an explicit environment.
    pass

def _exit(n: int) -> None:
    # Exit the process immediately with status n, without cleanup.
    pass

def fork() -> int:
    # Fork a child process, returning 0 in the child and the child's pid in the parent.
    pass

def forkpty() -> tuple[int, int]:
    # Fork a child process attached to a new pseudo-terminal.
    pass

def kill(pid: int, sig: int) -> None:
    # Send signal sig to the process with the given pid.
    pass

def killpg(pgid: int, sig: int) -> None:
    # Send signal sig to the process group pgid.
    pass

def nice(increment: int) -> int:
    # Add increment to the process's scheduling priority, returning the new value.
    pass

def plock(op: int) -> None:
    # Lock program segments into memory (rarely supported).
    pass

def posix_spawn(path: _Path, argv: Sequence[str], env: Mapping[str, str], **kwargs: Any) -> int:
    # Spawn a new process using the posix_spawn() system call.
    pass

def posix_spawnp(path: _Path, argv: Sequence[str], env: Mapping[str, str], **kwargs: Any) -> int:
    # Like posix_spawn(), searching PATH for the executable.
    pass

def register_at_fork(*, before: Callable[[], Any] | None = None, after_in_parent: Callable[[], Any] | None = None, after_in_child: Callable[[], Any] | None = None) -> None:
    # Register callbacks to run before and after os.fork().
    pass

def spawnl(mode: int, path: _Path, *args: Any) -> int:
    # Spawn a process, passing args individually.
    pass

def spawnle(mode: int, path: _Path, *args: Any) -> int:
    # Spawn a process, passing args individually and an environment mapping.
    pass

def spawnlp(mode: int, file: _Path, *args: Any) -> int:
    # Spawn a process, searching PATH for file.
    pass

def spawnlpe(mode: int, file: _Path, *args: Any) -> int:
    # Spawn a process, searching PATH and passing an environment mapping.
    pass

def spawnv(mode: int, path: _Path, args: Sequence[str]) -> int:
    # Spawn a process, passing args as a sequence.
    pass

def spawnve(mode: int, path: _Path, args: Sequence[str], env: Mapping[str, str]) -> int:
    # Spawn a process, passing args and an explicit environment.
    pass

def spawnvp(mode: int, file: _Path, args: Sequence[str]) -> int:
    # Spawn a process, searching PATH for file.
    pass

def spawnvpe(mode: int, file: _Path, args: Sequence[str], env: Mapping[str, str]) -> int:
    # Spawn a process, searching PATH and passing an explicit environment.
    pass

def startfile(path: _Path, operation: str = "open") -> None:
    # Start a file with its associated application (Windows).
    pass

def system(command: str) -> int:
    # Execute a shell command and return its exit status.
    pass

def times() -> times_result:
    # Return accumulated process and child process CPU times.
    pass

def wait() -> tuple[int, int]:
    # Wait for completion of a child process, returning (pid, exit_status).
    pass

def waitid(idtype: int, id: int, options: int) -> waitid_result | None:
    # Wait for completion of one or more child processes matching criteria.
    pass

def waitpid(pid: int, options: int) -> tuple[int, int]:
    # Wait for completion of a specific child process.
    pass

def wait3(options: int) -> tuple[int, int, Any]:
    # Like waitpid(), for any child, without specifying a pid.
    pass

def wait4(pid: int, options: int) -> tuple[int, int, Any]:
    # Like waitpid(), also returning resource usage information.
    pass

def WCOREDUMP(status: int) -> bool:
    # Return whether a core dump was generated for the process that caused status.
    pass

def WIFCONTINUED(status: int) -> bool:
    # Return whether the process was resumed from a job control stop.
    pass

def WIFSTOPPED(status: int) -> bool:
    # Return whether the process was stopped.
    pass

def WIFSIGNALED(status: int) -> bool:
    # Return whether the process exited due to a signal.
    pass

def WIFEXITED(status: int) -> bool:
    # Return whether the process exited normally, via exit() or return.
    pass

def WEXITSTATUS(status: int) -> int:
    # Return the process's exit status.
    pass

def WSTOPSIG(status: int) -> int:
    # Return the signal that caused the process to stop.
    pass

def WTERMSIG(status: int) -> int:
    # Return the signal that caused the process to exit.
    pass


# ---------------------------------------------------------------------------
# Miscellaneous System Information
# ---------------------------------------------------------------------------

def confstr(name: str | int) -> str | None:
    # Return string-valued system configuration information.
    pass

def sysconf(name: str | int) -> int:
    # Return integer-valued system configuration information.
    pass

def urandom(size: int) -> bytes:
    # Return size random bytes suitable for cryptographic use.
    pass

def getrandom(size: int, flags: int = 0) -> bytes:
    # Return size random bytes from the operating system's random number generator.
    pass
