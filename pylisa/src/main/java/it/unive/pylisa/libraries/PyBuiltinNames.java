package it.unive.pylisa.libraries;

import java.util.Set;

/**
 * The names that are always defined in a Python module, even if they are not
 * bound by the module itself: the names of the {@code builtins} module (as of
 * Python 3.12), and the attributes that the interpreter sets on each module
 * (e.g. {@code __name__}). Reading one of these names when it is not bound in
 * the module never raises {@code NameError}, even if the builtin is not modeled
 * by the analysis.
 */
public final class PyBuiltinNames {

	private static final Set<String> NAMES = Set.of(
			// constants
			"False", "True", "None", "Ellipsis", "NotImplemented", "__debug__",
			// module attributes
			"__name__", "__doc__", "__file__", "__package__", "__spec__", "__loader__", "__builtins__",
			"__annotations__", "__dict__", "__cached__", "__path__",
			// functions and classes
			"abs", "aiter", "all", "anext", "any", "ascii", "bin", "bool", "breakpoint", "bytearray", "bytes",
			"callable", "chr", "classmethod", "compile", "complex", "copyright", "credits", "delattr", "dict", "dir",
			"divmod", "enumerate", "eval", "exec", "exit", "filter", "float", "format", "frozenset", "getattr",
			"globals", "hasattr", "hash", "help", "hex", "id", "input", "int", "isinstance", "issubclass", "iter",
			"len", "license", "list", "locals", "map", "max", "memoryview", "min", "next", "object", "oct", "open",
			"ord", "pow", "print", "property", "quit", "range", "repr", "reversed", "round", "set", "setattr",
			"slice", "sorted", "staticmethod", "str", "sum", "super", "tuple", "type", "vars", "zip",
			// exceptions and warnings
			"ArithmeticError", "AssertionError", "AttributeError", "BaseException", "BaseExceptionGroup",
			"BlockingIOError", "BrokenPipeError", "BufferError", "BytesWarning", "ChildProcessError",
			"ConnectionAbortedError", "ConnectionError", "ConnectionRefusedError", "ConnectionResetError",
			"DeprecationWarning", "EncodingWarning", "EnvironmentError", "EOFError", "Exception", "ExceptionGroup",
			"FileExistsError", "FileNotFoundError", "FloatingPointError", "FutureWarning", "GeneratorExit",
			"ImportError", "ImportWarning", "IndentationError", "IndexError", "InterruptedError",
			"IsADirectoryError", "IOError", "KeyboardInterrupt", "KeyError", "LookupError", "MemoryError",
			"ModuleNotFoundError", "NameError", "NotADirectoryError", "NotImplementedError", "OSError",
			"OverflowError", "PendingDeprecationWarning", "PermissionError", "ProcessLookupError",
			"RecursionError", "ReferenceError", "ResourceWarning", "RuntimeError", "RuntimeWarning",
			"StopAsyncIteration", "StopIteration", "SyntaxError", "SyntaxWarning", "SystemError", "SystemExit",
			"TabError", "TimeoutError", "TypeError", "UnboundLocalError", "UnicodeDecodeError",
			"UnicodeEncodeError", "UnicodeError", "UnicodeTranslateError", "UnicodeWarning", "UserWarning",
			"ValueError", "Warning", "ZeroDivisionError");

	private PyBuiltinNames() {
	}

	/**
	 * Yields whether the given name is always defined in a Python module (see
	 * {@link PyBuiltinNames}).
	 *
	 * @param name the name
	 *
	 * @return whether the name is a builtin one
	 */
	public static boolean isBuiltin(
			String name) {
		return NAMES.contains(name);
	}
}
