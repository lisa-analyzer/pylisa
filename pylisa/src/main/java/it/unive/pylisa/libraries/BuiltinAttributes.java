package it.unive.pylisa.libraries;

import java.util.Map;
import java.util.Set;

/**
 * The attributes of the objects of the builtin classes {@code int},
 * {@code bool}, {@code float}, {@code str}, {@code bytes} and {@code NoneType}
 * (the names in {@code dir} of each class, as of python 3.13.2, plus
 * {@code float.from_number}, the only one added in python 3.14, that the
 * benchmarks assume by default). Reading any other
 * attribute of their objects (e.g. {@code (5).strip}) raises
 * {@code AttributeError}. Builtin classes gain attributes in newer versions of
 * python, and practically never lose them: an attribute of a newer version is
 * then not known, rather than missing.
 */
public final class BuiltinAttributes {

	// dir(int)
	private static final Set<String> INT = Set.of(
			"__abs__", "__add__", "__and__", "__bool__", "__ceil__", "__class__", "__delattr__", "__dir__",
			"__divmod__", "__doc__", "__eq__", "__float__", "__floor__", "__floordiv__", "__format__", "__ge__",
			"__getattribute__", "__getnewargs__", "__getstate__", "__gt__", "__hash__", "__index__", "__init__",
			"__init_subclass__", "__int__", "__invert__", "__le__", "__lshift__", "__lt__", "__mod__", "__mul__",
			"__ne__", "__neg__", "__new__", "__or__", "__pos__", "__pow__", "__radd__", "__rand__", "__rdivmod__",
			"__reduce__", "__reduce_ex__", "__repr__", "__rfloordiv__", "__rlshift__", "__rmod__", "__rmul__",
			"__ror__", "__round__", "__rpow__", "__rrshift__", "__rshift__", "__rsub__", "__rtruediv__", "__rxor__",
			"__setattr__", "__sizeof__", "__str__", "__sub__", "__subclasshook__", "__truediv__", "__trunc__",
			"__xor__", "as_integer_ratio", "bit_count", "bit_length", "conjugate", "denominator", "from_bytes",
			"imag", "is_integer", "numerator", "real", "to_bytes");

	// dir(bool)
	private static final Set<String> BOOL = Set.of(
			"__abs__", "__add__", "__and__", "__bool__", "__ceil__", "__class__", "__delattr__", "__dir__",
			"__divmod__", "__doc__", "__eq__", "__float__", "__floor__", "__floordiv__", "__format__", "__ge__",
			"__getattribute__", "__getnewargs__", "__getstate__", "__gt__", "__hash__", "__index__", "__init__",
			"__init_subclass__", "__int__", "__invert__", "__le__", "__lshift__", "__lt__", "__mod__", "__mul__",
			"__ne__", "__neg__", "__new__", "__or__", "__pos__", "__pow__", "__radd__", "__rand__", "__rdivmod__",
			"__reduce__", "__reduce_ex__", "__repr__", "__rfloordiv__", "__rlshift__", "__rmod__", "__rmul__",
			"__ror__", "__round__", "__rpow__", "__rrshift__", "__rshift__", "__rsub__", "__rtruediv__", "__rxor__",
			"__setattr__", "__sizeof__", "__str__", "__sub__", "__subclasshook__", "__truediv__", "__trunc__",
			"__xor__", "as_integer_ratio", "bit_count", "bit_length", "conjugate", "denominator", "from_bytes",
			"imag", "is_integer", "numerator", "real", "to_bytes");

	// dir(float), with from_number (added in python 3.14)
	private static final Set<String> FLOAT = Set.of(
			"__abs__", "__add__", "__bool__", "__ceil__", "__class__", "__delattr__", "__dir__", "__divmod__",
			"__doc__", "__eq__", "__float__", "__floor__", "__floordiv__", "__format__", "__ge__", "__getattribute__",
			"__getformat__", "__getnewargs__", "__getstate__", "__gt__", "__hash__", "__init__", "__init_subclass__",
			"__int__", "__le__", "__lt__", "__mod__", "__mul__", "__ne__", "__neg__", "__new__", "__pos__", "__pow__",
			"__radd__", "__rdivmod__", "__reduce__", "__reduce_ex__", "__repr__", "__rfloordiv__", "__rmod__",
			"__rmul__", "__round__", "__rpow__", "__rsub__", "__rtruediv__", "__setattr__", "__sizeof__", "__str__",
			"__sub__", "__subclasshook__", "__truediv__", "__trunc__", "as_integer_ratio", "conjugate", "from_number",
			"fromhex", "hex", "imag", "is_integer", "real");

	// dir(str)
	private static final Set<String> STR = Set.of(
			"__add__", "__class__", "__contains__", "__delattr__", "__dir__", "__doc__", "__eq__", "__format__",
			"__ge__", "__getattribute__", "__getitem__", "__getnewargs__", "__getstate__", "__gt__", "__hash__",
			"__init__", "__init_subclass__", "__iter__", "__le__", "__len__", "__lt__", "__mod__", "__mul__",
			"__ne__", "__new__", "__reduce__", "__reduce_ex__", "__repr__", "__rmod__", "__rmul__", "__setattr__",
			"__sizeof__", "__str__", "__subclasshook__", "capitalize", "casefold", "center", "count", "encode",
			"endswith", "expandtabs", "find", "format", "format_map", "index", "isalnum", "isalpha", "isascii",
			"isdecimal", "isdigit", "isidentifier", "islower", "isnumeric", "isprintable", "isspace", "istitle",
			"isupper", "join", "ljust", "lower", "lstrip", "maketrans", "partition", "removeprefix", "removesuffix",
			"replace", "rfind", "rindex", "rjust", "rpartition", "rsplit", "rstrip", "split", "splitlines",
			"startswith", "strip", "swapcase", "title", "translate", "upper", "zfill");

	// dir(bytes)
	private static final Set<String> BYTES = Set.of(
			"__add__", "__buffer__", "__bytes__", "__class__", "__contains__", "__delattr__", "__dir__", "__doc__",
			"__eq__", "__format__", "__ge__", "__getattribute__", "__getitem__", "__getnewargs__", "__getstate__",
			"__gt__", "__hash__", "__init__", "__init_subclass__", "__iter__", "__le__", "__len__", "__lt__",
			"__mod__", "__mul__", "__ne__", "__new__", "__reduce__", "__reduce_ex__", "__repr__", "__rmod__",
			"__rmul__", "__setattr__", "__sizeof__", "__str__", "__subclasshook__", "capitalize", "center", "count",
			"decode", "endswith", "expandtabs", "find", "fromhex", "hex", "index", "isalnum", "isalpha", "isascii",
			"isdigit", "islower", "isspace", "istitle", "isupper", "join", "ljust", "lower", "lstrip", "maketrans",
			"partition", "removeprefix", "removesuffix", "replace", "rfind", "rindex", "rjust", "rpartition",
			"rsplit", "rstrip", "split", "splitlines", "startswith", "strip", "swapcase", "title", "translate",
			"upper", "zfill");

	// dir(NoneType)
	private static final Set<String> NONE_TYPE = Set.of(
			"__bool__", "__class__", "__delattr__", "__dir__", "__doc__", "__eq__", "__format__", "__ge__",
			"__getattribute__", "__getstate__", "__gt__", "__hash__", "__init__", "__init_subclass__", "__le__",
			"__lt__", "__ne__", "__new__", "__reduce__", "__reduce_ex__", "__repr__", "__setattr__", "__sizeof__",
			"__str__", "__subclasshook__");

	// the attributes of the library classes of the builtin values
	private static final Map<String, Set<String>> ATTRIBUTES = Map.of(
			LibrarySpecificationProvider.INT, INT,
			LibrarySpecificationProvider.BOOL, BOOL,
			LibrarySpecificationProvider.FLOAT, FLOAT,
			LibrarySpecificationProvider.STR, STR,
			LibrarySpecificationProvider.BYTES, BYTES,
			LibrarySpecificationProvider.NONE_TYPE, NONE_TYPE);

	private BuiltinAttributes() {
	}

	/**
	 * Whether the objects of the given library class, that is the class of
	 * builtin values (see {@code PyBinaryDispatch#classOf}), surely do not
	 * have the given attribute.
	 *
	 * @param cls  the name of the library class
	 * @param name the name of the attribute
	 *
	 * @return {@code true} if reading the attribute raises
	 *             {@code AttributeError}, {@code false} if the objects have it,
	 *             or if the attributes of the class are not known
	 */
	public static boolean missing(
			String cls,
			String name) {
		Set<String> attributes = cls == null ? null : ATTRIBUTES.get(cls);
		return attributes != null && !attributes.contains(name);
	}
}
