package it.unive.pylisa.libraries;

import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyTypeTokenType;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Set;
import java.util.function.Predicate;

/**
 * The type hints of {@code isinstance(value, hint)} and
 * {@code _sv_verifier.check_type(value, hint)}, given by the runtime type of
 * {@code hint}. The supported hints are {@code int} (also satisfied by a
 * {@code bool}), {@code float}, {@code str}, {@code bytes}, {@code list} and
 * {@code None}, the classes of the program (also satisfied by the objects of
 * their subclasses; since the bases of the classes are not tracked, both
 * outcomes are possible for an object of a class with bases) and their unions
 * (e.g. {@code int | None}, see {@code PyBitwiseOr}). A builtin function that
 * is not a type (e.g. {@code len}) is never satisfied. For any other hint (e.g.
 * {@code list[int]}), both outcomes are possible.
 */
public final class TypeHints {

	// the builtin functions that are not types
	private static final Set<String> FUNCTIONS = Set.of("print", "len", "input", "repr", "exec", "eval", "compile", "sum",
			"all", "any", "isinstance");

	// the hint of a function that is not a type
	private static final Predicate<Type> NOT_A_TYPE = t -> false;

	private TypeHints() {
	}

	/**
	 * Whether the values of type {@code t} have the type described by a hint of
	 * type {@code hint}.
	 *
	 * @param t    the runtime type of the value
	 * @param hint the runtime type of the hint
	 *
	 * @return whether the values satisfy the hint
	 */
	public static Satisfiability satisfies(
			Type t,
			Type hint) {
		if (t.isUntyped())
			return Satisfiability.UNKNOWN;
		if (hint instanceof PyClassType)
			// a member of a union
			return instanceOf(t, ((PyClassType) hint).getUnit());
		if (hint instanceof PyTypeTokenType) {
			// a class (see PyClassRef), or a union: satisfied if one of its
			// members is
			Satisfiability sat = Satisfiability.NOT_SATISFIED;
			for (Type member : hint.asTypeTokenType().getTypes())
				sat = sat.or(satisfies(t, member));
			return sat;
		}
		Predicate<Type> accepted = accepted(hint);
		if (accepted == null)
			return Satisfiability.UNKNOWN;
		return accepted.test(t) ? Satisfiability.SATISFIED : Satisfiability.NOT_SATISFIED;
	}

	/**
	 * Whether a hint of type {@code hint} is a type that {@code isinstance}
	 * accepts as its second argument: a class, a builtin type or a union.
	 * {@code None} (out of a union), the builtin functions that are not types
	 * (e.g. {@code len}) and the values of the builtin types (e.g. {@code 3})
	 * are not.
	 *
	 * @param hint the runtime type of the hint
	 *
	 * @return whether the hint is a type
	 */
	public static Satisfiability isType(
			Type hint) {
		if (hint instanceof PyTypeTokenType)
			return Satisfiability.SATISFIED;
		if (hint.isNullType() || PyBinaryDispatch.isBuiltinValueType(hint))
			return Satisfiability.NOT_SATISFIED;
		if (!hint.isCodeMemberType())
			// e.g. a tuple of types
			return Satisfiability.UNKNOWN;
		Predicate<Type> accepted = accepted(hint);
		if (accepted != null)
			return accepted == NOT_A_TYPE ? Satisfiability.NOT_SATISFIED : Satisfiability.SATISFIED;
		return "super".equals(builtin(hint)) || "type".equals(builtin(hint)) ? Satisfiability.SATISFIED
				: Satisfiability.UNKNOWN;
	}

	// the types of the values described by a hint of the given type, or null
	// if the hint is not supported
	private static Predicate<Type> accepted(
			Type hint) {
		if (hint.isNullType())
			return PyNative.NONE;
		String builtin = builtin(hint);
		if (builtin == null)
			return null;
		switch (builtin) {
		case "int":
			return PyNative.INT;
		case "float":
			return t -> t.isNumericType() && !t.asNumericType().isIntegral();
		case "str":
			return PyNative.STR;
		case "bytes":
			return PyNative.BYTES;
		case "list":
			return ListModel::isList;
		default:
			// a function of the program is not a type either
			return builtin.isEmpty() || FUNCTIONS.contains(builtin) ? NOT_A_TYPE : null;
		}
	}

	// the name of the builtin function of the given type, or null if it is not
	// one
	private static String builtin(
			Type hint) {
		if (!hint.isCodeMemberType())
			return null;
		Set<CodeMember> members = hint.asCodeMemberType().getMembers();
		if (members.size() != 1)
			return null;
		CodeMember function = members.iterator().next();
		if (!(function instanceof NativeCFG) || !(function.getDescriptor().getUnit() instanceof Program))
			// a function of the program, or of a library
			return "";
		return function.getDescriptor().getName();
	}

	// whether the values of type t are objects of cls, or of one of its
	// subclasses: the bases of the classes are not tracked (see
	// PyClassParser), so an object of a class with bases might be an instance
	// of any class
	private static Satisfiability instanceOf(
			Type t,
			CompilationUnit cls) {
		Type inner = t.isPointerType() ? t.asPointerType().getInnerType() : null;
		CompilationUnit unit = inner != null && inner.isUnitType()
				&& inner.asUnitType().getUnit() instanceof CompilationUnit
						? (CompilationUnit) inner.asUnitType().getUnit()
						: null;
		if (unit != null && unit.isInstanceOf(cls))
			return Satisfiability.SATISFIED;
		if (unit != null && unit.getAnnotationList().stream()
				.anyMatch(a -> a.getAnnotationName().equals(PyBinaryDispatch.UNTRACKED_BASES)))
			return Satisfiability.UNKNOWN;
		return Satisfiability.NOT_SATISFIED;
	}
}
