package it.unive.pylisa.libraries.svverifier;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyTypeTokenType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;
import java.util.Set;
import java.util.function.Predicate;

/**
 * Native implementation of {@code _sv_verifier.check_type(value, hint)}, that
 * returns {@code None} if {@code value} has the type described by {@code hint},
 * and raises {@code TypeError} otherwise. The supported hints are {@code int}
 * (also satisfied by a {@code bool}), {@code float}, {@code str}, {@code bytes}
 * and {@code None}, and the classes of the program (also satisfied by the
 * objects of their subclasses; since the bases of the classes are not tracked,
 * both outcomes are possible for an object of a class with bases); a function
 * that is not a type (e.g. {@code len}) is never satisfied, as
 * {@code isinstance} raises {@code TypeError} on it. For any other hint (e.g.
 * {@code list[int]}), both outcomes are possible.
 */
public class CheckType extends PyNative {

	protected CheckType(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "check_type", params);
	}

	public static CheckType build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new CheckType(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		SymbolicExpression value = args[0];
		Set<Type> hints = analysis.getRuntimeTypesOf(state, args[1], this);
		boolean matches = false, fails = false;
		if (hints.isEmpty())
			matches = fails = true;
		for (Type hint : hints) {
			CompilationUnit cls = classOf(hint);
			Predicate<Type> accepted = cls == null ? accepted(hint) : null;
			Satisfiability sat = cls != null ? instanceOf(analysis, state, value, cls)
					: accepted == null ? Satisfiability.UNKNOWN : hasType(analysis, state, value, accepted);
			matches |= sat != Satisfiability.NOT_SATISFIED;
			fails |= sat != Satisfiability.SATISFIED;
		}

		AnalysisState<A> result = state.bottom();
		if (matches)
			result = result.lub(compute(analysis, state, new PyNoneConstant(getLocation())));
		if (fails)
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}

	// the types of the values described by a hint of the given type, or null
	// if the hint is not supported
	private static Predicate<Type> accepted(
			Type hint) {
		if (hint.isNullType())
			return NONE;
		if (hint.isCodeMemberType()) {
			Set<CodeMember> members = hint.asCodeMemberType().getMembers();
			CodeMember function = members.size() == 1 ? members.iterator().next() : null;
			if (function == null)
				return null;
			if (!(function instanceof NativeCFG) || !(function.getDescriptor().getUnit() instanceof Program))
				// isinstance raises TypeError if its second argument is not a
				// type
				return t -> false;
			switch (function.getDescriptor().getName()) {
			case "int":
				return INT;
			case "float":
				return t -> t.isNumericType() && !t.asNumericType().isIntegral();
			case "str":
				return STR;
			case "bytes":
				return BYTES;
			case "super":
				return null;
			default:
				// a builtin function, that is not a type (e.g. len)
				return t -> false;
			}
		}
		return null;
	}

	// the class of the program described by a hint of the given type (see
	// PyClassRef), or null if the hint is not a class
	private static CompilationUnit classOf(
			Type hint) {
		if (!(hint instanceof PyTypeTokenType))
			return null;
		Set<Type> classes = hint.asTypeTokenType().getTypes();
		if (classes.size() != 1 || !(classes.iterator().next() instanceof PyClassType))
			return null;
		return ((PyClassType) classes.iterator().next()).getUnit();
	}

	// whether value is an object of cls, or of one of its subclasses: the
	// bases of the classes are not tracked (see PyClassParser), so an object
	// of a class with bases might be an instance of any class
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Satisfiability instanceOf(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression value,
			CompilationUnit cls)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, value, this);
		if (types.isEmpty() || types.stream().anyMatch(Type::isUntyped))
			return Satisfiability.UNKNOWN;
		boolean maybe = false, maybeNot = false;
		for (Type t : types) {
			Type inner = t.isPointerType() ? t.asPointerType().getInnerType() : null;
			CompilationUnit unit = inner != null && inner.isUnitType()
					&& inner.asUnitType().getUnit() instanceof CompilationUnit
							? (CompilationUnit) inner.asUnitType().getUnit()
							: null;
			if (unit != null && unit.isInstanceOf(cls))
				maybe = true;
			else if (unit != null && unit.getAnnotationList().stream()
					.anyMatch(a -> a.getAnnotationName().equals(PyBinaryDispatch.UNTRACKED_BASES)))
				maybe = maybeNot = true;
			else
				maybeNot = true;
		}
		if (!maybeNot)
			return Satisfiability.SATISFIED;
		return maybe ? Satisfiability.UNKNOWN : Satisfiability.NOT_SATISFIED;
	}
}
