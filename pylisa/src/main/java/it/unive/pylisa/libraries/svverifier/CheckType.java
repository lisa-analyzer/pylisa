package it.unive.pylisa.libraries.svverifier;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
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
 * and {@code None}; a function that is not a type (e.g. {@code len}) is never
 * satisfied, as {@code isinstance} raises {@code TypeError} on it. For any
 * other hint (e.g. a class, or {@code list[int]}), both outcomes are possible.
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
			Predicate<Type> accepted = accepted(hint);
			Satisfiability sat = accepted == null ? Satisfiability.UNKNOWN
					: hasType(analysis, state, value, accepted);
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
}
