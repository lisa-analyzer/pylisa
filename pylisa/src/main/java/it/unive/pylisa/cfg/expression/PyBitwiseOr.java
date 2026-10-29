package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyTypeTokenType;
import java.util.HashSet;
import java.util.Set;
import org.apache.commons.collections4.SetUtils;

/**
 * Python's {@code a | b}: it calls {@code type(a).__or__(a, b)}, falling back
 * to {@code type(b).__ror__(b, a)}; if neither applies, a {@code TypeError} is
 * raised. See {@link PyBinaryDispatch} for the details. On types (e.g.
 * {@code int | str}, or {@code int | None}), it builds the union of the types:
 * its value is an unknown type object.
 */
public class PyBitwiseOr extends BinaryExpression {

	// the builtin functions that are types, so that | builds a union on them
	private static final Set<String> BUILTIN_TYPES = Set.of("int", "float", "str", "bytes", "bool", "list",
			"dict", "tuple", "set", "frozenset", "object", "type");

	public PyBitwiseOr(
			CFG cfg,
			CodeLocation loc,
			Expression left,
			Expression right) {
		super(cfg, loc, "|", Untyped.INSTANCE, left, right);
	}

	@Override
	protected int compareSameClassAndParams(
			Statement o) {
		return 0;
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fwdBinarySemantics(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			SymbolicExpression left,
			SymbolicExpression right,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		Set<Type> ltypes = analysis.getRuntimeTypesOf(state, left, context, this);
		Set<Type> rtypes = analysis.getRuntimeTypesOf(state, right, context, this);
		if (types(ltypes, true) && types(rtypes, true) && (types(ltypes, false) || types(rtypes, false)))
			// a union of types (None | None is not one)
			return analysis.smallStepSemantics(state,
					new PushAny(new PyTypeTokenType(members(ltypes, rtypes)), getLocation()), context, this);
		return PyBinaryDispatch.dispatch(interprocedural, state, expressions, this, left, right,
				"__or__", "__ror__", false, PyBinaryDispatch.Fallback.TYPE_ERROR);
	}

	/**
	 * The members of the union of two types, given the runtime types of the
	 * operands: the types of the values described by the classes (as
	 * {@link PyClassType}s), the builtin types (as the types of their
	 * functions, e.g. {@code int}), {@code None} (as the null type), and the
	 * members of the unions (as for {@code int | str | None}). If an operand
	 * might be different types (e.g. either {@code int} or {@code str}), the
	 * union is unknown, and its only member is {@link Untyped}.
	 */
	private static Set<Type> members(
			Set<Type> left,
			Set<Type> right) {
		if (left.size() != 1 || right.size() != 1)
			return Set.of(Untyped.INSTANCE);
		Set<Type> members = new HashSet<>();
		for (Type t : SetUtils.union(left, right))
			if (t instanceof PyTypeTokenType)
				members.addAll(t.asTypeTokenType().getTypes());
			else
				members.add(t);
		return members;
	}

	// whether all the values of the given runtime types are types (classes,
	// builtin types, or None if allowed)
	private static boolean types(
			Set<Type> types,
			boolean none) {
		if (types.isEmpty())
			return false;
		for (Type t : types)
			if (t.isNullType()) {
				if (!none)
					return false;
			} else if (!(t instanceof PyTypeTokenType) && !builtinType(t))
				return false;
		return true;
	}

	// whether the values of the given type are builtin types (e.g. int)
	private static boolean builtinType(
			Type t) {
		if (!t.isCodeMemberType())
			return false;
		for (CodeMember member : t.asCodeMemberType().getMembers())
			if (!(member instanceof NativeCFG) || !(member.getDescriptor().getUnit() instanceof Program)
					|| !BUILTIN_TYPES.contains(member.getDescriptor().getName()))
				return false;
		return true;
	}
}
