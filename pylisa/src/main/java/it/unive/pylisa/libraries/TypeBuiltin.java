package it.unive.pylisa.libraries;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.CodeMemberType;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.expression.PyBuiltinRef;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyTypeTokenType;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.Collections;
import java.util.Set;

/**
 * Native implementation of Python's {@code type(obj)}: the class of
 * {@code obj}, that is the same value as the name of the class (so that, e.g.,
 * {@code type(x) is int} and {@code type(x) is type(y)} can be decided, see
 * {@code PyBinaryDispatch#identical}). This is the builtin function
 * {@code int}, {@code float}, {@code str}, {@code bytes} or {@code list} for
 * their values (see {@code PyBuiltinRef}), and the class of an object (see
 * {@code PyTypeLiteral}). The class of any other value (e.g. a {@code bool}
 * or {@code None}, whose classes are not modeled) is unknown.
 */
public class TypeBuiltin extends PyNative {

	protected TypeBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "type", params);
	}

	public static TypeBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new TypeBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		Set<Type> types = analysis.getRuntimeTypesOf(state, args[0], this);
		if (types.isEmpty())
			return unknown(analysis, state, Untyped.INSTANCE);
		AnalysisState<A> result = state.bottomExecution();
		for (Type t : types) {
			Constant cls = classOf(t);
			result = result.lub(cls == null ? unknown(analysis, state, Untyped.INSTANCE)
					: compute(analysis, state, cls));
		}
		return result;
	}

	// the class of the values of type t, or null if it is not known
	private Constant classOf(
			Type t) {
		CodeLocation loc = getLocation();
		String builtin = null;
		if (t.isNumericType())
			builtin = t.asNumericType().isIntegral() ? "int" : "float";
		else if (t.isStringType())
			builtin = "str";
		else if (t instanceof PyBytesType)
			builtin = "bytes";
		else if (ListModel.isList(t))
			builtin = "list";
		if (builtin != null) {
			NativeCFG function = PyBuiltinRef.builtin(getCFG().getProgram(), builtin);
			return new Constant(new CodeMemberType(Collections.singleton(function)), function, loc);
		}

		Type inner = t.isPointerType() ? t.asPointerType().getInnerType() : null;
		if (inner instanceof PyClassType)
			return new Constant(new PyTypeTokenType(Collections.singleton(inner)), ((PyClassType) inner).getUnit(), loc);
		return null;
	}
}
