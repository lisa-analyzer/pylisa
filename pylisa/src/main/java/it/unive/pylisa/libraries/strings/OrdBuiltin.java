package it.unive.pylisa.libraries.strings;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import java.util.Set;

/**
 * Native implementation of {@code ord(c)}: the code point of a {@code str} of
 * one character (or the value of a {@code bytes} of one byte), an {@code int}
 * that is known when {@code c} is a constant. A {@code str} (or a
 * {@code bytes}) of another length, and the other builtin values (e.g. an
 * {@code int}), raise {@code TypeError}. For objects, the result is unknown.
 */
public class OrdBuiltin extends PyNative {

	protected OrdBuiltin(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "ord", params);
	}

	public static OrdBuiltin build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new OrdBuiltin(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression c = args[0];
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		if (c instanceof Constant && ((Constant) c).getValue() instanceof String) {
			String s = (String) ((Constant) c).getValue();
			if (s.codePointCount(0, s.length()) == 1)
				return compute(analysis, context, state, new Constant(Int32Type.INSTANCE, s.codePointAt(0), loc));
			return raise(analysis, context, state, LibrarySpecificationProvider.TYPE_ERROR);
		}

		Set<Type> types = analysis.getRuntimeTypesOf(state, c, context, this);
		boolean unknown = types.isEmpty() || types.stream().anyMatch(Type::isUntyped);
		boolean text = unknown || types.stream().anyMatch(t -> STR.test(t) || BYTES.test(t));
		boolean object = unknown || types.stream().anyMatch(t -> !PyBinaryDispatch.isBuiltinValueType(t));
		boolean other = unknown || types.stream().anyMatch(t -> !STR.test(t) && !BYTES.test(t)
				&& PyBinaryDispatch.isBuiltinValueType(t));
		AnalysisState<A> result = state.bottomExecution();
		if (text) {
			// its length is not known
			result = result.lub(compute(analysis, context, state, new PushAny(Int32Type.INSTANCE, loc)));
			raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.TYPE_ERROR);
		}
		if (other)
			raise(raising, Satisfiability.UNKNOWN, state, LibrarySpecificationProvider.TYPE_ERROR);
		if (object)
			result = result.lub(unknown(analysis, context, state, null));
		return raising.moveFrom(result);
	}
}
