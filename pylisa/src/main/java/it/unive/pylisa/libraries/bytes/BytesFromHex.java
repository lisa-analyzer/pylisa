package it.unive.pylisa.libraries.bytes;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.ExceptionGuard;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.bytes.BytesUnary;
import it.unive.pylisa.symbolic.operators.bytes.FromHexRaises;

/**
 * Native implementation of the class method {@code bytes.fromhex(string)}:
 * {@code string} must be a {@code str} or, since python 3.14, {@code bytes}
 * (otherwise {@code TypeError} is raised), made of pairs of hexadecimal digits
 * optionally separated by ASCII whitespace (otherwise {@code ValueError} is
 * raised).
 */
public class BytesFromHex extends PyNative {

	protected BytesFromHex(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "fromhex", params);
	}

	public static BytesFromHex build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new BytesFromHex(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		// a str or, since python 3.14, bytes
		Satisfiability typed = hasType(analysis, context, state, args[0], STR.or(BYTES));
		PyExceptions.Raising<A, D> raising = new PyExceptions.Raising<>(analysis, context);
		AnalysisState<A> ifTyped = typed == Satisfiability.NOT_SATISFIED ? state.bottomExecution()
				: ExceptionGuard.guardedCompute(analysis, context, state,
						new UnaryExpression(BoolType.INSTANCE, args[0], FromHexRaises.INSTANCE, loc),
						LibrarySpecificationProvider.VALUE_ERROR,
						new UnaryExpression(PyBytesType.INSTANCE, args[0], BytesUnary.FROMHEX, loc), getCFG(), loc,
						getOriginatingStatement(), getOriginatingStatement(), raising);
		return raising.moveFrom(typeChecked(state, typed, ifTyped, raising));
	}
}
