package it.unive.pylisa.libraries.bytes;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGt;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.LogicalOr;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.ExceptionGuard;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.bytes.BytesOperation;

/**
 * Native implementation of {@code bytes.__contains__(self, item)}: {@code item}
 * can be {@code bytes} (a substring) or an {@code int}, that must be between 0
 * and 255 (otherwise {@code ValueError} is raised). Any other item (e.g. a
 * {@code str}) raises {@code TypeError}.
 */
public class BytesContains extends PyNative {

	protected BytesContains(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "__contains__", params);
	}

	public static BytesContains build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new BytesContains(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		SymbolicExpression self = args[0], item = args[1];
		Satisfiability isInt = hasType(analysis, state, item, INT);
		Satisfiability isBytes = hasType(analysis, state, item, t -> t instanceof PyBytesType);
		BinaryExpression contains = new BinaryExpression(BoolType.INSTANCE, self, item, BytesOperation.CONTAINS, loc);

		AnalysisState<A> result = state.bottom();
		if (isInt != Satisfiability.NOT_SATISFIED) {
			BinaryExpression outOfRange = new BinaryExpression(BoolType.INSTANCE,
					new BinaryExpression(BoolType.INSTANCE, item, new Constant(Int32Type.INSTANCE, 0, loc),
							ComparisonLt.INSTANCE, loc),
					new BinaryExpression(BoolType.INSTANCE, item, new Constant(Int32Type.INSTANCE, 255, loc),
							ComparisonGt.INSTANCE, loc),
					LogicalOr.INSTANCE, loc);
			result = result.lub(ExceptionGuard.guardedCompute(analysis, state, outOfRange,
					LibrarySpecificationProvider.VALUE_ERROR, contains, getCFG(), loc, getOriginatingStatement(),
					this));
		}
		if (isBytes != Satisfiability.NOT_SATISFIED)
			result = result.lub(compute(analysis, state, contains));
		if (isInt != Satisfiability.SATISFIED && isBytes != Satisfiability.SATISFIED)
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}
}
