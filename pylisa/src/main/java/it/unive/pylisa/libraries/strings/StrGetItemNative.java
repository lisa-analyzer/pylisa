package it.unive.pylisa.libraries.strings;

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
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.UnaryExpression;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonEq;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonGe;
import it.unive.lisa.symbolic.value.operator.binary.ComparisonLt;
import it.unive.lisa.symbolic.value.operator.binary.LogicalOr;
import it.unive.lisa.symbolic.value.operator.binary.NumericNonOverflowingSub;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.ExceptionGuard;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.operators.SliceCreation;
import it.unive.pylisa.symbolic.operators.strings.StrGetItem;
import it.unive.pylisa.symbolic.operators.strings.StrGetSlice;
import it.unive.pylisa.symbolic.operators.value.StringLength;
import java.util.function.Predicate;

/**
 * Native implementation of {@code str.__getitem__(self, index)}: {@code s[i]}
 * (raising {@code IndexError} if {@code i} is out of range) and
 * {@code s[start:stop:step]} (raising {@code ValueError} if {@code step} is
 * zero). Any other index raises {@code TypeError}.
 */
public class StrGetItemNative extends PyNative {

	protected StrGetItemNative(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "__getitem__", params);
	}

	public static StrGetItemNative build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrGetItemNative(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		SymbolicExpression s = args[0];
		SymbolicExpression index = args[1];
		CodeLocation loc = getLocation();
		Type sliceType = PyClassType.lookup(LibrarySpecificationProvider.SLICE);
		Predicate<Type> isSlice = sliceType::equals;

		boolean literalSlice = index instanceof TernaryExpression
				&& ((TernaryExpression) index).getOperator() == SliceCreation.INSTANCE;
		Satisfiability integer = literalSlice ? Satisfiability.NOT_SATISFIED : hasType(analysis, state, index, INT);
		Satisfiability slice = literalSlice ? Satisfiability.SATISFIED : hasType(analysis, state, index, isSlice);

		AnalysisState<A> result = state.bottom();
		if (integer != Satisfiability.NOT_SATISFIED) {
			// IndexError if index >= len(s) or index < -len(s)
			UnaryExpression len = new UnaryExpression(Int32Type.INSTANCE, s, StringLength.INSTANCE, loc);
			BinaryExpression minusLen = new BinaryExpression(Int32Type.INSTANCE,
					new Constant(Int32Type.INSTANCE, 0, loc), len, NumericNonOverflowingSub.INSTANCE, loc);
			BinaryExpression outOfRange = new BinaryExpression(BoolType.INSTANCE,
					new BinaryExpression(BoolType.INSTANCE, index, len, ComparisonGe.INSTANCE, loc),
					new BinaryExpression(BoolType.INSTANCE, index, minusLen, ComparisonLt.INSTANCE, loc),
					LogicalOr.INSTANCE, loc);
			result = result.lub(ExceptionGuard.guardedCompute(analysis, state, outOfRange,
					LibrarySpecificationProvider.INDEX_ERROR,
					new BinaryExpression(StringType.INSTANCE, s, index, StrGetItem.INSTANCE, loc),
					getCFG(), loc, st, this));
		}

		if (slice != Satisfiability.NOT_SATISFIED) {
			BinaryExpression value = new BinaryExpression(StringType.INSTANCE, s, index, StrGetSlice.INSTANCE, loc);
			if (literalSlice) {
				// bounds and step must be integers or None
				TernaryExpression sl = (TernaryExpression) index;
				Satisfiability typed = Satisfiability.SATISFIED;
				for (SymbolicExpression bound : new SymbolicExpression[] { sl.getLeft(), sl.getMiddle(),
						sl.getRight() })
					typed = typed.and(hasType(analysis, state, bound, INT.or(NONE)));
				BinaryExpression zeroStep = new BinaryExpression(BoolType.INSTANCE, sl.getRight(),
						new Constant(Int32Type.INSTANCE, 0, loc), ComparisonEq.INSTANCE, loc);
				result = result.lub(typeChecked(analysis, state, typed,
						ExceptionGuard.guardedCompute(analysis, state, zeroStep,
								LibrarySpecificationProvider.VALUE_ERROR, value, getCFG(), loc, st, this)));
			} else
				// the step of a slice object is not known
				result = result.lub(compute(analysis, state, value))
						.lub(raise(analysis, state, LibrarySpecificationProvider.VALUE_ERROR));
		}

		if (integer != Satisfiability.SATISFIED && slice != Satisfiability.SATISFIED)
			result = result.lub(raise(analysis, state, LibrarySpecificationProvider.TYPE_ERROR));
		return result;
	}
}
