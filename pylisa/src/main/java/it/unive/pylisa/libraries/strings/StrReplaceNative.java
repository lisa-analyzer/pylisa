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
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.TernaryExpression;
import it.unive.lisa.symbolic.value.operator.ternary.StringReplace;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.PyNative;
import it.unive.pylisa.symbolic.PyNoneConstant;
import it.unive.pylisa.symbolic.operators.strings.ArgPair;
import it.unive.pylisa.symbolic.operators.strings.StrReplaceCount;
import java.util.function.Predicate;

/**
 * Native implementation of {@code str.replace(self, old, new, count=None)} and
 * of the same method of {@code bytes}, whose {@code old} and {@code new} must
 * be {@code bytes}: {@code old} and {@code new} must be {@code str}s and
 * {@code count} an {@code int}, otherwise {@code TypeError} is raised. A
 * negative or omitted {@code count} replaces all the occurrences.
 */
public class StrReplaceNative extends PyNative {

	protected StrReplaceNative(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "replace", params);
	}

	public static StrReplaceNative build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrReplaceNative(cfg, location, exprs);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> semantics(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression[] args)
			throws SemanticException {
		CodeLocation loc = getLocation();
		AnalysisState<A> result = state.bottom();
		for (boolean bytes : textModes(analysis, state, args[0])) {
			Predicate<Type> text = bytes ? BYTES : STR;
			Type type = bytes ? PyBytesType.INSTANCE : StringType.INSTANCE;
			Satisfiability typed = hasType(analysis, state, args[1], text)
					.and(hasType(analysis, state, args[2], text))
					.and(hasType(analysis, state, args[3], INT.or(NONE)));
			SymbolicExpression value;
			if (!bytes && args[3] instanceof PyNoneConstant)
				value = new TernaryExpression(type, args[0], args[1], args[2], StringReplace.INSTANCE, loc);
			else {
				// bytes always use the python-specific operator
				SymbolicExpression count = args[3] instanceof PyNoneConstant
						? new Constant(Int32Type.INSTANCE, -1, loc)
						: args[3];
				value = new TernaryExpression(type, args[0],
						new BinaryExpression(Untyped.INSTANCE, args[1], args[2], ArgPair.INSTANCE, loc),
						count, StrReplaceCount.INSTANCE, loc);
			}
			result = result.lub(typeChecked(analysis, state, typed, compute(analysis, state, value)));
		}
		return result;
	}
}
