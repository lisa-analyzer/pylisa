package it.unive.pylisa.libraries.strings;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.symbolic.operators.value.StringFormat;
import it.unive.pylisa.symbolic.operators.value.StringFormatRaises;

/**
 * Native implementation of {@code str.__mod__(self, other)} and
 * {@code bytes.__mod__(self, other)}: printf-style formatting of {@code self}
 * with {@code other} ({@code "%s" % x}, {@code b"%d" % 5}). It raises
 * {@code TypeError} or {@code ValueError} when Python does (e.g.
 * {@code "ab" % 7}, {@code "%q" % 7}, or {@code b"%s" % "x"}), as decided by
 * the domains through {@link StringFormatRaises}. There is no {@code __rmod__}:
 * formatting is always driven by the left-hand format.
 */
public class StrMod extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected StrMod(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static StrMod build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new StrMod(cfg, location, "__mod__", exprs[0], exprs[1]);
	}

	@Override
	final public void setOriginatingStatement(
			Statement st) {
		this.st = st;
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
		CodeLocation loc = getLocation();

		// whether the formatting raises is decided by the domains that can
		// evaluate it, while all others answer that it might
		it.unive.lisa.symbolic.value.BinaryExpression typeError = new it.unive.lisa.symbolic.value.BinaryExpression(
				BoolType.INSTANCE, left, right, StringFormatRaises.TYPE_ERROR, loc);
		it.unive.lisa.symbolic.value.BinaryExpression valueError = new it.unive.lisa.symbolic.value.BinaryExpression(
				BoolType.INSTANCE, left, right, StringFormatRaises.VALUE_ERROR, loc);
		Satisfiability raisesTypeError = analysis.satisfies(state, typeError, this);
		Satisfiability raisesValueError = analysis.satisfies(state, valueError, this);

		// bytes formats produce bytes
		Type type = analysis.getRuntimeTypesOf(state, left, this).stream()
				.anyMatch(t -> t instanceof PyBytesType) ? PyBytesType.INSTANCE : StringType.INSTANCE;
		AnalysisState<A> result = state.bottom();
		if (raisesTypeError != Satisfiability.SATISFIED && raisesValueError != Satisfiability.SATISFIED)
			result = result.lub(analysis.smallStepSemantics(state,
					new it.unive.lisa.symbolic.value.BinaryExpression(type, left, right, StringFormat.INSTANCE, loc),
					st));
		if (raisesTypeError != Satisfiability.NOT_SATISFIED)
			result = result.lub(PyExceptions.raise(analysis, state, getCFG(), loc, this,
					LibrarySpecificationProvider.TYPE_ERROR));
		if (raisesValueError != Satisfiability.NOT_SATISFIED)
			result = result.lub(PyExceptions.raise(analysis, state, getCFG(), loc, this,
					LibrarySpecificationProvider.VALUE_ERROR));
		return result;
	}
}
