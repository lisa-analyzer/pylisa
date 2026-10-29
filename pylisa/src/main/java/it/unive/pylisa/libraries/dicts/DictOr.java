package it.unive.pylisa.libraries.dicts;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.BinaryExpression;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.PluggableStatement;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

/**
 * Native implementation of {@code dict.__or__(self, other)}, that is,
 * {@code self | other} for two dictionaries: a new dictionary with the entries
 * of {@code self}, overridden by the ones of {@code other}, as {@code {**self,
 * **other}} (see {@link DictModel#unpackInto}).
 */
public class DictOr extends BinaryExpression implements PluggableStatement {

	protected Statement st;

	protected DictOr(
			CFG cfg,
			CodeLocation location,
			String constructName,
			Expression self,
			Expression other) {
		super(cfg, location, constructName, self, other);
	}

	public static DictOr build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new DictOr(cfg, location, "__or__", exprs[0], exprs[1]);
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
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = getLocation();
		if (LibrarySpecificationProvider.isLibraryLoaded(LibrarySpecificationProvider.PANDAS))
			// dictionaries are symbolic values (see DictionaryCreation)
			return analysis.smallStepSemantics(state, new PushAny(Untyped.INSTANCE, loc), context, this);

		DictionaryCreation creation = new DictionaryCreation(getCFG(), new DerivedLocation(loc, "or"));
		AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				expressions);
		getMetaVariables().addAll(creation.getMetaVariables());
		AnalysisState<A> result = created.bottomExecution();
		for (SymbolicExpression dict : created.getExecutionExpressions()) {
			AnalysisState<A> merged = DictModel.unpackInto(analysis, context, created, dict, left, st, true);
			merged = DictModel.unpackInto(analysis, context, merged, dict, right, st, true);
			result = result.lub(analysis.smallStepSemantics(merged, dict, context, this));
		}
		return result;
	}
}
