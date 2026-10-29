package it.unive.pylisa.libraries.svverifier;

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
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.libraries.dicts.DictModel;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.apache.commons.lang3.tuple.Pair;

/**
 * Native implementation of {@code _sv_verifier.nondet_dict(nondet_key,
 * nondet_value)}, that returns a new dictionary with any number of entries,
 * whose keys and values are produced by calling {@code nondet_key()} and
 * {@code nondet_value()} (see {@link NondetList} for the values of the calls).
 * Since the keys are not known, none of them is tracked (see
 * {@link DictModel}): the values are the ones of the untracked keys, and any
 * key might be missing. The dictionary might also be empty, when the functions
 * are never called.
 */
public class NondetDict extends NondetList {

	protected NondetDict(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "nondet_dict", params);
	}

	public static NondetDict build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new NondetDict(cfg, location, exprs);
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemanticsAux(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			ExpressionSet[] params,
			StatementStore<A> expressions)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		ScopeId<A> context = interprocedural.getCurrentContext();
		CodeLocation loc = getLocation();
		Set<Identifier> metas = new HashSet<>();

		// the keys, then the values (as d[nondet_key()] = nondet_value())
		AnalysisState<A> keys = generated(interprocedural, state, getSubExpressions()[0], params[0], expressions,
				metas);
		AnalysisState<A> computed = keys.bottomExecution();
		List<Pair<SymbolicExpression, SymbolicExpression>> entries = new ArrayList<>();
		for (SymbolicExpression key : keys.getExecutionExpressions()) {
			AnalysisState<A> values = generated(interprocedural, keys, getSubExpressions()[1], params[1],
					expressions, metas);
			computed = computed.lub(values);
			for (SymbolicExpression value : values.getExecutionExpressions())
				entries.add(Pair.of(key, value));
		}

		// the dictionary is either empty, or it has the entries computed
		// above, whose keys are not tracked
		DictionaryCreation creation = new DictionaryCreation(getCFG(), new DerivedLocation(loc, "nondet"));
		AnalysisState<A> result = creation.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
				expressions);
		// e.g. the errors raised by the functions
		result = result.lub(keys.bottomExecution()).lub(computed.bottomExecution());
		if (!entries.isEmpty()) {
			AnalysisState<A> created = creation.forwardSemanticsAux(interprocedural, computed, new ExpressionSet[0],
					expressions);
			for (SymbolicExpression dict : created.getExecutionExpressions())
				for (Pair<SymbolicExpression, SymbolicExpression> entry : entries)
					result = result.lub(analysis.smallStepSemantics(
							DictModel.write(analysis, context, created, dict, entry.getLeft(), entry.getRight(), st),
							dict,
							context, this));
		}
		getMetaVariables().addAll(creation.getMetaVariables());
		return result.forgetIdentifiers(metas, this);
	}
}
