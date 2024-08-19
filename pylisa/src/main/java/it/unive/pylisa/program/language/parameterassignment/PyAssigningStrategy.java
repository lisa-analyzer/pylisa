package it.unive.pylisa.program.language.parameterassignment;

import it.unive.lisa.analysis.AbstractState;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.language.parameterassignment.ParameterAssigningStrategy;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.apache.commons.lang3.tuple.Pair;

public class PyAssigningStrategy implements ParameterAssigningStrategy {

	/**
	 * The singleton instance of this class.
	 */
	public static final PyAssigningStrategy INSTANCE = new PyAssigningStrategy();

	private PyAssigningStrategy() {
	}

	@Override
	@SuppressWarnings("unchecked")
	public <A extends AbstractState<A>> Pair<AnalysisState<A>, ExpressionSet[]> prepare(
			Call call,
			AnalysisState<A> callState,
			InterproceduralAnalysis<A> interprocedural,
			StatementStore<A> expressions,
			Parameter[] formals,
			ExpressionSet[] parameters)
			throws SemanticException {

		ExpressionSet[] slots = new ExpressionSet[formals.length];
		Set<Type>[] slotsTypes = new Set[formals.length];
		Expression[] actuals = call.getParameters();

		ExpressionSet[] defaults = new ExpressionSet[formals.length];
		Set<Type>[] defaultTypes = new Set[formals.length];
		for (int pos = 0; pos < slots.length; pos++) {
			Expression def = formals[pos].getDefaultValue();
			if (def != null) {
				callState = def.forwardSemantics(callState, interprocedural, expressions);
				expressions.put(def, callState);
				defaults[pos] = callState.getComputedExpressions();
				Set<Type> types = new HashSet<>();
				for (SymbolicExpression e : defaults[pos])
					types.addAll(callState.getState().getRuntimeTypesOf(e, call, callState.getState()));
				defaultTypes[pos] = types;
			}
		}

		AnalysisState<A> logic = pythonLogic(
				formals,
				actuals,
				parameters,
				call.parameterTypes(expressions),
				defaults,
				defaultTypes,
				slots,
				slotsTypes,
				interprocedural,
				call.getCFG(),
				callState.bottom());
		if (logic != null)
			return Pair.of(logic, parameters);

		// prepare the state for the call: assign the value to each parameter
		AnalysisState<A> prepared = callState;
		for (int i = 0; i < formals.length; i++) {
			AnalysisState<A> temp = prepared.bottom();
			for (SymbolicExpression exp : slots[i])
				temp = temp.lub(prepared.assign(formals[i].toSymbolicVariable(), exp, call));
			prepared = temp;
		}

		// we remove expressions from the stack
		prepared = new AnalysisState<>(prepared.getState(), new ExpressionSet(), prepared.getFixpointInformation());
		return Pair.of(prepared, slots);
	}

	@SuppressWarnings("unchecked")
	private <A extends AbstractState<A>> AnalysisState<A> pythonLogic(
			Parameter[] formals,
			Expression[] actuals,
			ExpressionSet[] given,
			Set<Type>[] givenTypes,
			ExpressionSet[] defaults,
			Set<Type>[] defaultTypes,
			ExpressionSet[] slots,
			Set<Type>[] slotTypes,
			InterproceduralAnalysis<A> interprocedural,
			CFG callCFG,
			AnalysisState<A> failure)
			throws SemanticException {
		Set<String> namedPars = new HashSet<>();
		int namedParOffset = getNamedParIndex(actuals);
		if (namedParOffset >= 0)
			for (int i = namedParOffset; i < actuals.length; i++)
				namedPars.add(((NamedParameterExpression) actuals[i]).getParameterName());
		else
			namedParOffset = actuals.length;

		int aPos = 0;
		int fPos = 0;

		// first phase: positional arguments
		for (; aPos < namedParOffset && fPos < formals.length; aPos++, fPos++) {
			if (formals[fPos] instanceof VarKeywordParameter)
				// problem: varKeywordParameter in positional parameter
				return failure;
			else if (formals[fPos] instanceof VarPositionalParameter)
				// all the next positional parameter must be inserted inside a
				// list
				break;
			else {
				slots[fPos] = given[aPos];
				slotTypes[fPos] = givenTypes[aPos];
			}
		}

		// second phase: check vargsPos
		if (fPos < formals.length && formals[fPos] instanceof VarPositionalParameter) {
			List<Expression> vargsList = new ArrayList<>();
			for (; aPos < namedParOffset; aPos++) {
				// stop if actuals[pos] == NamedParameterExpression
				if (actuals[aPos] instanceof NamedParameterExpression)
					break;
				vargsList.add(actuals[aPos]);
			}

			if (fPos >= slotTypes.length) {
				// no more space!
				return failure;
			}

			int offset = aPos - vargsList.size();
			// create the expressions set
			ExpressionSet[] symbolicExprs = new ExpressionSet[aPos - offset];
			for (int i = 0; i < aPos - offset; i++) {
				symbolicExprs[i] = given[offset + i];
			}
			ListCreation listCreation = new ListCreation(callCFG, SyntheticLocation.INSTANCE,
					vargsList.toArray(Expression[]::new));
			AnalysisState<A> listSemantics = listCreation.forwardSemanticsAux(interprocedural,
					failure.bottom(), symbolicExprs, null);
			slots[fPos] = listSemantics.getComputedExpressions();
			slotTypes[fPos] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.LIST));
			fPos++;
		}

		// third phase: kwargs
		// kwargs must be the last parameter.
		if (formals.length > 0 && formals[formals.length - 1] instanceof VarKeywordParameter) {
			// for every named parameter, if it is NOT in the formal named
			// parameter list, then add it to the varkeyword.
			// 1. prepare dict.
			List<Pair<Expression, Expression>> pairExprs = new ArrayList<>();
			List<ExpressionSet> symbExprs = new ArrayList<>();
			for (int i = aPos; i < actuals.length; i++) {
				boolean found = false;
				// ACTUAL VAR. NAME
				String name = ((NamedParameterExpression) actuals[i]).getParameterName();
				for (int j = fPos; j < formals.length; j++)
					if (formals[j].getName().equals(name)) {
						found = true;
						break;
					}

				if (!found) {
					ExpressionSet left = new ExpressionSet(new Constant(StringType.INSTANCE,
							((NamedParameterExpression) actuals[i]).getParameterName(), SyntheticLocation.INSTANCE));
					ExpressionSet right = given[i];
					symbExprs.add(left);
					symbExprs.add(right);
					Expression _right = ((NamedParameterExpression) actuals[i]).getSubExpression();
					Expression _left = new StringLiteral(callCFG, SyntheticLocation.INSTANCE,
							((NamedParameterExpression) actuals[i]).getParameterName());
					pairExprs.add(Pair.of(_left, _right));
					namedPars.remove(((NamedParameterExpression) actuals[i]).getParameterName());
				}
			}

			DictionaryCreation dictCreation = new DictionaryCreation(callCFG, SyntheticLocation.INSTANCE,
					pairExprs.toArray(Pair[]::new));
			AnalysisState<A> dictSemantics = dictCreation.forwardSemanticsAux(interprocedural,
					failure.bottom(), symbExprs.toArray(ExpressionSet[]::new), null);
			slots[formals.length - 1] = dictSemantics.getComputedExpressions();
			slotTypes[formals.length - 1] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.DICT));
		}

		// fourth phase: keyword arguments
		for (; aPos < actuals.length; aPos++) {
			String name = ((NamedParameterExpression) actuals[aPos]).getParameterName();
			for (int i = fPos; i < formals.length; i++)
				if (formals[i].getName().equals(name)) {
					if (slots[i] != null)
						// already filled -> TypeError
						return failure;
					else {
						slots[i] = given[aPos];
						slotTypes[i] = givenTypes[aPos];
					}
					break;
				}
		}

		// fifth phase: default values
		for (int pos = 0; pos < slots.length; pos++)
			if (slots[pos] == null) {
				if (defaults[pos] == null)
					// unfilled and no default value -> TypeError
					return failure;
				else {
					slots[pos] = defaults[pos];
					slotTypes[pos] = defaultTypes[pos];
				}
			}

		return null;
	}

	static int getNamedParIndex(
			Expression[] expressions) {
		for (int i = 0; i < expressions.length; i++) {
			if (expressions[i] instanceof NamedParameterExpression) {
				return i;
			}
		}
		return -1;
	}
}
