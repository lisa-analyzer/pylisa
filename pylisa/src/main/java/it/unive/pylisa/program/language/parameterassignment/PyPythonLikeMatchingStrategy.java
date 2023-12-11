package it.unive.pylisa.program.language.parameterassignment;

import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.language.resolution.FixedOrderMatchingStrategy;
import it.unive.lisa.program.language.resolution.ParameterMatchingStrategy;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.*;
import org.apache.commons.lang3.tuple.Pair;

public class PyPythonLikeMatchingStrategy implements ParameterMatchingStrategy {
	private final FixedOrderMatchingStrategy delegate;

	/**
	 * Builds the strategy.
	 *
	 * @param delegate the strategy to delegate the match after the actual
	 *                     parameters have been shuffled
	 */
	public PyPythonLikeMatchingStrategy(
			FixedOrderMatchingStrategy delegate) {
		this.delegate = delegate;
	}

	@Override
	@SuppressWarnings("unchecked")
	public boolean matches(
			Call call,
			Parameter[] formals,
			Expression[] actuals,
			Set<Type>[] types) {
		Expression[] slots = new Expression[formals.length];
		Set<Type>[] slotTypes = new Set[formals.length];

		Expression[] defaults = new Expression[formals.length];
		Set<Type>[] defaultTypes = new Set[formals.length];
		for (int pos = 0; pos < slots.length; pos++) {
			Expression def = formals[pos].getDefaultValue();
			if (def != null) {
				defaults[pos] = def;
				defaultTypes[pos] = def.getStaticType().allInstances(call.getProgram().getTypes());
			}
		}

		Boolean logic = PyPythonLikeMatchingStrategy.pythonLogic(
				formals,
				actuals,
				actuals,
				types,
				defaults,
				defaultTypes,
				slots,
				slotTypes,
				call.getCFG(),
				false);
		if (logic != null)
			return logic;

		return delegate.matches(call, formals, slots, slotTypes);
	}

	public static <T, F> F pythonLogic(
			Parameter[] formals,
			Expression[] actuals,
			T[] given,
			Set<Type>[] givenTypes,
			T[] defaults,
			Set<Type>[] defaultTypes,
			T[] slots,
			Set<Type>[] slotTypes,
			CFG callCFG,
			F failure) {

		Set<String> namedParameterExpressions = new HashSet<>();
		int namedParameterOffset = getNamedParameterExpressionIndex(actuals);
		if (namedParameterOffset >= 0) {
			for (int i = namedParameterOffset; i < actuals.length; i++) {
				namedParameterExpressions.add(((NamedParameterExpression) actuals[i]).getParameterName());
			}
		} else {
			namedParameterOffset = actuals.length;
		}

		int actualPos = 0;
		int formalsPos = 0;
		boolean vargsPos = false;
		boolean vargsKw = false;

		// first phase: positional arguments
		for (; actualPos < namedParameterOffset && formalsPos < formals.length; actualPos++, formalsPos++) {
			if (formals[formalsPos] instanceof VarKeywordParameter) {
				// problem: varKeywordParameter in positional parameter
				return failure;
			} else if (formals[formalsPos] instanceof VarPositionalParameter) {
				// all the next positional parameter must be inserted inside a
				// list.
				break;
			} else {
				slots[formalsPos] = given[actualPos];
				slotTypes[formalsPos] = givenTypes[actualPos];
			}
		}
		// second phase: check vargsPos
		if (formalsPos < formals.length && formals[formalsPos] instanceof VarPositionalParameter) {
			List<Expression> vargsList = new ArrayList<>();
			for (; actualPos < namedParameterOffset; actualPos++) {
				// stop if actuals[pos] == NamedParameterExpression
				if (actuals[actualPos] instanceof NamedParameterExpression)
					break;
				vargsList.add(actuals[actualPos]);
			}
			ListCreation listCreation = new ListCreation(callCFG, SyntheticLocation.INSTANCE,
					vargsList.toArray(Expression[]::new));
			if (formalsPos >= slotTypes.length) {
				// no more spaces!
				return failure;
			}

			slots[formalsPos] = (T) listCreation;
			slotTypes[formalsPos] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.LIST));
			formalsPos++;
		}

		// third phase: kwargs
		// kwargs must be the last parameter.
		if (formals.length > 0 && formals[formals.length - 1] instanceof VarKeywordParameter) {
			// for every named parameter, if it is NOT in the formal named
			// parameter list, then add it to the varkeyword.
			// 1. prepare dict.
			List<Pair<Expression, Expression>> pairExprs = new ArrayList<>();
			for (int i = actualPos; i < actuals.length; i++) {
				boolean found = false;
				String name = ((NamedParameterExpression) actuals[i]).getParameterName(); // ACTUAL
																							// VAR.
																							// NAME
				for (int j = formalsPos; j < formals.length; j++) {
					if (formals[j].getName().equals(name)) {
						found = true;
						break;
					}
				}
				if (!found) {
					Expression right = ((NamedParameterExpression) actuals[i]).getSubExpression();
					Expression left = new StringLiteral(callCFG, SyntheticLocation.INSTANCE,
							((NamedParameterExpression) actuals[i]).getParameterName());
					pairExprs.add(Pair.of(left, right));
					namedParameterExpressions.remove(((NamedParameterExpression) actuals[i]).getParameterName());
				}
			}

			DictionaryCreation dictCreation = new DictionaryCreation(callCFG, SyntheticLocation.INSTANCE,
					pairExprs.toArray(Pair[]::new));
			slots[formals.length - 1] = (T) dictCreation;
			slotTypes[formals.length - 1] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.DICT));
		}

		// fourth phase: keyword arguments
		for (; actualPos < actuals.length; actualPos++) {
			if (!(actuals[actualPos] instanceof NamedParameterExpression)) {
				return failure;
			}
			String name = ((NamedParameterExpression) actuals[actualPos]).getParameterName();
			for (int i = formalsPos; i < formals.length; i++)
				if (formals[i].getName().equals(name)) {
					if (slots[i] != null)
						// already filled -> TypeError
						return failure;
					else {
						slots[i] = given[actualPos];
						slotTypes[i] = givenTypes[actualPos];
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

	static int getNamedParameterExpressionIndex(
			Expression[] expressions) {
		for (int i = 0; i < expressions.length; i++) {
			if (expressions[i] instanceof NamedParameterExpression) {
				return i;
			}
		}
		return -1;
	}
}
