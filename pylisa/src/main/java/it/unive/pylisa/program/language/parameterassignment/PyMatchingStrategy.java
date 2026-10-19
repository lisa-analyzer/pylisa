package it.unive.pylisa.program.language.parameterassignment;

import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.language.resolution.FixedOrderMatchingStrategy;
import it.unive.lisa.program.language.resolution.ParameterMatchingStrategy;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.apache.commons.lang3.tuple.Pair;

public class PyMatchingStrategy implements ParameterMatchingStrategy {
	private final FixedOrderMatchingStrategy delegate;

	/**
	 * Builds the strategy.
	 *
	 * @param delegate the strategy to delegate the match after the actual
	 *                     parameters have been shuffled
	 */
	public PyMatchingStrategy(
			FixedOrderMatchingStrategy delegate) {
		this.delegate = delegate;
	}

	@Override
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

		Boolean logic = PyMatchingStrategy.pythonLogic(
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
			// the arguments do not match the parameters: if the callee is
			// defined in the program (i.e., it is not a native whose semantics
			// rely on the number of its arguments), it is still a target of
			// the call, that raises TypeError (see PyAssigningStrategy)
			return formals.length > 0 && Arrays.stream(formals).allMatch(PyParameter.class::isInstance);

		return delegate.matches(call, formals, slots, slotTypes);
	}

	@Override
	public int distanceFromPerfectTarget(
			UnresolvedCall call,
			Set<Type>[] types,
			CodeMember cm,
			boolean instance) {
		if (call.getParameters().length > cm.getDescriptor().getFormals().length)
			// a callee whose parameters do not match the arguments (see
			// matches): python has no overloading, so the distance only ranks
			// equivalent targets
			return 0;
		return ParameterMatchingStrategy.super.distanceFromPerfectTarget(call, types, cm, instance);
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

			ListCreation listCreation = new ListCreation(callCFG, SyntheticLocation.INSTANCE,
					vargsList.toArray(Expression[]::new));
			if (fPos >= slotTypes.length)
				// no more space!
				return failure;

			slots[fPos] = (T) listCreation;
			slotTypes[fPos] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.LIST));
			fPos++;
		}

		if (aPos < namedParOffset)
			// more positional arguments than positional parameters
			return failure;

		// third phase: kwargs
		// kwargs must be the last parameter.
		if (formals.length > 0 && formals[formals.length - 1] instanceof VarKeywordParameter) {
			// for every named parameter, if it is NOT in the formal named
			// parameter list, then add it to the varkeyword.
			// 1. prepare dict.
			List<Pair<Expression, Expression>> pairExprs = new ArrayList<>();
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
					Expression right = ((NamedParameterExpression) actuals[i]).getSubExpression();
					Expression left = new StringLiteral(callCFG, SyntheticLocation.INSTANCE,
							((NamedParameterExpression) actuals[i]).getParameterName());
					pairExprs.add(Pair.of(left, right));
					namedPars.remove(((NamedParameterExpression) actuals[i]).getParameterName());
				}
			}

			DictionaryCreation dictCreation = new DictionaryCreation(callCFG, SyntheticLocation.INSTANCE,
					pairExprs.toArray(Pair[]::new));
			slots[formals.length - 1] = (T) dictCreation;
			slotTypes[formals.length - 1] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.DICT));
		}

		// fourth phase: keyword arguments
		for (; aPos < actuals.length; aPos++) {
			if (!(actuals[aPos] instanceof NamedParameterExpression))
				return failure;
			String name = ((NamedParameterExpression) actuals[aPos]).getParameterName();
			boolean found = false;
			for (int i = 0; i < formals.length; i++)
				if (!(formals[i] instanceof VarPositionalParameter) && !(formals[i] instanceof VarKeywordParameter)
						&& formals[i].getName().equals(name)) {
					found = true;
					if (slots[i] != null)
						// already filled -> TypeError
						return failure;
					else {
						slots[i] = given[aPos];
						slotTypes[i] = givenTypes[aPos];
					}
					break;
				}
			if (!found && !(formals.length > 0 && formals[formals.length - 1] instanceof VarKeywordParameter))
				// no parameter with that name -> TypeError
				return failure;
		}

		// fifth phase: default values
		for (int pos = 0; pos < slots.length; pos++)
			if (slots[pos] == null) {
				if (formals[pos] instanceof VarPositionalParameter) {
					// no positional argument is left for *args: it is empty
					slots[pos] = (T) new ListCreation(callCFG, SyntheticLocation.INSTANCE);
					slotTypes[pos] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.LIST));
				} else if (defaults[pos] == null)
					// unfilled and no default value -> TypeError
					return failure;
				else {
					slots[pos] = defaults[pos];
					slotTypes[pos] = defaultTypes[pos];
				}
			}

		return null;
	}

	private static int getNamedParIndex(
			Expression[] expressions) {
		for (int i = 0; i < expressions.length; i++)
			if (expressions[i] instanceof NamedParameterExpression)
				return i;
		return -1;
	}
}
