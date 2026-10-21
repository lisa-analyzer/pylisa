package it.unive.pylisa.program.language.parameterassignment;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.SyntheticLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.NamedParameterExpression;
import it.unive.lisa.program.cfg.statement.literal.StringLiteral;
import it.unive.lisa.program.language.parameterassignment.ParameterAssigningStrategy;
import it.unive.lisa.program.type.Int32Type;
import it.unive.lisa.program.type.StringType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.UnsupportedStatementException;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.Iteration;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.expression.StarExpression;
import it.unive.pylisa.cfg.expression.Unpacking;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import it.unive.pylisa.libraries.lists.ListModel;
import java.util.ArrayList;
import java.util.Arrays;
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

	/**
	 * The arguments of a call collected by the {@code *args} parameter of the
	 * callee, that are stored in a list built before the call.
	 *
	 * @param pos      the position of the parameter
	 * @param elements the arguments
	 * @param values   the values of the arguments
	 */
	private record VarArgs(
			int pos,
			Expression[] elements,
			ExpressionSet[] values) {
	}

	/**
	 * The keyword arguments of a call collected by the {@code **kwargs}
	 * parameter of the callee, that are stored in a dictionary built before the
	 * call.
	 *
	 * @param pos     the position of the parameter
	 * @param entries the names and values of the arguments
	 * @param values  the values of the names and of the arguments
	 */
	private record KwArgs(
			int pos,
			Pair<Expression, Expression>[] entries,
			ExpressionSet[] values) {
	}

	/**
	 * Prepares the call of a native: the arguments are matched to the
	 * parameters as for the other calls (see {@link #prepare}), and the native
	 * is executed in the state where the call happens, as the parameters of a
	 * native are not variables of its own.
	 */
	@Override
	public <A extends AbstractLattice<A>,
			D extends AbstractDomain<A>> Pair<AnalysisState<A>, ExpressionSet[]> prepareForNative(
					Call call,
					AnalysisState<A> callState,
					InterproceduralAnalysis<A, D> interprocedural,
					StatementStore<A> expressions,
					Parameter[] formals,
					ExpressionSet[] parameters)
					throws SemanticException {
		return Pair.of(callState,
				prepare(call, callState, interprocedural, expressions, formals, parameters).getRight());
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Pair<AnalysisState<A>, ExpressionSet[]> prepare(
			Call call,
			AnalysisState<A> callState,
			InterproceduralAnalysis<A, D> interprocedural,
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
				defaults[pos] = callState.getExecutionExpressions();
				Set<Type> types = new HashSet<>();
				for (SymbolicExpression e : defaults[pos])
					types.addAll(interprocedural.getAnalysis().getRuntimeTypesOf(callState, e, call));
				defaultTypes[pos] = types;
			}
		}

		// the starred arguments (*xs): their elements are passed as positional
		// arguments
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type>[] actualTypes = call.parameterTypes(expressions, analysis);
		Starred starred = starred(analysis, callState, call, formals, actuals, parameters, actualTypes);
		actuals = starred.actuals();
		parameters = starred.given();
		actualTypes = starred.types();
		callState = callState.lub(starred.errors());

		VarArgs[] varargs = new VarArgs[1];
		@SuppressWarnings("unchecked")
		KwArgs[] kwargs = new KwArgs[1];
		boolean matches = pythonLogic(
				formals,
				actuals,
				parameters,
				actualTypes,
				defaults,
				defaultTypes,
				slots,
				slotsTypes,
				varargs,
				kwargs,
				interprocedural,
				call.getCFG(),
				callState.bottom());
		if (!matches)
			// the arguments do not match the parameters of the callee: python
			// raises TypeError
			return Pair.of(PyExceptions.raise(interprocedural.getAnalysis(), callState, call.getCFG(),
					call.getLocation(), call, LibrarySpecificationProvider.TYPE_ERROR), parameters);

		Set<Identifier> metas = new HashSet<>();
		if (varargs[0] != null) {
			// the list holding the arguments collected by *args
			ListCreation list = new ListCreation(call.getCFG(), call.getLocation(), varargs[0].elements());
			callState = list.forwardSemanticsAux(interprocedural, callState, varargs[0].values(), expressions);
			if (starred.unknown() != null) {
				// the other elements of a starred argument of unknown length,
				// at unknown indices (there might be none)
				AnalysisState<A> filled = callState;
				Type listType = PyClassType.lookup(LibrarySpecificationProvider.LIST);
				for (SymbolicExpression created : callState.getExecutionExpressions()) {
					AccessChild unknown = new AccessChild(Untyped.INSTANCE,
							new HeapDereference(listType, created, call.getLocation()),
							new Variable(Untyped.INSTANCE, ListCreation.UNKNOWN_INDEX, call.getLocation()),
							call.getLocation());
					for (SymbolicExpression element : starred.unknown())
						filled = filled.lub(analysis.assign(callState, unknown, element, call));
				}
				callState = filled.withExecutionExpressions(callState.getExecutionExpressions());
			}
			slots[varargs[0].pos()] = callState.getExecutionExpressions();
			metas.addAll(list.getMetaVariables());
		}

		if (kwargs[0] != null) {
			// the dictionary holding the arguments collected by **kwargs
			DictionaryCreation dict = new DictionaryCreation(call.getCFG(),
					new DerivedLocation(call.getLocation(), "kwargs"), kwargs[0].entries());
			callState = dict.forwardSemanticsAux(interprocedural, callState, kwargs[0].values(), expressions);
			slots[kwargs[0].pos()] = callState.getExecutionExpressions();
			metas.addAll(dict.getMetaVariables());
		}

		// prepare the state for the call: assign the value to each parameter
		AnalysisState<A> prepared = callState;
		for (int i = 0; i < formals.length; i++) {
			AnalysisState<A> temp = prepared.bottomExecution();
			for (SymbolicExpression exp : slots[i])
				temp = temp.lub(
						interprocedural.getAnalysis().assign(prepared, formals[i].toSymbolicVariable(), exp, call));
			prepared = temp;
		}

		// we remove expressions from the stack
		prepared = prepared.forgetIdentifiers(metas, call).withExecutionExpressions(new ExpressionSet());
		return Pair.of(prepared, slots);
	}

	/**
	 * The arguments of a call, where the starred ones ({@code *xs}) are
	 * replaced by their elements.
	 *
	 * @param actuals the arguments
	 * @param given   their values
	 * @param types   their runtime types
	 * @param unknown the elements of the starred argument of unknown length,
	 *                    that are passed to {@code *args} (if any), or
	 *                    {@code null}
	 * @param errors  the errors raised by passing the starred arguments (e.g.
	 *                    {@code TypeError} if they are not iterable, or if
	 *                    their length might not match the parameters)
	 */
	private record Starred<A extends AbstractLattice<A>>(
			Expression[] actuals,
			ExpressionSet[] given,
			Set<Type>[] types,
			ExpressionSet unknown,
			AnalysisState<A> errors) {
	}

	/**
	 * Replaces the starred arguments with their elements: a list or tuple whose
	 * length is known is replaced by its elements. A starred argument of
	 * unknown length is supported as the last positional argument: any of its
	 * elements is passed to each of the positional parameters that are left,
	 * and the others are collected by {@code *args}; {@code TypeError} might be
	 * raised, as its length might not match them.
	 */
	@SuppressWarnings("unchecked")
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Starred<A> starred(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Call call,
			Parameter[] formals,
			Expression[] actuals,
			ExpressionSet[] given,
			Set<Type>[] types)
			throws SemanticException {
		AnalysisState<A> errors = state.bottomExecution();
		if (Arrays.stream(actuals).noneMatch(StarExpression.class::isInstance))
			return new Starred<>(actuals, given, types, null, errors);

		List<Expression> acts = new ArrayList<>();
		List<ExpressionSet> values = new ArrayList<>();
		List<Set<Type>> valueTypes = new ArrayList<>();
		ExpressionSet unknown = null;
		Expression unknownIterable = null;
		int unknownPos = -1;
		for (int i = 0; i < actuals.length; i++) {
			if (!(actuals[i] instanceof StarExpression)) {
				acts.add(actuals[i]);
				values.add(given[i]);
				valueTypes.add(types[i]);
				continue;
			}
			Expression iterable = ((StarExpression) actuals[i]).getSubExpression();
			List<ExpressionSet> elements = exactElements(analysis, state, given[i], call);
			if (elements != null) {
				for (ExpressionSet element : elements) {
					acts.add(iterable);
					values.add(element);
					valueTypes.add(typesOf(analysis, state, element, call));
				}
				continue;
			}
			for (int j = i + 1; j < actuals.length; j++)
				if (!(actuals[j] instanceof NamedParameterExpression))
					throw new UnsupportedStatementException(
							"starred arguments of unknown length are supported only as the last positional ones");
			AnalysisState<A> iterated = state.bottomExecution();
			for (SymbolicExpression value : given[i])
				iterated = iterated.lub(Iteration.elements(analysis, state, iterable, value, call, call.getMetaVariables()));
			errors = errors.lub(iterated.bottomExecution());
			unknown = iterated.getExecutionExpressions();
			unknownIterable = iterable;
			unknownPos = acts.size();
		}

		if (unknown != null) {
			// any element is passed to each positional parameter that is left
			// (not passed by name), the other ones are collected by *args
			Set<String> named = new HashSet<>();
			for (Expression actual : actuals)
				if (actual instanceof NamedParameterExpression)
					named.add(((NamedParameterExpression) actual).getParameterName());
			boolean varargs = Arrays.stream(formals).anyMatch(VarPositionalParameter.class::isInstance);
			int filled = 0;
			Set<Type> unknownTypes = typesOf(analysis, state, unknown, call);
			for (int f = unknownPos; f < formals.length && !(formals[f] instanceof VarPositionalParameter)
					&& !(formals[f] instanceof VarKeywordParameter) && !named.contains(formals[f].getName()); f++) {
				acts.add(unknownPos + filled, unknownIterable);
				values.add(unknownPos + filled, unknown);
				valueTypes.add(unknownPos + filled, unknownTypes);
				filled++;
			}
			if (filled > 0 || !varargs)
				// the length might not match the parameters
				errors = errors.lub(PyExceptions.raise(analysis, state, call.getCFG(), call.getLocation(), call,
						LibrarySpecificationProvider.TYPE_ERROR));
			if (!varargs)
				unknown = null;
		}
		return new Starred<>(acts.toArray(Expression[]::new), values.toArray(ExpressionSet[]::new),
				valueTypes.toArray(Set[]::new), unknown, errors);
	}

	// the elements of the list or tuple, if its length is known
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> List<ExpressionSet> exactElements(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			ExpressionSet values,
			Call call)
			throws SemanticException {
		if (values.size() != 1)
			return null;
		SymbolicExpression value = values.iterator().next();
		Set<Type> types = analysis.getRuntimeTypesOf(state, value, call);
		if (types.size() != 1)
			return null;
		Type type = types.iterator().next();
		if (!ListModel.isList(type) && !Unpacking.isTuple(type))
			return null;
		Integer length = ListModel.shape(analysis, state, value, call).length();
		if (length == null)
			return null;
		CodeLocation loc = call.getLocation();
		HeapDereference deref = new HeapDereference(type.asPointerType().getInnerType(), value, loc);
		List<ExpressionSet> elements = new ArrayList<>();
		for (int i = 0; i < length; i++)
			elements.add(analysis.smallStepSemantics(state,
					new AccessChild(Untyped.INSTANCE, deref, new Constant(Int32Type.INSTANCE, i, loc), loc), call)
					.getExecutionExpressions());
		return elements;
	}

	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Set<Type> typesOf(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			ExpressionSet values,
			Call call)
			throws SemanticException {
		Set<Type> types = new HashSet<>();
		for (SymbolicExpression value : values)
			types.addAll(analysis.getRuntimeTypesOf(state, value, call));
		return types;
	}

	// fills the slots of the formal parameters with the actual ones, returning
	// whether the actual parameters match the formal ones
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> boolean pythonLogic(
			Parameter[] formals,
			Expression[] actuals,
			ExpressionSet[] given,
			Set<Type>[] givenTypes,
			ExpressionSet[] defaults,
			Set<Type>[] defaultTypes,
			ExpressionSet[] slots,
			Set<Type>[] slotTypes,
			VarArgs[] varargs,
			KwArgs[] kwargs,
			InterproceduralAnalysis<A, D> interprocedural,
			CFG callCFG,
			AnalysisState<A> bottom)
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
				return false;
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
				return false;
			}

			int offset = aPos - vargsList.size();
			// create the expressions set
			ExpressionSet[] symbolicExprs = new ExpressionSet[aPos - offset];
			for (int i = 0; i < aPos - offset; i++) {
				symbolicExprs[i] = given[offset + i];
			}
			// the list is built before the call (see prepare)
			varargs[0] = new VarArgs(fPos, vargsList.toArray(Expression[]::new), symbolicExprs);
			slots[fPos] = new ExpressionSet();
			slotTypes[fPos] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.LIST));
			fPos++;
		}

		if (aPos < namedParOffset)
			// more positional arguments than positional parameters
			return false;

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

			// the dictionary is built before the call (see prepare)
			kwargs[0] = new KwArgs(formals.length - 1, pairExprs.toArray(Pair[]::new),
					symbExprs.toArray(ExpressionSet[]::new));
			slots[formals.length - 1] = new ExpressionSet();
			slotTypes[formals.length - 1] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.DICT));
		}

		// fourth phase: keyword arguments
		boolean varKeyword = formals.length > 0 && formals[formals.length - 1] instanceof VarKeywordParameter;
		for (; aPos < actuals.length; aPos++) {
			String name = ((NamedParameterExpression) actuals[aPos]).getParameterName();
			boolean found = false;
			for (int i = 0; i < formals.length; i++)
				if (!(formals[i] instanceof VarPositionalParameter) && !(formals[i] instanceof VarKeywordParameter)
						&& formals[i].getName().equals(name)) {
					found = true;
					if (slots[i] != null)
						// already filled -> TypeError
						return false;
					else {
						slots[i] = given[aPos];
						slotTypes[i] = givenTypes[aPos];
					}
					break;
				}
			if (!found && !varKeyword)
				// no parameter with that name -> TypeError
				return false;
		}

		// fifth phase: default values
		for (int pos = 0; pos < slots.length; pos++)
			if (slots[pos] == null) {
				if (formals[pos] instanceof VarPositionalParameter) {
					// no positional argument is left for *args: it is empty
					varargs[0] = new VarArgs(pos, new Expression[0], new ExpressionSet[0]);
					slots[pos] = new ExpressionSet();
					slotTypes[pos] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.LIST));
				} else if (defaults[pos] == null)
					// unfilled and no default value -> TypeError
					return false;
				else {
					slots[pos] = defaults[pos];
					slotTypes[pos] = defaultTypes[pos];
				}
			}

		return true;
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
