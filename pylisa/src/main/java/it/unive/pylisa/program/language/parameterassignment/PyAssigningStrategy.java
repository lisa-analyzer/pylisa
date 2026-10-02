package it.unive.pylisa.program.language.parameterassignment;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
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
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.VarKeywordParameter;
import it.unive.pylisa.cfg.VarPositionalParameter;
import it.unive.pylisa.cfg.expression.DictionaryCreation;
import it.unive.pylisa.cfg.expression.ListCreation;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyExceptionType;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.program.PySyntheticLocation;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class PyAssigningStrategy implements ParameterAssigningStrategy {

	private static final Logger LOG = LogManager.getLogger(PyAssigningStrategy.class);

	/**
	 * The singleton instance of this class.
	 */
	public static final PyAssigningStrategy INSTANCE = new PyAssigningStrategy();

	private PyAssigningStrategy() {
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
				defaults[pos] = callState.getExecution().getComputedExpressions();
				Set<Type> types = new HashSet<>();
				for (SymbolicExpression e : defaults[pos])
					types.addAll(interprocedural.getAnalysis().getRuntimeTypesOf(callState, e, call));
				defaultTypes[pos] = types;
			}
		}

		AnalysisState<A> logic = pythonLogic(
				formals,
				actuals,
				parameters,
				call.parameterTypes(expressions, interprocedural.getAnalysis()),
				defaults,
				defaultTypes,
				slots,
				slotsTypes,
				interprocedural,
				call.getCFG(),
				callState.bottom());
		if (logic != null) {
			if (isSimplePositionalCall(actuals, formals, parameters)) {
				LOG.warn("The arguments of {} at {} do not match the parameters by Python's rules, where Python "
						+ "raises TypeError: they are bound by position", call, call.getLocation());
				AnalysisState<A> prepared = callState;
				for (int i = 0; i < formals.length; i++) {
					AnalysisState<A> temp = prepared.bottom();
					for (SymbolicExpression exp : parameters[i])
						temp = temp.lub(interprocedural.getAnalysis().assign(
								prepared,
								formals[i].toSymbolicVariable(),
								exp,
								call));
					prepared = temp;
				}
				prepared = prepared.withExecutionExpressions(new ExpressionSet());
				return Pair.of(orTypeError(prepared, callState, call, interprocedural), parameters);
			}

			// Keep analysis soundly conservative: parameter matching failures
			// should not collapse execution to bottom.
			LOG.warn("The arguments {} of {} at {} do not match the parameters {} of the callee, where Python "
					+ "raises TypeError: every parameter is unknown", Arrays.toString(actuals), call,
					call.getLocation(), Arrays.toString(formals));
			AnalysisState<A> prepared = callState;
			ExpressionSet[] unknown = new ExpressionSet[formals.length];
			for (int i = 0; i < formals.length; i++) {
				PushAny any = new PushAny(Untyped.INSTANCE, PySyntheticLocation.INSTANCE);
				prepared = interprocedural.getAnalysis().assign(prepared, formals[i].toSymbolicVariable(), any, call);
				unknown[i] = new ExpressionSet(any);
			}
			prepared = prepared.withExecutionExpressions(new ExpressionSet());
			// one unknown value per parameter, as a native callee reads them
			return Pair.of(orTypeError(prepared, callState, call, interprocedural), unknown);
		}

		// prepare the state for the call: assign the value to each parameter
		AnalysisState<A> prepared = callState;
		for (int i = 0; i < formals.length; i++) {
			AnalysisState<A> temp = prepared.bottom();
			for (SymbolicExpression exp : slots[i])
				temp = temp.lub(
						interprocedural.getAnalysis().assign(prepared, formals[i].toSymbolicVariable(), exp, call));
			prepared = temp;
		}

		// we remove expressions from the stack
		// prepared = new AnalysisState<>(prepared, new ExpressionSet(),
		// prepared.getExecution().getFixpointInformation());
		prepared = prepared.withExecutionExpressions(new ExpressionSet());
		return Pair.of(prepared, slots);
	}

	/**
	 * Adds to the state prepared for a call whose arguments do not match the
	 * callee's parameters the executions where Python raises
	 * {@code TypeError} at the call. The prepared state stays: the mismatch
	 * may be the analysis's own, when it passes the receiver of a method call
	 * where Python does not, or does not where Python does.
	 */
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> orTypeError(
			AnalysisState<A> prepared,
			AnalysisState<A> callState,
			Call call,
			InterproceduralAnalysis<A, D> interprocedural)
			throws SemanticException {
		return prepared.lub(interprocedural.getAnalysis().moveExecutionToError(callState,
				new AnalysisState.Error(PyExceptionType.TYPE_ERROR, call), call));
	}

	private boolean isSimplePositionalCall(
			Expression[] actuals,
			Parameter[] formals,
			ExpressionSet[] parameters) {
		if (actuals.length != formals.length || parameters.length != formals.length)
			return false;

		for (Expression actual : actuals)
			if (actual instanceof NamedParameterExpression)
				return false;

		for (Parameter formal : formals)
			if (formal instanceof VarKeywordParameter || formal instanceof VarPositionalParameter)
				return false;

		return true;
	}

	/**
	 * Fills the slot of each formal parameter with the values of the arguments
	 * {@link ArgumentBinding} binds to it: a list of them for a {@code *args}
	 * parameter, a dictionary of them by name for a {@code **kw} parameter,
	 * the default value for a parameter with no argument.
	 *
	 * @return {@code null} when the slots are filled, {@code failure} when
	 *             the arguments do not match the parameters
	 */
	@SuppressWarnings({ "unchecked", "rawtypes" })
	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> pythonLogic(
			Parameter[] formals,
			Expression[] actuals,
			ExpressionSet[] given,
			Set<Type>[] givenTypes,
			ExpressionSet[] defaults,
			Set<Type>[] defaultTypes,
			ExpressionSet[] slots,
			Set<Type>[] slotTypes,
			InterproceduralAnalysis<A, D> interprocedural,
			CFG callCFG,
			AnalysisState<A> failure)
			throws SemanticException {
		Optional<List<List<Integer>>> binding = ArgumentBinding.bind(formals, actuals);
		if (binding.isEmpty())
			return failure;
		for (int pos = 0; pos < formals.length; pos++) {
			List<Integer> bound = binding.get().get(pos);
			if (formals[pos] instanceof VarPositionalParameter) {
				Expression[] extra = bound.stream().map(i -> actuals[i]).toArray(Expression[]::new);
				ExpressionSet[] values = bound.stream().map(i -> given[i]).toArray(ExpressionSet[]::new);
				ListCreation listCreation = new ListCreation(callCFG, PySyntheticLocation.INSTANCE, extra);
				AnalysisState<A> listSemantics = listCreation.forwardSemanticsAux(interprocedural,
						failure.bottom(), values, null);
				slots[pos] = listSemantics.getExecution().getComputedExpressions();
				slotTypes[pos] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.LIST));
			} else if (formals[pos] instanceof VarKeywordParameter && pos == formals.length - 1) {
				List<Pair<Expression, Expression>> pairExprs = new ArrayList<>();
				List<ExpressionSet> symbExprs = new ArrayList<>();
				for (int i : bound) {
					NamedParameterExpression keyword = (NamedParameterExpression) actuals[i];
					symbExprs.add(new ExpressionSet(new Constant(StringType.INSTANCE, keyword.getParameterName(),
							PySyntheticLocation.INSTANCE)));
					symbExprs.add(given[i]);
					pairExprs.add(Pair.of(
							new StringLiteral(callCFG, PySyntheticLocation.INSTANCE, keyword.getParameterName()),
							keyword.getSubExpression()));
				}
				DictionaryCreation dictCreation = new DictionaryCreation(callCFG, PySyntheticLocation.INSTANCE,
						pairExprs.toArray(Pair[]::new));
				AnalysisState dictSemantics = dictCreation.forwardSemanticsAux(interprocedural,
						interprocedural.getAnalysis().makeLattice().bottom(), symbExprs.toArray(ExpressionSet[]::new),
						null);
				slots[pos] = dictSemantics.getExecution().getComputedExpressions();
				slotTypes[pos] = Set.of(PyClassType.lookup(LibrarySpecificationProvider.DICT));
			} else if (bound.isEmpty()) {
				slots[pos] = defaults[pos];
				slotTypes[pos] = defaultTypes[pos];
			} else {
				slots[pos] = given[bound.get(0)];
				slotTypes[pos] = givenTypes[bound.get(0)];
			}
		}
		return null;
	}
}
