package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.StatementStore;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.Call.CallType;
import it.unive.lisa.program.cfg.statement.call.OpenCall;
import it.unive.lisa.program.cfg.statement.call.ResolvedCall;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.cfg.statement.evaluation.LeftToRightEvaluation;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;
import it.unive.pylisa.libraries.PyExceptions;
import java.util.Collections;
import java.util.Objects;
import java.util.Set;

/**
 * Python's dispatch of binary operators to dunder methods, shared by all binary
 * arithmetic, bitwise and comparison operators. For every pair of runtime types
 * {@code (A, B)} of {@code a op b}, mirroring CPython:
 * <ol>
 * <li>{@code A.__op__(a, b)} is called, if {@code A} defines it and accepts a
 * {@code B} as argument;</li>
 * <li>otherwise, {@code B.__rop__(b, a)} is called, if {@code B} defines it and
 * accepts an {@code A} as argument (for arithmetic operators, only if {@code A}
 * and {@code B} are different classes);</li>
 * <li>otherwise, the operator's {@link Fallback} applies (e.g. a
 * {@code TypeError} is raised).</li>
 * </ol>
 * Methods are looked up in the class of the corresponding operand only (and in
 * its superclasses). A method "accepts" an argument if the declared type of its
 * {@code other} parameter is {@code Untyped}, or is a supertype of the
 * argument's type, where {@code float} is considered a supertype of {@code int}
 * as float methods accept ints. A method returning bottom is treated as
 * returning {@code NotImplemented}. The rule giving priority to the reflected
 * method of a proper subclass of {@code A} is not implemented, since no modeled
 * builtin class subclasses another one.
 * <p>
 * The fallback is applied only if both operands are instances of builtin
 * classes whose operators are fully modeled ({@code int}, {@code float},
 * {@code str}): for any other class, a missing method could just be missing
 * from the library models, and the result is an unknown value.
 */
public final class PyBinaryDispatch {

	/**
	 * What happens when neither operand implements the operator.
	 */
	public enum Fallback {
		/**
		 * A {@code TypeError} is raised (arithmetic, bitwise and ordering
		 * operators).
		 */
		TYPE_ERROR,

		/**
		 * The result is {@code False} ({@code ==}, falling back to identity).
		 */
		FALSE,

		/**
		 * The result is {@code True} ({@code !=}, falling back to identity).
		 */
		TRUE
	}

	private static final Set<String> FULLY_MODELED = Set.of(
			LibrarySpecificationProvider.INT,
			LibrarySpecificationProvider.FLOAT,
			LibrarySpecificationProvider.STR);

	private PyBinaryDispatch() {
	}

	/**
	 * Computes the semantics of {@code left op right}.
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where both operands have been evaluated
	 * @param expressions     the states of the operator's sub-expressions
	 * @param operator        the operator being evaluated
	 * @param left            the symbolic value of the left operand
	 * @param right           the symbolic value of the right operand
	 * @param op              the name of the dunder method (e.g.
	 *                            {@code __add__})
	 * @param rop             the name of the reflected dunder method (e.g.
	 *                            {@code __radd__}, or {@code __gt__} for
	 *                            {@code __lt__})
	 * @param comparison      whether this is a rich comparison, whose reflected
	 *                            method is also tried on operands of the same
	 *                            class
	 * @param fallback        what happens when no method applies
	 *
	 * @return the state after the operator
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> dispatch(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			StatementStore<A> expressions,
			it.unive.lisa.program.cfg.statement.BinaryExpression operator,
			SymbolicExpression left,
			SymbolicExpression right,
			String op,
			String rop,
			boolean comparison,
			Fallback fallback)
			throws SemanticException {
		Analysis<A, D> analysis = interprocedural.getAnalysis();
		Set<Type> rtsl = analysis.getRuntimeTypesOf(state, left, operator);
		Set<Type> rtsr = analysis.getRuntimeTypesOf(state, right, operator);
		ExpressionSet leftSet = new ExpressionSet(left);
		ExpressionSet rightSet = new ExpressionSet(right);

		AnalysisState<A> result = state.bottom();
		for (Type tl : rtsl)
			for (Type tr : rtsr) {
				String cl = classOf(tl);
				String cr = classOf(tr);

				AnalysisState<A> res = tryCall(interprocedural, state, expressions, operator, cl, op,
						operator.getLeft(), operator.getRight(), leftSet, rightSet, tl, tr);
				if (res == null && (comparison || !Objects.equals(cl, cr)))
					res = tryCall(interprocedural, state, expressions, operator, cr, rop,
							operator.getRight(), operator.getLeft(), rightSet, leftSet, tr, tl);
				if (res == null)
					res = cl != null && cr != null && FULLY_MODELED.contains(cl) && FULLY_MODELED.contains(cr)
							? fallback(analysis, state, operator, fallback)
							: analysis.smallStepSemantics(state,
									new PushAny(Untyped.INSTANCE, operator.getLocation()), operator);
				result = result.lub(res);
			}

		return result;
	}

	/**
	 * Yields the name of the library class modeling the Python class of the
	 * given runtime type, or {@code null} if there is none.
	 *
	 * @param type the runtime type
	 *
	 * @return the name of the class, or {@code null}
	 */
	public static String classOf(
			Type type) {
		if (type.isPointerType() && type.asPointerType().getInnerType().isUnitType())
			return type.asPointerType().getInnerType().asUnitType().getUnit().getName();
		if (type.isUnitType())
			return type.asUnitType().getUnit().getName();
		if (type.isBooleanType())
			return LibrarySpecificationProvider.BOOL;
		if (type.isStringType())
			return LibrarySpecificationProvider.STR;
		if (type.isNumericType())
			return type.asNumericType().isIntegral()
					? LibrarySpecificationProvider.INT
					: LibrarySpecificationProvider.FLOAT;
		return null;
	}

	/**
	 * Whether a method whose parameter is declared of type {@code formal} can
	 * be called (without returning {@code NotImplemented}) with an argument of
	 * type {@code actual}.
	 */
	private static boolean accepts(
			Type formal,
			Type actual) {
		if (formal.isUntyped() || formal.equals(actual))
			return true;
		if (formal.isNumericType() && actual.isNumericType())
			// float methods accept ints, int methods do not accept floats
			return !formal.asNumericType().isIntegral() || actual.asNumericType().isIntegral();
		if (formal.isNumericType() || actual.isNumericType()
				|| formal.isStringType() || actual.isStringType()
				|| formal.isBooleanType() || actual.isBooleanType())
			return false;
		return actual.canBeAssignedTo(formal);
	}

	/**
	 * Resolves {@code cls.name(args)}, looking for {@code name} only in the
	 * class {@code cls} (and in its superclasses).
	 *
	 * @param interprocedural the interprocedural analysis
	 * @param state           the state where the arguments have been evaluated
	 * @param caller          the expression performing the call
	 * @param cls             the name of the class, or {@code null}
	 * @param name            the name of the method
	 * @param args            the arguments, receiver included
	 * @param types           the runtime types of the arguments
	 *
	 * @return the resolved call, or {@code null} if {@code cls} does not define
	 *             a suitable {@code name}
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Call resolveInClass(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			Expression caller,
			String cls,
			String name,
			Expression[] args,
			Set<Type>[] types) {
		if (cls == null)
			return null;

		UnresolvedCall call = new UnresolvedCall(
				caller.getCFG(),
				caller.getLocation(),
				CallType.STATIC,
				cls,
				name,
				LeftToRightEvaluation.INSTANCE,
				args);

		Call resolved;
		try {
			resolved = interprocedural.resolve(call, types,
					state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class));
		} catch (CallResolutionException e) {
			return null;
		}

		// the call graph does not throw when no target is found, it yields an
		// open call instead
		if (resolved instanceof OpenCall || !(resolved instanceof ResolvedCall))
			return null;
		return resolved;
	}

	/**
	 * Whether values of the given runtime type are instances of a builtin class
	 * modeled as a value (e.g. {@code str} or {@code int}) rather than as an
	 * object in memory: their methods are found through {@link #classOf(Type)},
	 * since call resolution only handles classes of objects.
	 *
	 * @param type the runtime type
	 *
	 * @return whether the type is a builtin value type
	 */
	public static boolean isBuiltinValueType(
			Type type) {
		return type.isStringType() || type.isNumericType() || type.isBooleanType();
	}

	/**
	 * Calls {@code cls.name(self, other)}, returning {@code null} if
	 * {@code cls} does not define it, if it does not accept {@code other}, or
	 * if it returns bottom (i.e., {@code NotImplemented}).
	 */
	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> tryCall(
			InterproceduralAnalysis<A, D> interprocedural,
			AnalysisState<A> state,
			StatementStore<A> expressions,
			Expression operator,
			String cls,
			String name,
			Expression self,
			Expression other,
			ExpressionSet selfValue,
			ExpressionSet otherValue,
			Type selfType,
			Type otherType)
			throws SemanticException {
		@SuppressWarnings("unchecked")
		Set<Type>[] types = new Set[] { Collections.singleton(selfType), Collections.singleton(otherType) };
		Call resolved = resolveInClass(interprocedural, state, operator, cls, name, new Expression[] { self, other },
				types);
		if (resolved == null)
			return null;
		for (CodeMember target : ((ResolvedCall) resolved).getTargets()) {
			Parameter[] formals = target.getDescriptor().getFormals();
			if (formals.length != 2 || !accepts(formals[1].getStaticType(), otherType))
				return null;
		}

		AnalysisState<A> result = resolved.forwardSemanticsAux(interprocedural, state,
				new ExpressionSet[] { selfValue, otherValue }, expressions);
		operator.getMetaVariables().addAll(resolved.getMetaVariables());
		return result.isBottom() ? null : result;
	}

	private static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> fallback(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			Expression operator,
			Fallback fallback)
			throws SemanticException {
		CodeLocation loc = operator.getLocation();
		switch (fallback) {
		case FALSE:
			return analysis.smallStepSemantics(state, new Constant(BoolType.INSTANCE, false, loc), operator);
		case TRUE:
			return analysis.smallStepSemantics(state, new Constant(BoolType.INSTANCE, true, loc), operator);
		case TYPE_ERROR:
		default:
			return PyExceptions.raise(analysis, state, operator.getCFG(), loc, operator,
					LibrarySpecificationProvider.TYPE_ERROR);
		}
	}
}
