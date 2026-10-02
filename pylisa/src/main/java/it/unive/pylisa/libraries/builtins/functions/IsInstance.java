package it.unive.pylisa.libraries.builtins.functions;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.BoolType;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.PushAny;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Type;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.natives.LibraryNative;
import it.unive.pylisa.libraries.natives.ModelState;
import it.unive.pylisa.program.PyClassUnit;
import java.util.List;
import java.util.Set;

/**
 * The model of the builtin {@code isinstance(obj, classinfo)}.
 * <p>
 * The result is decided from the types the two arguments may have at run
 * time. It is {@code True} (or {@code False}) only when {@code classinfo} is
 * a class and every type {@code obj} may have is certainly an instance (or
 * certainly not an instance) of it; in every other case, such as a tuple of
 * classes, a value of unknown type or types that disagree, it is an unknown
 * boolean. The call raises nothing: the {@code TypeError} Python raises for a
 * {@code classinfo} that is not a class is not modelled.
 * </p>
 * <p>
 * Values of the primitive types of the analysis are instances of the builtin
 * classes Python gives them: a string of {@code str}, {@code None} of
 * {@code NoneType}, a boolean of {@code bool} and {@code int}, an integer of
 * {@code int}, a float of {@code float}, and all of them of {@code object}.
 * An object is an instance of its class and of the ancestors of the class.
 * </p>
 */
public class IsInstance extends LibraryNative {

	private static final int OBJ = 0;

	private static final int CLASSINFO = 1;

	private static final String OBJECT = "builtins.object";

	/**
	 * Builds the model of one call.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments of the call
	 */
	protected IsInstance(
			CFG cfg,
			CodeLocation location,
			Expression... parameters) {
		super(cfg, location, "isinstance", parameters);
	}

	/**
	 * Builds the model of one call. This is the factory the library loader
	 * uses for native implementations.
	 *
	 * @param cfg        the CFG the call belongs to
	 * @param location   the location of the call
	 * @param parameters the arguments of the call
	 *
	 * @return the model
	 */
	public static IsInstance build(
			CFG cfg,
			CodeLocation location,
			Expression[] parameters) {
		return new IsInstance(cfg, location, parameters);
	}

	@Override
	protected <A extends AbstractLattice<A>, D extends AbstractDomain<A>> ModelState<A, D> model(
			ModelState<A, D> state,
			ExpressionSet[] arguments)
			throws SemanticException {
		return state.forEachCombination(List.of(arguments[OBJ], arguments[CLASSINFO]),
				(current, values) -> current.returning(result(current, values.get(OBJ), values.get(CLASSINFO))));
	}

	private <A extends AbstractLattice<A>, D extends AbstractDomain<A>> SymbolicExpression result(
			ModelState<A, D> state,
			SymbolicExpression obj,
			SymbolicExpression classinfo)
			throws SemanticException {
		Set<Type> classes = state.runtimeTypes(classinfo);
		Set<Type> types = state.runtimeTypes(obj);
		if (classes.size() != 1 || !(classes.iterator().next() instanceof PyClassType target) || types.isEmpty())
			return unknown();
		boolean someInstance = false;
		boolean someNot = false;
		for (Type type : types) {
			Boolean instance = isInstance(type, target);
			if (instance == null)
				return unknown();
			someInstance |= instance;
			someNot |= !instance;
		}
		if (someInstance && someNot)
			return unknown();
		return new Constant(BoolType.INSTANCE, someInstance, getLocation());
	}

	/**
	 * Decides whether a value of a type is an instance of a class.
	 *
	 * @param type   the type of the value
	 * @param target the class
	 *
	 * @return whether it is, or {@code null} if that is not known
	 */
	private static Boolean isInstance(
			Type type,
			PyClassType target) {
		String name = target.getUnit().getName();
		if (OBJECT.equals(name))
			return true;
		// a class whose own hierarchy is not known may check instances itself
		// (a metaclass's __instancecheck__)
		if (target.getUnit() instanceof PyClassUnit targetUnit && !targetUnit.hasKnownHierarchy())
			return null;
		if (type.isStringType())
			return name.equals("builtins.str");
		if (type.isNullType())
			return name.equals("builtins.NoneType");
		if (type.isBooleanType())
			return name.equals("builtins.bool") || name.equals("builtins.int");
		if (type.isNumericType() && type.asNumericType().isIntegral())
			return name.equals("builtins.int");
		if (type.isNumericType())
			return name.equals("builtins.float");
		if (type instanceof ReferenceType reference && reference.getInnerType() instanceof PyClassType of) {
			boolean instance = of.getUnit().isInstanceOf(target.getUnit());
			// ancestors that may not be the class's own make a success
			// uncertain, unknown ancestors a failure
			if (of.getUnit() instanceof PyClassUnit unit
					&& (instance ? !unit.hasUnambiguousAncestors() : !unit.hasKnownHierarchy()))
				return null;
			return instance;
		}
		return null;
	}

	private PushAny unknown() {
		return new PushAny(BoolType.INSTANCE, getLocation());
	}
}
