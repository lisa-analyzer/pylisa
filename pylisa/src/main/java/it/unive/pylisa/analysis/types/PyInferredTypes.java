package it.unive.pylisa.analysis.types;

import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.SemanticOracle;
import it.unive.lisa.analysis.nonrelational.type.TypeEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.lattices.types.TypeSet;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.value.BinaryExpression;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.lisa.symbolic.value.ValueExpression;
import it.unive.lisa.symbolic.value.operator.binary.BinaryOperator;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.TypeSystem;
import it.unive.pylisa.symbolic.operators.compare.Is;
import it.unive.pylisa.symbolic.operators.compare.IsNot;
import java.util.Set;
import java.util.stream.Collectors;

/**
 * The inferred types of {@link InferredTypes}, also refined by the identity
 * tests with {@code None} (see {@link Is}): in the branches of
 * {@code x is None}, {@code x} has only the type of {@code None}, or all its
 * types but that one.
 */
public class PyInferredTypes extends InferredTypes {

	@Override
	public TypeEnvironment<TypeSet> assumeBinaryExpression(
			TypeEnvironment<TypeSet> environment,
			BinaryExpression expression,
			ProgramPoint src,
			ProgramPoint dest,
			SemanticOracle oracle)
			throws SemanticException {
		BinaryOperator operator = expression.getOperator();
		if (operator != Is.INSTANCE && operator != IsNot.INSTANCE)
			return super.assumeBinaryExpression(environment, expression, src, dest, oracle);

		ValueExpression left = (ValueExpression) expression.getLeft();
		ValueExpression right = (ValueExpression) expression.getRight();
		Identifier id;
		if (left instanceof Identifier && isNone(eval(environment, right, src, oracle)))
			id = (Identifier) left;
		else if (right instanceof Identifier && isNone(eval(environment, left, src, oracle)))
			id = (Identifier) right;
		else
			return environment;

		TypeSystem types = src.getProgram().getTypes();
		TypeSet starting = environment.getState(id);
		if (starting.isBottom())
			return environment.bottom();
		Set<Type> elems = starting.isTop() ? types.getTypes() : starting.elements;
		// x is None keeps the type of None, x is not None all the others
		Set<Type> update = elems.stream()
				.filter(t -> t.isNullType() == (operator == Is.INSTANCE))
				.collect(Collectors.toSet());
		if (update.isEmpty())
			return environment.bottom();
		return environment.putState(id, new TypeSet(types, update));
	}

	// whether the types are only the one of None
	private static boolean isNone(
			TypeSet types) {
		return !types.isTop() && !types.isBottom() && !types.elements.isEmpty()
				&& types.elements.stream().allMatch(Type::isNullType);
	}
}
