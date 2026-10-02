package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.Lattice;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.analysis.combination.ValueLatticeProduct;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.string.BoundedStringSet;
import it.unive.lisa.lattices.SimpleAbstractState;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.value.Constant;
import it.unive.lisa.symbolic.value.Identifier;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import java.util.Optional;

/**
 * Reads the abstract value that a value domain associates with an identifier,
 * as a domain-independent {@link Val}. There is one reader per value domain a
 * test configuration can use: this is the only place where tests depend on the
 * shape of a specific value domain.
 */
@FunctionalInterface
public interface ValueReader {

	/**
	 * Reads the value of an identifier.
	 *
	 * @param valueState the value component of an analysis state
	 * @param identifier the identifier to read (a variable or a heap location)
	 *
	 * @return the value, or {@code null} if the state holds bottom for the
	 *             identifier (no execution reaches this point with a value
	 *             for it)
	 *
	 * @throws IllegalArgumentException if {@code valueState} does not come
	 *                                      from the domain this reader is for
	 */
	Val read(
			Lattice<?> valueState,
			Identifier identifier);

	/**
	 * Reads the value of an expression in a state: every expression the heap
	 * rewrites it to is read, an identifier with this reader and a constant as
	 * it is; any other expression is unknown.
	 *
	 * @param <A>        the kind of abstract state
	 * @param <D>        the kind of abstract domain
	 * @param analysis   the analysis that computed the state
	 * @param state      the state, made of heap, value and type components
	 * @param expression the expression
	 * @param point      the program point of the state
	 *
	 * @return the join of the values, or empty if no execution gives the
	 *             expression a value
	 *
	 * @throws SemanticException     if the expression cannot be rewritten
	 * @throws IllegalStateException if the state is not made of heap, value
	 *                                   and type components
	 */
	default <A extends AbstractLattice<A>, D extends AbstractDomain<A>> Optional<Val> valueOf(
			Analysis<A, D> analysis,
			AnalysisState<A> state,
			SymbolicExpression expression,
			ProgramPoint point)
			throws SemanticException {
		if (!(state.getExecutionState() instanceof SimpleAbstractState<?, ?, ?> components))
			throw new IllegalStateException("Only states made of heap, value and type components are supported, got "
					+ state.getExecutionState().getClass().getName());
		Val result = null;
		for (SymbolicExpression denoted : analysis.rewrite(state, expression, point)) {
			Val value;
			if (denoted instanceof Identifier identifier)
				value = read(components.valueState, identifier);
			else if (denoted instanceof Constant constant)
				value = Val.exact(constant.getValue());
			else
				value = Val.top();
			if (value != null)
				result = result == null ? value : result.join(value);
		}
		return Optional.ofNullable(result);
	}

	/**
	 * Yields the reader for pylisa's {@link ConstantPropagation}.
	 *
	 * @return the reader
	 */
	static ValueReader constantPropagation() {
		return (
				valueState,
				identifier) -> {
			if (!(valueState instanceof ValueEnvironment<?> environment))
				throw new IllegalArgumentException("Not a constant propagation state: " + valueState);
			Lattice<?> value = environment.getState(identifier);
			if (!(value instanceof ConstantPropagation constant))
				throw new IllegalArgumentException("Not a constant propagation value: " + value);
			if (constant.isBottom())
				return null;
			if (constant.isTop())
				return Val.top();
			return Val.exact(constant.getConstant());
		};
	}

	/**
	 * Yields the reader for the product of pylisa's {@link ConstantPropagation}
	 * with a {@link BoundedStringSet}. The two components describe the same
	 * values, so the more precise of their two readings is returned: an exact
	 * constant, else a finite set of strings, else unknown.
	 *
	 * @return the reader
	 */
	static ValueReader constantPropagationWithStringSets() {
		ValueReader constants = constantPropagation();
		return (
				valueState,
				identifier) -> {
			if (!(valueState instanceof ValueLatticeProduct<?, ?> product))
				throw new IllegalArgumentException("Not a product state: " + valueState);
			Val constant = constants.read(product.first, identifier);
			if (!(constant instanceof Val.Top))
				return constant;
			if (!(product.second instanceof ValueEnvironment<?> environment))
				throw new IllegalArgumentException("Not a string set state: " + product.second);
			Lattice<?> value = environment.getState(identifier);
			if (!(value instanceof BoundedStringSet.BSS strings))
				throw new IllegalArgumentException("Not a string set value: " + value);
			if (strings.isTop() || strings.isBottom() || strings.elements().isEmpty())
				return constant;
			return Val.oneOf(strings.elements());
		};
	}
}
