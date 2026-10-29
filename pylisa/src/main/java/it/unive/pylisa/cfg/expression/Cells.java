package it.unive.pylisa.cfg.expression;

import it.unive.lisa.analysis.AbstractDomain;
import it.unive.lisa.analysis.AbstractLattice;
import it.unive.lisa.analysis.Analysis;
import it.unive.lisa.analysis.AnalysisState;
import it.unive.lisa.analysis.SemanticException;
import it.unive.lisa.interprocedural.ScopeId;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.ProgramPoint;
import it.unive.lisa.symbolic.SymbolicExpression;
import it.unive.lisa.symbolic.heap.AccessChild;
import it.unive.lisa.symbolic.heap.HeapDereference;
import it.unive.lisa.symbolic.heap.HeapReference;
import it.unive.lisa.symbolic.heap.MemoryAllocation;
import it.unive.lisa.symbolic.value.Variable;
import it.unive.lisa.type.ReferenceType;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.analysis.heap.Allocations;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.scoping.NameKind;

/**
 * Helpers for the variables shared between a function and the functions nested
 * into it (see {@link NameKind#CELL} and {@link NameKind#FREE}). The cells of a
 * function (or of a comprehension) are the fields of an object allocated when
 * the function starts, referenced by a local variable of the function (see
 * {@link #CELLS}). A nested function refers to the cells of the enclosing
 * functions through its closure (see
 * {@link it.unive.pylisa.cfg.type.PyClosureUnit}), passed as its first
 * parameter (see {@link #CLOSURE}): the field {@code x} of the closure points
 * to the object holding the cell {@code x}.
 */
public final class Cells {

	/**
	 * The name of the local variable referencing the cells of a function.
	 */
	public static final String CELLS = "$cells";

	/**
	 * The name of the (hidden) parameter holding the closure of a nested
	 * function.
	 */
	public static final String CLOSURE = "$closure";

	private Cells() {
	}

	/**
	 * Builds the synthetic class of objects holding cells, or closures.
	 *
	 * @param program  the program
	 * @param location the location of the function (or comprehension)
	 * @param name     the name of the class
	 *
	 * @return the type of the objects of the class
	 */
	public static PyClassType cellsType(
			Program program,
			CodeLocation location,
			String name) {
		return PyClassType.register(name, new ClassUnit(location, program, name, true));
	}

	/**
	 * Yields the access to the field {@code name} of the object referenced by
	 * the given expression.
	 *
	 * @param object   the expression referencing the object
	 * @param name     the name of the field
	 * @param location the location of the access
	 *
	 * @return the access
	 */
	public static AccessChild field(
			SymbolicExpression object,
			String name,
			CodeLocation location) {
		return new AccessChild(Untyped.INSTANCE, new HeapDereference(Untyped.INSTANCE, object, location),
				new Variable(Untyped.INSTANCE, name, location), location);
	}

	/**
	 * Yields the access to the cell {@code name} of the function, referenced by
	 * the given local variable.
	 *
	 * @param cells    the name of the local variable referencing the cells
	 * @param name     the name of the cell
	 * @param location the location of the access
	 *
	 * @return the access
	 */
	public static AccessChild cell(
			String cells,
			String name,
			CodeLocation location) {
		return field(new Variable(Untyped.INSTANCE, cells, location), name, location);
	}

	/**
	 * Yields the access to the cell {@code name} of an enclosing function,
	 * through the closure of the function.
	 *
	 * @param name     the name of the cell
	 * @param location the location of the access
	 *
	 * @return the access
	 */
	public static AccessChild free(
			String name,
			CodeLocation location) {
		return field(cell(CLOSURE, name, location), name, location);
	}

	/**
	 * Allocates an object of the given type, and assigns a reference to it to
	 * the given variable.
	 *
	 * @param <A>      the kind of {@link AbstractLattice} of the state
	 * @param <D>      the kind of {@link AbstractDomain} of the analysis
	 * @param analysis the analysis
	 * @param context  the interprocedural context where the computation happens
	 * @param state    the state
	 * @param type     the type of the object
	 * @param variable the name of the variable
	 * @param location the location of the allocation
	 * @param pp       the program point of the allocation
	 *
	 * @return the state after the assignment
	 *
	 * @throws SemanticException if the analysis fails
	 */
	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> allocate(
			Analysis<A, D> analysis,
			ScopeId<?> context,
			AnalysisState<A> state,
			PyClassType type,
			String variable,
			CodeLocation location,
			ProgramPoint pp)
			throws SemanticException {
		MemoryAllocation alloc = new MemoryAllocation(type, location, false);
		HeapReference ref = new HeapReference(new ReferenceType(type), alloc, location);
		return analysis.assign(Allocations.allocate(analysis, context, state, alloc, pp),
				new Variable(Untyped.INSTANCE, variable, location), ref, context, pp);
	}
}
