package it.unive.pylisa;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.cfg.DerivedLocation;
import it.unive.pylisa.cfg.PyParameter;
import it.unive.pylisa.cfg.expression.Cells;
import it.unive.pylisa.cfg.expression.PyClosureCreation;
import it.unive.pylisa.cfg.expression.literal.PyFunctionLiteral;
import it.unive.pylisa.cfg.statement.PyCellsInit;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.cfg.type.PyClosureUnit;
import it.unive.pylisa.scoping.NameKind;
import it.unive.pylisa.scoping.PyScope;
import it.unive.pylisa.scoping.PyScope.ScopeKind;
import java.util.ArrayList;
import java.util.List;
import org.antlr.v4.runtime.Token;
import org.apache.commons.lang3.ArrayUtils;

/**
 * Helpers used by the parsers to build functions (and lambdas) that share
 * variables with the functions nested into them, or with the ones they are
 * nested into (see {@link Cells}).
 */
final class Closures {

	private Closures() {
	}

	/**
	 * Yields the name under which a function is registered in its unit: the
	 * name of the function for functions of the module and methods, and its
	 * qualified name (as CPython's {@code __qualname__}, e.g.
	 * {@code outer.<locals>.inner}) for nested functions, so that they cannot
	 * be resolved by their name only. If the unit already has a code member
	 * with the same name (e.g. a function defined in both branches of an
	 * {@code if}), the location of the function is appended to the name, so
	 * that every function is registered.
	 *
	 * @param scope    the scope of the function
	 * @param location the location of the function
	 * @param unit     the unit where the function is registered
	 * @param instance whether the function is an instance method
	 *
	 * @return the name
	 */
	static String functionName(
			PyScope scope,
			CodeLocation location,
			Unit unit,
			boolean instance) {
		PyScope parent = scope.getParent();
		String name = parent.getKind() == ScopeKind.MODULE || parent.getKind() == ScopeKind.CLASS
				? scope.getName()
				: qualifiedName(scope);
		boolean taken = instance
				? !((ClassUnit) unit).getInstanceCodeMembersByName(name, false).isEmpty()
				: !unit.getCodeMembersByName(name).isEmpty();
		return taken ? name + "@" + lineCol(location) : name;
	}

	// the line and column of a location, if known
	private static String lineCol(
			CodeLocation location) {
		CodeLocation source = DerivedLocation.source(location);
		if (source instanceof SourceCodeLocation)
			return ((SourceCodeLocation) source).getLine() + ":" + ((SourceCodeLocation) source).getCol();
		return location.getCodeLocation();
	}

	// the qualified name of a scope, as CPython's __qualname__
	private static String qualifiedName(
			PyScope scope) {
		PyScope parent = scope.getParent();
		if (parent == null || parent.getKind() == ScopeKind.MODULE)
			return scope.getName();
		if (parent.getKind() == ScopeKind.CLASS)
			return qualifiedName(parent) + "." + scope.getName();
		return qualifiedName(parent) + ".<locals>." + scope.getName();
	}

	/**
	 * Yields the parameters of a function, preceded by the hidden parameter
	 * holding its closure (see {@link Cells#CLOSURE}) if the function refers to
	 * variables of the enclosing functions.
	 *
	 * @param scope      the scope of the function
	 * @param parameters the parameters of the function
	 * @param location   the location of the function
	 *
	 * @return the parameters
	 */
	static PyParameter[] withClosure(
			PyScope scope,
			PyParameter[] parameters,
			CodeLocation location) {
		if (scope.getFreeVariables().isEmpty())
			return parameters;
		if (scope.getParent().getKind() == ScopeKind.CLASS)
			throw new UnsupportedStatementException(
					"methods referring to variables of enclosing functions are not supported");
		return ArrayUtils.insert(0, parameters, new PyParameter(location, Cells.CLOSURE));
	}

	/**
	 * Yields the statement allocating the cells of a function, if the function
	 * has variables shared with the functions nested into it.
	 *
	 * @param program  the program
	 * @param scope    the scope of the function
	 * @param cfg      the cfg of the function
	 * @param location the location of the function
	 *
	 * @return the statement, or {@code null}
	 */
	static PyCellsInit cellsInit(
			Program program,
			PyScope scope,
			CFG cfg,
			CodeLocation location) {
		if (scope.getCellVariables().isEmpty())
			return null;
		List<String> parameters = new ArrayList<>();
		for (String parameter : scope.getParameters())
			if (scope.kindOf(parameter) == NameKind.CELL)
				parameters.add(parameter);
		PyClassType type = Cells.cellsType(program, location, cfg.getDescriptor().getFullName() + "$cells");
		return new PyCellsInit(cfg, new DerivedLocation(location, "cells"), type, parameters);
	}

	/**
	 * Yields the name of the local variable referencing the cells of a
	 * comprehension, that is evaluated inline in the enclosing function.
	 *
	 * @param scope the scope of the comprehension
	 *
	 * @return the name of the variable
	 */
	static String comprehensionCells(
			PyScope scope) {
		Token start = scope.getNode().getStart();
		return Cells.CELLS + "@" + start.getLine() + ":" + start.getCharPositionInLine();
	}

	/**
	 * Yields the expression computing the value of a function when its
	 * definition is executed: the function itself, or its closure if it refers
	 * to variables of the enclosing functions.
	 *
	 * @param program  the program
	 * @param cfg      the cfg where the definition is
	 * @param location the location of the definition
	 * @param function the function
	 * @param scope    the scope of the function
	 *
	 * @return the expression
	 */
	static Expression functionValue(
			Program program,
			CFG cfg,
			CodeLocation location,
			CFG function,
			PyScope scope) {
		if (scope.getFreeVariables().isEmpty())
			return new PyFunctionLiteral(cfg, location, function);

		List<PyClosureCreation.FreeVariable> free = new ArrayList<>();
		for (String name : scope.getFreeVariables())
			free.add(new PyClosureCreation.FreeVariable(name, owner(scope.getParent(), name)));
		String typeName = function.getDescriptor().getFullName() + "$closure";
		PyClassType type = PyClassType.register(typeName,
				new PyClosureUnit(function.getDescriptor().getLocation(), program, typeName, function));
		return new PyClosureCreation(cfg, location, type, function, free);
	}

	// the local variable referencing the object holding the cell of the given
	// name, as seen from the given scope (where the closure is created), or
	// null if the cell is reached through the closure of the function of the
	// scope
	private static String owner(
			PyScope scope,
			String name) {
		while (true) {
			NameKind kind = scope.kindOf(name);
			if (kind == NameKind.CELL)
				return scope.getKind() == ScopeKind.COMPREHENSION ? comprehensionCells(scope) : Cells.CELLS;
			if (kind == NameKind.FREE && scope.getKind() == ScopeKind.COMPREHENSION)
				// comprehensions are evaluated inline
				scope = scope.getParent();
			else if (kind == NameKind.FREE && scope.getKind() == ScopeKind.FUNCTION)
				return null;
			else
				throw new UnsupportedStatementException("cannot find the cell of '" + name + "' in "
						+ scope.getName() + " (as " + kind + ")");
		}
	}
}
