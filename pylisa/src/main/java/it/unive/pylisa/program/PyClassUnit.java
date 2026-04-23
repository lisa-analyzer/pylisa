package it.unive.pylisa.program;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.InterfaceUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.cfg.CodeLocation;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * A Python-specific class unit that extends {@link ClassUnit} to allow any mix
 * of ancestors — {@link ClassUnit}, {@link InterfaceUnit}, and unknown units
 * (e.g. {@link UnknownModuleUnit}) — in a single ordered list that approximates
 * Python's MRO ordering.
 * <p>
 * The standard LiSA {@link ClassUnit#addAncestor(CompilationUnit)} throws a
 * {@link ClassCastException} for non-{@code ClassUnit}/{@code InterfaceUnit}
 * ancestors. Python's dynamic inheritance can reference modules, unknown
 * symbols, or other non-ClassUnit compilation units, so this subclass relaxes
 * that constraint by maintaining a private ancestors list and overriding the
 * ancestor-related accessors.
 * <p>
 * Because this class extends {@link ClassUnit}, all
 * {@code instanceof ClassUnit} checks throughout the frontend continue to work
 * correctly for user-defined Python classes, and
 * {@link it.unive.lisa.program.language.validation.BaseValidationLogic} routes
 * to the correct {@code validateAndFinalize(ClassUnit)} overload.
 */
public class PyClassUnit extends ClassUnit {

	private static final Logger log = LogManager.getLogger(PyClassUnit.class);

	/**
	 * All ancestors in source-declaration order, mixing known and unknown
	 * units.
	 */
	private final List<CompilationUnit> allAncestors = new LinkedList<>();

	/**
	 * Known instances (direct subclasses) of this unit, populated by the
	 * validator.
	 */
	private final Set<Unit> instances = new LinkedHashSet<>();

	/**
	 * The Python-visible qualified name (e.g. {@code dispatch.config.Secret}).
	 * Multiple units can share a base name when the same simple name is
	 * conditionally redefined in the same scope — each definition site still
	 * has a unique identity ({@link #getName()}), but they collapse to the same
	 * base name at the language level (imports, attribute access).
	 */
	private final String baseName;

	/**
	 * Builds a Python class unit. The {@code name} is taken as both the
	 * identity name and the Python-visible base name — use this overload for
	 * classes coming from library specs (one def-site per name).
	 *
	 * @param location the source location of the class definition
	 * @param program  the LiSA program this unit belongs to
	 * @param name     the fully-qualified name of the class
	 * @param sealed   whether this class is sealed (cannot be subclassed)
	 */
	public PyClassUnit(
			CodeLocation location,
			Program program,
			String name,
			boolean sealed) {
		this(location, program, name, name, sealed);
	}

	/**
	 * Builds a Python class unit with an explicit base name, allowing multiple
	 * def-sites to share the same Python-visible qualified name while each
	 * retaining a unique identity. Intended for user code where the same class
	 * name is declared in multiple control-flow branches.
	 *
	 * @param location the source location of the class definition
	 * @param program  the LiSA program this unit belongs to
	 * @param name     the unique identity name (typically base name +
	 *                     {@code "@line:col"})
	 * @param baseName the Python-visible qualified name (without location
	 *                     suffix)
	 * @param sealed   whether this class is sealed (cannot be subclassed)
	 */
	public PyClassUnit(
			CodeLocation location,
			Program program,
			String name,
			String baseName,
			boolean sealed) {
		super(location, program, name, sealed);
		this.baseName = baseName;
	}

	/**
	 * Yields the Python-visible qualified name of this class (no location
	 * suffix). Two conditionally-defined classes with the same textual name
	 * share a base name but differ in {@link #getName()}.
	 *
	 * @return the base name
	 */
	public String getBaseName() {
		return baseName;
	}

	/**
	 * Adds an ancestor to this class unit. Unlike {@link ClassUnit}, accepts
	 * any {@link CompilationUnit} — including {@link UnknownModuleUnit} — and
	 * logs a warning if the ancestor is not a {@link ClassUnit} or
	 * {@link InterfaceUnit}, since {@code super()} resolution will fall back to
	 * {@code builtins.object} in that case.
	 *
	 * @param unit the ancestor compilation unit
	 *
	 * @return {@code true} if the ancestor was added
	 */
	@Override
	public boolean addAncestor(
			CompilationUnit unit) {
		if (!(unit instanceof ClassUnit) && !(unit instanceof InterfaceUnit))
			log.warn("Ancestor '{}' of class '{}' is not a ClassUnit/InterfaceUnit; "
					+ "super() will fall back to builtins.object",
					unit.getName(), getName());
		return allAncestors.add(unit);
	}

	/**
	 * {@inheritDoc}
	 * <p>
	 * Returns all ancestors in source-declaration order.
	 */
	@Override
	public Collection<CompilationUnit> getImmediateAncestors() {
		return Collections.unmodifiableList(allAncestors);
	}

	/**
	 * Records a subclass (or self) as an instance of this unit, and propagates
	 * the registration up through the ancestor chain.
	 * <p>
	 * Overrides {@link ClassUnit#addInstance(Unit)} because that method only
	 * iterates over {@code ClassUnit.superclasses}, which is always empty for
	 * {@link PyClassUnit} (ancestors are stored in {@link #allAncestors}).
	 *
	 * @param unit the unit to record as an instance
	 */
	@Override
	public void addInstance(
			Unit unit)
			throws it.unive.lisa.program.ProgramValidationException {
		instances.add(unit);
		for (CompilationUnit ancestor : allAncestors)
			ancestor.addInstance(unit);
	}

	/**
	 * {@inheritDoc}
	 * <p>
	 * Checks {@code this == unit}, then recursively checks all ancestors.
	 */
	@Override
	public boolean isInstanceOf(
			CompilationUnit unit) {
		if (this == unit)
			return true;
		for (CompilationUnit ancestor : allAncestors)
			if (ancestor.isInstanceOf(unit))
				return true;
		return false;
	}

	/**
	 * Returns the set of direct subclasses recorded for this unit.
	 *
	 * @return unmodifiable view of the instances set
	 */
	public Collection<Unit> getInstances() {
		return Collections.unmodifiableSet(instances);
	}
}
