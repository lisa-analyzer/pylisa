package it.unive.pylisa.cfg;

import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.program.language.hierarchytraversal.HierarchyTraversalStrategy;
import it.unive.pylisa.PyClassParser;
import it.unive.pylisa.cfg.expression.PyClassAttributes;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

/**
 * The method resolution order of python classes: the C3 linearization of a
 * class and of its bases (the ancestors of its unit, in the order in which they
 * are listed in the class definition, see {@code PyClassParser}). An attribute
 * of a class (e.g. a method) is the one defined by the first class in the order
 * that defines it. This is also the traversal strategy of the hierarchy for
 * the resolution of calls (see {@link #traverse}).
 */
public final class PyMro implements HierarchyTraversalStrategy {

	/**
	 * The singleton instance of this class.
	 */
	public static final PyMro INSTANCE = new PyMro();

	private PyMro() {
	}

	/**
	 * Yields the method resolution order of the given class.
	 *
	 * @param unit the class
	 *
	 * @return the classes, starting from {@code unit}, or {@code null} if the
	 *             order cannot be built (python raises {@code TypeError} when
	 *             defining such a class)
	 */
	public static List<CompilationUnit> of(
			CompilationUnit unit) {
		List<List<CompilationUnit>> sequences = new ArrayList<>();
		List<CompilationUnit> bases = bases(unit);
		for (CompilationUnit base : bases) {
			List<CompilationUnit> mro = of(base);
			if (mro == null)
				return null;
			sequences.add(new LinkedList<>(mro));
		}
		sequences.add(new LinkedList<>(bases));

		List<CompilationUnit> result = new ArrayList<>();
		result.add(unit);
		while (sequences.stream().anyMatch(s -> !s.isEmpty())) {
			// the first head that is not in the tail of any sequence
			CompilationUnit next = null;
			for (List<CompilationUnit> seq : sequences)
				if (!seq.isEmpty() && sequences.stream().noneMatch(s -> s.indexOf(seq.get(0)) > 0)) {
					next = seq.get(0);
					break;
				}
			if (next == null)
				return null;
			result.add(next);
			for (List<CompilationUnit> seq : sequences)
				if (!seq.isEmpty() && seq.get(0) == next)
					seq.remove(0);
		}
		return result;
	}

	// the bases of the class, in order
	private static List<CompilationUnit> bases(
			CompilationUnit unit) {
		if (unit instanceof ClassUnit)
			return new ArrayList<>(((ClassUnit) unit).getSuperclasses());
		return new ArrayList<>(unit.getImmediateAncestors());
	}

	/**
	 * Yields the class defining the given attribute for the objects of
	 * {@code unit}: the first class in its method resolution order, after
	 * {@code after} (if not {@code null}, as for {@code super()}), that defines
	 * a method with that name, or assigns it in its body.
	 *
	 * @param unit  the class
	 * @param name  the name of the attribute
	 * @param after the class after which the attribute is looked up, or
	 *                  {@code null}
	 *
	 * @return the class, or {@code null} if no class defines it (also when the
	 *             order cannot be built)
	 */
	public static CompilationUnit definer(
			CompilationUnit unit,
			String name,
			CompilationUnit after) {
		List<CompilationUnit> mro = of(unit);
		if (mro == null)
			return null;
		int from = after == null ? 0 : mro.indexOf(after) + 1;
		if (after != null && from == 0)
			return null;
		for (CompilationUnit cu : mro.subList(from, mro.size()))
			if (defines(cu, name))
				return cu;
		return null;
	}

	/**
	 * Whether the class itself (not its bases) defines the given attribute:
	 * as a method (also static), or by assigning it in its body.
	 *
	 * @param unit the class
	 * @param name the name of the attribute
	 *
	 * @return whether the class defines the attribute
	 */
	public static boolean defines(
			CompilationUnit unit,
			String name) {
		if (!unit.getInstanceCodeMembersByName(name, false).isEmpty())
			return true;
		for (CodeMember member : unit.getCodeMembersByName(name))
			if (!member.getDescriptor().getName().equals(PyClassParser.INSTRUMENTED_CLASS_INIT_NAME))
				return true;
		return unit instanceof ClassUnit && ((ClassUnit) unit).getLocation() instanceof SourceCodeLocation
				&& PyClassAttributes.ownAttribute((ClassUnit) unit, name) != null;
	}

	/**
	 * Traverses the method resolution order of {@code start}. For a call, the
	 * traversal stops at the first class defining the called name, as python
	 * looks methods up by name: if that method does not accept the arguments,
	 * the ones of the next classes are not considered.
	 */
	@Override
	public Iterable<CompilationUnit> traverse(
			Statement st,
			CompilationUnit start) {
		List<CompilationUnit> mro = of(start);
		if (mro == null)
			// e.g. a library class with several ancestors
			mro = new ArrayList<>(List.of(start));
		if (!(st instanceof UnresolvedCall))
			return mro;
		String name = ((UnresolvedCall) st).getTargetName();
		Collection<CompilationUnit> traversed = new ArrayList<>();
		for (CompilationUnit cu : mro) {
			traversed.add(cu);
			if (defines(cu, name))
				break;
		}
		return traversed;
	}
}
