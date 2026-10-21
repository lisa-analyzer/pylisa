package it.unive.pylisa.cfg;

import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.Unit;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.annotations.values.StringAnnotationValue;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.pylisa.PyClassParser;
import it.unive.pylisa.cfg.expression.PyClassAttributes;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * The bases of the classes of the program, as written in the source (see
 * {@link #BASES}), used to decide whether a class is abstract (see
 * {@link #isAbstract}): a class whose metaclass is {@code abc.ABCMeta} (e.g. a
 * subclass of {@code abc.ABC}) cannot be instantiated ({@code TypeError})
 * while it has abstract methods, that is, methods decorated with
 * {@code @abc.abstractmethod} (in the class or in its bases) that it does not
 * override. The bases are resolved to the classes of the program with the
 * same name; when they cannot be (e.g. a base that is a library class, or
 * several bases when some abstract method is inherited, as another base might
 * define it), the abstract methods of the class are not known.
 */
public final class ClassBases {

	/**
	 * The annotation of a class listing its bases, as written in the source:
	 * a member {@value #BASE} for each base, and {@value #METACLASS} for the
	 * {@code metaclass} keyword. A base that is not a name (e.g. {@code *bases})
	 * is {@value #UNKNOWN}.
	 */
	public static final String BASES = "pylisa.bases";

	/**
	 * A member of {@link #BASES}, holding a base.
	 */
	public static final String BASE = "base";

	/**
	 * A member of {@link #BASES}, holding the metaclass.
	 */
	public static final String METACLASS = "metaclass";

	/**
	 * A base that is not a name.
	 */
	public static final String UNKNOWN = "?";

	/**
	 * The annotation of a method decorated with {@code @abc.abstractmethod}.
	 */
	public static final String ABSTRACT = "pylisa.abstract_method";

	private ClassBases() {
	}

	/**
	 * Builds the annotation listing the given bases and metaclass.
	 *
	 * @param bases     the bases, as written in the source
	 * @param metaclass the metaclass, or {@code null}
	 *
	 * @return the annotation
	 */
	public static Annotation bases(
			List<String> bases,
			String metaclass) {
		List<AnnotationMember> members = new ArrayList<>();
		for (String base : bases)
			members.add(new AnnotationMember(BASE, new StringAnnotationValue(base)));
		if (metaclass != null)
			members.add(new AnnotationMember(METACLASS, new StringAnnotationValue(metaclass)));
		return new Annotation(BASES, members);
	}

	/**
	 * Whether creating an object of the given class raises {@code TypeError}
	 * because the class is abstract.
	 *
	 * @param program the program
	 * @param unit    the class
	 *
	 * @return whether the class is abstract
	 */
	public static Satisfiability isAbstract(
			Program program,
			ClassUnit unit) {
		if (program.getAllCFGs().stream().noneMatch(ClassBases::abstractMethod))
			// no abstract method in the program
			return Satisfiability.NOT_SATISFIED;
		Info info = info(program, unit, new HashSet<>());
		if (info == null)
			return Satisfiability.UNKNOWN;
		return Satisfiability.fromBoolean(info.abcMeta() && !info.abstracts().isEmpty());
	}

	// the abstract methods of a class, and whether its metaclass is ABCMeta
	private record Info(
			Set<String> abstracts,
			boolean abcMeta) {
	}

	// the info of the class, or null if it is not known
	private static Info info(
			Program program,
			ClassUnit unit,
			Set<ClassUnit> visiting) {
		if (!visiting.add(unit))
			return null;
		Set<String> inherited = new HashSet<>();
		boolean abcMeta = false;
		int bases = 0;
		for (Annotation ann : unit.getAnnotationList())
			if (ann.getAnnotationName().equals(BASES))
				for (AnnotationMember member : ann.getAnnotationMembers()) {
					// toString is the getter of the value
					String name = member.getValue().toString();
					if (member.getId().equals(METACLASS)) {
						if (!isABCMeta(program, name))
							// another metaclass
							return null;
						abcMeta = true;
						continue;
					}
					ClassUnit base = programClass(program, name);
					if (base != null) {
						Info baseInfo = info(program, base, visiting);
						if (baseInfo == null)
							return null;
						abcMeta |= baseInfo.abcMeta();
						bases++;
						inherited.addAll(baseInfo.abstracts());
					} else if (isABC(program, name))
						abcMeta = true;
					else if (!name.equals("object"))
						// e.g. a library class
						return null;
				}

		if (bases > 1 && !inherited.isEmpty())
			// several bases, and abstract methods: one of the bases might
			// define them, depending on the order of resolution
			return null;
		Set<String> abstracts = new HashSet<>();
		for (String name : inherited)
			if (!defines(unit, name))
				abstracts.add(name);
		for (CodeMember member : members(unit))
			if (abstractMethod(member))
				abstracts.add(member.getDescriptor().getName());
		return new Info(abstracts, abcMeta);
	}

	// whether the class defines name, with a method that is not abstract or
	// in its body
	private static boolean defines(
			ClassUnit unit,
			String name) {
		for (CodeMember member : members(unit))
			if (member.getDescriptor().getName().equals(name) && !abstractMethod(member))
				return true;
		return PyClassAttributes.attribute(unit, name) != null;
	}

	// the methods of the class (also the static ones)
	private static List<CodeMember> members(
			ClassUnit unit) {
		List<CodeMember> members = new ArrayList<>(unit.getInstanceCodeMembers(false));
		for (CodeMember member : unit.getCodeMembers())
			if (!member.getDescriptor().getName().equals(PyClassParser.INSTRUMENTED_CLASS_INIT_NAME))
				members.add(member);
		return members;
	}

	private static boolean abstractMethod(
			CodeMember member) {
		return member instanceof CFG && member.getDescriptor().getAnnotationList().stream()
				.anyMatch(a -> a.getAnnotationName().equals(ABSTRACT));
	}

	// the class of the program with the given name, if any
	private static ClassUnit programClass(
			Program program,
			String name) {
		Unit unit = program.getUnit(name);
		return unit instanceof ClassUnit && ((ClassUnit) unit).getLocation() instanceof SourceCodeLocation
				? (ClassUnit) unit
				: null;
	}

	private static boolean isABC(
			Program program,
			String name) {
		return programClass(program, name) == null && (name.equals("ABC") || name.equals("abc.ABC"));
	}

	private static boolean isABCMeta(
			Program program,
			String name) {
		return programClass(program, name) == null && (name.equals("ABCMeta") || name.equals("abc.ABCMeta"));
	}
}
