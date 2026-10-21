package it.unive.pylisa.cfg;

import it.unive.lisa.lattices.Satisfiability;
import it.unive.lisa.program.ClassUnit;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.annotations.Annotation;
import it.unive.lisa.program.annotations.AnnotationMember;
import it.unive.lisa.program.annotations.values.StringAnnotationValue;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.pylisa.cfg.expression.PyBinaryDispatch;
import java.util.ArrayList;
import java.util.List;

/**
 * The bases of the classes of the program, as written in the source (see
 * {@link #BASES}; {@code PyClassParser} resolves them to the ancestors of the
 * classes), and the abstract classes: a class whose metaclass is
 * {@code abc.ABCMeta} (e.g. a subclass of {@code abc.ABC}) cannot be
 * instantiated ({@code TypeError}) while it has abstract methods, that is,
 * methods decorated with {@code @abc.abstractmethod} that it does not override
 * (see {@link #isAbstract}).
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
	 * The annotation of a method decorated with {@code @abc.abstractmethod}
	 * (that must not be propagated to the methods overriding it, see
	 * {@link Annotation#isInherited()}).
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
		// not propagated by LiSA to the subclasses
		return new Annotation(BASES, members, true);
	}

	/**
	 * Whether creating an object of the given class raises {@code TypeError}
	 * because the class is abstract: its metaclass is {@code abc.ABCMeta}, and
	 * some of its attributes is an abstract method, that is, the first class
	 * in its method resolution order defining it (see {@link PyMro#definer})
	 * defines it as an abstract method. This is not known for classes whose
	 * bases are not known.
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
		List<CompilationUnit> mro = untracked(unit) ? null : PyMro.of(unit);
		if (mro == null || mro.stream().anyMatch(ClassBases::untracked))
			return Satisfiability.UNKNOWN;
		if (mro.stream().noneMatch(ClassBases::declaresABCMeta))
			return Satisfiability.NOT_SATISFIED;
		for (CompilationUnit cu : mro)
			for (CodeMember member : cu.getInstanceCodeMembers(false))
				if (abstractMethod(member) && PyMro.definer(unit, member.getDescriptor().getName(), null) == cu)
					return Satisfiability.SATISFIED;
		return Satisfiability.NOT_SATISFIED;
	}

	/**
	 * Whether the given class might have {@code abc.ABCMeta} as metaclass,
	 * whose instance checks also hold for the virtual subclasses registered
	 * with {@code register}, that are not tracked.
	 *
	 * @param unit the class
	 *
	 * @return whether the class might be an abstract base class
	 */
	public static boolean mightBeAbstractBase(
			ClassUnit unit) {
		List<CompilationUnit> mro = untracked(unit) ? null : PyMro.of(unit);
		return mro == null || mro.stream().anyMatch(cu -> untracked(cu) || declaresABCMeta(cu));
	}

	// whether the bases of the class are not known
	private static boolean untracked(
			CompilationUnit unit) {
		return unit.getAnnotationList().stream()
				.anyMatch(a -> a.getAnnotationName().equals(PyBinaryDispatch.UNTRACKED_BASES));
	}

	// whether the class has ABC as a base, or ABCMeta as metaclass
	private static boolean declaresABCMeta(
			CompilationUnit unit) {
		for (Annotation ann : unit.getAnnotationList())
			if (ann.getAnnotationName().equals(BASES))
				for (AnnotationMember member : ann.getAnnotationMembers()) {
					// toString is the getter of the value
					String name = member.getValue().toString();
					if (member.getId().equals(METACLASS) ? name.equals("ABCMeta") || name.equals("abc.ABCMeta")
							: name.equals("ABC") || name.equals("abc.ABC"))
						return true;
				}
		return false;
	}

	private static boolean abstractMethod(
			CodeMember member) {
		return member instanceof CFG && member.getDescriptor().getAnnotationList().stream()
				.anyMatch(a -> a.getAnnotationName().equals(ABSTRACT));
	}
}
