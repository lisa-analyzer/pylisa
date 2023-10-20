package it.unive.pylisa.libraries.loader;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.Program;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.libraries.LibrarySpecificationParser.LibraryCreationException;
import java.util.Collection;
import java.util.HashSet;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;

public class Runtime {

	private final Collection<Method> methods = new HashSet<>();
	private final Collection<Field> fields = new HashSet<>();
	private final Collection<ClassDef> classes = new HashSet<>();

	public Collection<Method> getMethods() {
		return methods;
	}

	public Collection<Field> getFields() {
		return fields;
	}

	public Collection<ClassDef> getClasses() {
		return classes;
	}

	@Override
	public int hashCode() {
		return Objects.hash(classes, fields, methods);
	}

	@Override
	public boolean equals(
			Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Runtime other = (Runtime) obj;
		return Objects.equals(classes, other.classes) && Objects.equals(fields, other.fields)
				&& Objects.equals(methods, other.methods);
	}

	public void fillProgram(
			Program program,
			AtomicReference<CompilationUnit> rootHolder) {
		CodeLocation location = new SourceCodeLocation("standard_python_library", 0, 0);

		for (ClassDef cls : this.classes) {
			CompilationUnit c = cls.toLiSAUnit(location, program, rootHolder);
			program.addUnit(c);
			// create the corresponding type
			PyClassType.lookup(c.getName(), c);
		}
	}

	public void populateProgram(
			Program program,
			CFG init,
			CompilationUnit root) {
		CodeLocation location = new SourceCodeLocation("standard_python_library", 0, 0);

		for (Method mtd : this.methods) {
			NativeCFG construct = mtd.toLiSACfg(location, init, program);
			if (construct.getDescriptor().isInstance())
				throw new LibraryCreationException();
			program.addCodeMember(construct);
		}

		for (Field fld : this.fields) {
			Global field = fld.toLiSAObject(location, program);
			if (field.isInstance())
				throw new LibraryCreationException();
			program.addGlobal(field);
		}

		for (ClassDef cls : this.classes)
			cls.populateUnit(location, init, root);
	}
}
