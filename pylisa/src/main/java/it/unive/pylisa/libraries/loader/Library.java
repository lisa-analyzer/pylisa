package it.unive.pylisa.libraries.loader;

import it.unive.lisa.program.*;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Assignment;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.statement.ImportClass;
import it.unive.pylisa.cfg.statement.ImportFunction;
import it.unive.pylisa.cfg.statement.ImportModule;
import it.unive.pylisa.cfg.statement.PythonScopedAttributeAccessRef;
import it.unive.pylisa.cfg.type.*;
import it.unive.pylisa.libraries.LibrarySpecificationParser.LibraryCreationException;
import it.unive.pylisa.program.FunctionUnit;
import it.unive.pylisa.program.ModuleUnit;
import it.unive.pylisa.program.PySyntheticLocation;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;

public class Library {
	private final String name;
	private final String location;
	private final Collection<Method> methods = new HashSet<>();
	private final Collection<Field> fields = new HashSet<>();
	private final Collection<ClassDef> classes = new ArrayList<>();
	private final List<String> imports = new ArrayList<>();

	public Library(
			String name,
			String location) {
		this.name = name;
		this.location = location;
	}

	public String getName() {
		return name;
	}

	public String getLocation() {
		return location;
	}

	public Collection<Method> getMethods() {
		return methods;
	}

	public Collection<Field> getFields() {
		return fields;
	}

	/**
	 * Yields the names of the modules this library imports, in the order they
	 * must be imported before the library itself, as the import statements at
	 * the top of a Python module are executed before its body.
	 *
	 * @return the names of the imported modules
	 */
	public List<String> getImports() {
		return imports;
	}

	public Collection<ClassDef> getClasses() {
		return classes;
	}

	@Override
	public int hashCode() {
		return Objects.hash(classes, fields, location, methods, name);
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
		Library other = (Library) obj;
		return Objects.equals(classes, other.classes) && Objects.equals(fields, other.fields)
				&& Objects.equals(location, other.location) && Objects.equals(methods, other.methods)
				&& Objects.equals(name, other.name);
	}

	@Override
	public String toString() {
		return "Library [name=" + name + ", location=" + location + "]";
	}

	/*
	 * public CodeUnit toLiSAUnit( Program program,
	 * AtomicReference<CompilationUnit> rootHolder) { CodeLocation location =
	 * new SourceCodeLocation(this.location, 0, 0); CodeUnit unit = new
	 * CodeUnit(location, program, name); //program.addUnit(unit); for (ClassDef
	 * cls : this.classes) { CompilationUnit c = cls.toLiSAUnit(location,
	 * program, rootHolder, unit); //program.addUnit(c); // type registration is
	 * a side effect of the constructor if (cls.getTypeName() == null) {
	 * PyClassType.register(c.getName(), c); /*if (cls.getReifiedTypeName() !=
	 * null) { ReificationRegistry.registerRule(new ReifiedRoleType(unitType,
	 * cls.getReifiedTypeName().toLiSAType())); } //; } else try { Class<?> type
	 * = Class.forName(cls.getTypeName()); Constructor<?> constructor =
	 * type.getConstructor(CompilationUnit.class); constructor.newInstance(c); }
	 * catch (ClassNotFoundException | SecurityException |
	 * IllegalArgumentException | IllegalAccessException | NoSuchMethodException
	 * | InstantiationException | InvocationTargetException e) { throw new
	 * LibraryCreationException(e); } } return unit; }
	 */

	public ModuleUnit toLiSAPythonModuleUnit(
			Program program,
			AtomicReference<it.unive.lisa.program.CompilationUnit> rootHolder,
			CFG init) {

		CodeLocation location = new SourceCodeLocation(this.location, 0, 0);

		ModuleUnit module = new ModuleUnit(location, program, name);

		program.addUnit(module);

		PyModuleType.register(name, module);

		CFG moduleInit = createModuleInitCFG(location, program, module, init);

		module.addCodeMember(moduleInit);

		/*
		 * for (ClassDef cls : this.classes) { CompilationUnit classUnit =
		 * cls.toLiSAUnit(location, program, rootHolder);
		 * program.addUnit(classUnit); // register class type
		 * PyClassType.register(classUnit.getName(), classUnit);
		 * module.addInstanceGlobal( new Global(location, module,
		 * classUnit.getName(), PyClassType.lookup(classUnit.getName())) );
		 * addClassBindingToModuleInit(moduleInit, classUnit);
		 */

		return module;
	}

	/**
	 * Yields the units of the modules this library imports when it is
	 * initialized: its parent packages that are part of the program, then the
	 * modules it declares to import.
	 *
	 * @param program the program the library is added to
	 *
	 * @return the units, in import order
	 */
	private List<CompilationUnit> importedUnits(
			Program program) {
		List<String> names = new ArrayList<>();
		for (int dot = name.indexOf('.'); dot != -1; dot = name.indexOf('.', dot + 1))
			names.add(name.substring(0, dot));
		names.addAll(imports);
		List<CompilationUnit> units = new ArrayList<>();
		for (String imported : names)
			if (program.getUnit(imported) instanceof CompilationUnit unit)
				units.add(unit);
		return units;
	}

	private CFG createModuleInitCFG(
			CodeLocation location,
			Program program,
			ModuleUnit module,
			CFG init) {
		CodeMemberDescriptor desc = new CodeMemberDescriptor(
				location,
				module,
				false,
				"$init",
				Untyped.INSTANCE);
		desc.setOverridable(false);
		PyCFG initCFG = new PyCFG(desc);
		Expression e = null;
		// as in a Python module, the imports run before the rest of the body:
		// the parent packages first, then the imported modules
		for (CompilationUnit imported : importedUnits(program)) {
			ImportModule importStatement = new ImportModule(initCFG, PySyntheticLocation.INSTANCE,
					imported.getName(), imported);
			initCFG.addNode(importStatement, e == null);
			if (e != null)
				initCFG.addEdge(new SequentialEdge(e, importStatement));
			e = importStatement;
		}
		for (ClassDef cls : getClasses()) {
			// Use the class type itself, not an instance
			ClassUnit lisaClassUnit = cls.toLiSAClassUnit(program, init);
			PyClassType classType = PyClassType.register(lisaClassUnit.getName(), lisaClassUnit);
			Expression target = new PythonScopedAttributeAccessRef(initCFG, PySyntheticLocation.INSTANCE, module,
					new Global(PySyntheticLocation.INSTANCE, module, cls.getName(), false));

			Assignment classVarAssign = new PyAssign(initCFG, PySyntheticLocation.INSTANCE, target,
					new ImportClass(initCFG, PySyntheticLocation.INSTANCE, cls.getName(), lisaClassUnit));
			initCFG.addNode(classVarAssign, e == null);
			if (e != null) {
				initCFG.addEdge(new SequentialEdge(e, classVarAssign));
			}
			e = classVarAssign;
		}
		// TODO: Ww should assign methods like we did for classes.
		// builtins.staticmethod should become accessible by a attribute of
		// builtins.
		for (Method mtd : getMethods()) {
			// IMPLEMENT THIS

			FunctionUnit functionUnit = mtd.toLiSAFunctionUnit(PySyntheticLocation.INSTANCE, init, program, module,
					name);
			PyFunctionType functionType = PyFunctionType.register(functionUnit.getName(), functionUnit);
			Expression target = new PythonScopedAttributeAccessRef(initCFG, PySyntheticLocation.INSTANCE, module,
					new Global(PySyntheticLocation.INSTANCE, module, mtd.getName(), false));
			PyAssign funcAssign = new PyAssign(initCFG, PySyntheticLocation.INSTANCE, target,
					new ImportFunction(initCFG, PySyntheticLocation.INSTANCE, functionUnit.getName(), functionUnit));
			initCFG.addNode(funcAssign, e == null);
			if (e != null) {
				initCFG.addEdge(new SequentialEdge(e, funcAssign));
			}
			e = funcAssign;
		}
		// module-level fields with an initial value are set, as the module
		// body sets them in Python
		for (Field fld : fields) {
			if (fld.isInstance() || fld.getInitialValue() == null)
				continue;
			Expression target = new PythonScopedAttributeAccessRef(initCFG, PySyntheticLocation.INSTANCE, module,
					new Global(PySyntheticLocation.INSTANCE, module, fld.getName(), false));
			PyAssign fieldAssign = new PyAssign(initCFG, PySyntheticLocation.INSTANCE, target,
					fld.getInitialValue().toLiSAExpression(initCFG));
			initCFG.addNode(fieldAssign, e == null);
			if (e != null)
				initCFG.addEdge(new SequentialEdge(e, fieldAssign));
			e = fieldAssign;
		}
		if (e != null) {
			Ret ret = new Ret(initCFG, PySyntheticLocation.INSTANCE);
			initCFG.addNode(ret);
			initCFG.addEdge(new SequentialEdge(e, ret));
		} else {
			Ret ret = new Ret(initCFG, PySyntheticLocation.INSTANCE);
			initCFG.addNode(ret, true);
		}
		return initCFG;
	}

	public void populateUnit(
			CFG init,
			it.unive.lisa.program.CompilationUnit root,
			CodeUnit lib) {
		CodeLocation location = new SourceCodeLocation(this.location, 0, 0);

		for (Method mtd : this.methods) {
			NativeCFG construct = mtd.toLiSACfg(location, init, lib, name);
			if (construct.getDescriptor().isInstance())
				throw new LibraryCreationException();
			lib.addCodeMember(construct);
		}

		for (Field fld : this.fields) {
			Global field = fld.toLiSAObject(location, lib);
			if (field.isInstance())
				throw new LibraryCreationException();
			lib.addGlobal(field);
		}

		for (ClassDef cls : this.classes)
			cls.populateUnit(location, init, root);
	}
}
