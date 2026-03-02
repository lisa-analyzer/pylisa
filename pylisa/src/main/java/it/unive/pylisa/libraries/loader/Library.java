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
import it.unive.pylisa.cfg.statement.PythonUnitAttributeAccessRef;
import it.unive.pylisa.cfg.type.*;
import it.unive.pylisa.libraries.LibrarySpecificationParser.LibraryCreationException;
import it.unive.pylisa.program.ModuleUnit;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;

public class Library {
	private final String name;
	private final String location;
	private final Collection<Method> methods = new HashSet<>();
	private final Collection<Field> fields = new HashSet<>();
	private final Collection<ClassDef> classes = new ArrayList<>();

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
		for (ClassDef cls : getClasses()) {
			// Use the class type itself, not an instance
			ClassUnit lisaClassUnit = cls.toLiSAClassUnit(program, init);
			PyClassType classType = PyClassType.register(lisaClassUnit.getName(), lisaClassUnit);
			Expression target = new PythonUnitAttributeAccessRef(initCFG, SyntheticLocation.INSTANCE, module,
					new Global(SyntheticLocation.INSTANCE, module, cls.getName(), false));

			Assignment classVarAssign = new PyAssign(initCFG, SyntheticLocation.INSTANCE, target,
					new ImportClass(initCFG, SyntheticLocation.INSTANCE, cls.getName(), lisaClassUnit));
			initCFG.addNode(classVarAssign, e == null);
			if (e != null) {
				initCFG.addEdge(new SequentialEdge(e, classVarAssign));
			}
			e = classVarAssign;
		}
		// Assign functions
		/*
		 * for (Method mtd : lib.getMethods()) { SymbolicExpression funcExpr =
		 * new PyFunctionObject(mtd); // symbolic function object Assignment
		 * assign = new Assignment(initCFG, location, new MemberAccess(selfRef,
		 * mtd.getName()), funcExpr); initCFG.addNode(assign); }
		 */

		// Assign fields / constants
		/*
		 * for (Field fld : lib.getFields()) { SymbolicExpression fieldExpr =
		 * createFieldExpression(fld); // symbolic constant Assignment assign =
		 * new Assignment(initCFG, location, new MemberAccess(selfRef,
		 * fld.getName()), fieldExpr); initCFG.addNode(assign); }
		 */
		if (e != null) {
			Ret ret = new Ret(initCFG, SyntheticLocation.INSTANCE);
			initCFG.addNode(ret);
			initCFG.addEdge(new SequentialEdge(e, ret));
		} else {
			Ret ret = new Ret(initCFG, SyntheticLocation.INSTANCE);
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
			NativeCFG construct = mtd.toLiSACfg(location, init, lib);
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
