package it.unive.pylisa.libraries.loader;

import it.unive.lisa.program.*;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.CodeMemberDescriptor;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.edge.SequentialEdge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Ret;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.pylisa.cfg.PyCFG;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.statement.ImportFunction;
import it.unive.pylisa.cfg.statement.PythonScopedAttributeAccessRef;
import it.unive.pylisa.cfg.type.PyClassType;
import it.unive.pylisa.program.FunctionUnit;
import java.util.Collection;
import java.util.HashSet;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;

public class ClassDef {
	private final boolean root;
	private final boolean sealed;
	private final String typeName;
	private final String name;
	private final String base;
	private final Collection<Method> methods = new HashSet<>();
	private final Collection<Field> fields = new HashSet<>();
	private final Type reifiedTypeName;
	private final Library library;

	public ClassDef(
			boolean root,
			boolean sealed,
			Library library,
			String typeName,
			String name,
			String base,
			Type reifiedTypeName) {
		this.root = root;
		this.sealed = sealed;
		this.typeName = typeName;
		this.name = name;
		this.base = base;
		this.reifiedTypeName = reifiedTypeName;
		this.library = library;
	}

	public boolean isRoot() {
		return root;
	}

	public boolean isSealed() {
		return sealed;
	}

	public String getName() {
		return name;
	}

	public String getBase() {
		return base;
	}

	public String getTypeName() {
		return typeName;
	}

	public Collection<Method> getMethods() {
		return methods;
	}

	public Collection<Field> getFields() {
		return fields;
	}

	public Type getReifiedTypeName() {
		return reifiedTypeName;
	}

	@Override
	public int hashCode() {
		return Objects.hash(base, fields, methods, name, root, sealed, typeName);
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
		ClassDef other = (ClassDef) obj;
		return Objects.equals(base, other.base) && Objects.equals(fields, other.fields)
				&& Objects.equals(methods, other.methods) && Objects.equals(name, other.name) && root == other.root
				&& sealed == other.sealed && Objects.equals(typeName, other.typeName);
	}

	@Override
	public String toString() {
		return "ClassDef [root=" + root + ", sealed=" + sealed + ", typeName=" + typeName + ", name=" + name + ", base="
				+ base + ", methods=" + methods + ", fields=" + fields + "]";
	}

	public ClassUnit toLiSAUnit(
			CodeLocation location,
			Program program,
			AtomicReference<CompilationUnit> rootHolder) {
		ClassUnit unit = new ClassUnit(location, program, library.getName() + "." + getName(), this.sealed);
		if (this.root)
			if (rootHolder.get() != null)
				throw new IllegalStateException("More than one root class defined as hierarchy root");
			else
				rootHolder.set(unit);
		return unit;
	}

	public ClassUnit toLiSAClassUnit(
			Program program,
			CFG init) {
		ClassUnit unit = new ClassUnit(SyntheticLocation.INSTANCE, program, library.getName() + "." + getName(),
				this.sealed);

		PyClassType.register(unit.getName(), unit);
		program.addUnit(unit);
		PyCFG classInitCFG = new PyCFG(new CodeMemberDescriptor(SyntheticLocation.INSTANCE, unit, false, "$init"));

		unit.addCodeMember(classInitCFG);

		Statement first = null;
		Statement last = null;
		// todo add ancestors root
		if (this.base != null)
			unit.addAncestor(PyClassType.lookup(this.base).getUnit());
		for (Method mtd : this.methods) {
			FunctionUnit functionUnit = mtd.toLiSAFunctionUnit(SyntheticLocation.INSTANCE, init, program, unit);
			Expression target = new PythonScopedAttributeAccessRef(classInitCFG, SyntheticLocation.INSTANCE, unit,
					new Global(SyntheticLocation.INSTANCE, unit, mtd.getName(), false));
			PyAssign funcAssign = new PyAssign(classInitCFG, SyntheticLocation.INSTANCE, target,
					new ImportFunction(classInitCFG, SyntheticLocation.INSTANCE, unit.getName(), functionUnit));
			classInitCFG.addNode(funcAssign, first == null);
			if (first == null) {
				first = funcAssign;
			}
			if (last != null) {
				classInitCFG.addEdge(new SequentialEdge(last, funcAssign));
			}
			last = funcAssign;
			// FunctionUnit unit = new FunctionUnit(getLocation(ctx), program,
			// currentUnit + "." + ctx.NAME().getText());
			// classInitCFG.addNode();

		}
		Ret ret = new Ret(classInitCFG, SyntheticLocation.INSTANCE);
		classInitCFG.addNode(ret, first == null);
		if (last != null) {
			classInitCFG.addEdge(new SequentialEdge(last, ret));
		}
		return unit;
	}

	public ClassUnit populateUnit(
			CodeLocation location,
			CFG init,
			CompilationUnit root) {
		ClassUnit unit = (ClassUnit) PyClassType.lookup(this.name).getUnit();

		if (this.base != null)
			unit.addAncestor(PyClassType.lookup(this.base).getUnit());
		else if (root != null && unit != root)
			unit.addAncestor(root);

		for (Method mtd : this.methods) {
			NativeCFG construct = mtd.toLiSACfg(location, init, unit);
			if (construct.getDescriptor().isInstance())
				unit.addInstanceCodeMember(construct);
			else
				unit.addCodeMember(construct);
		}

		for (Field fld : this.fields) {
			Global field = fld.toLiSAObject(location, unit);
			if (field.isInstance())
				unit.addInstanceGlobal(field);
			else
				unit.addGlobal(field);
		}

		return unit;
	}
}
