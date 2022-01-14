package it.unive.pylisa.cfg.type;


import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.UnitType;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.PythonUnit;

public class PyLibraryType implements UnitType {
	private static CompilationUnit emptyUnit = new PythonUnit(new SourceCodeLocation("", 0, 0), "", true);
	private final String name;

	private final static Set<Type> instances = new HashSet<>();

	private final static Map<String, CompilationUnit> units = new HashMap<>();

	public PyLibraryType(String name) {
		this.name = name;
		instances.add(this);
	}

	public static void addUnit(CompilationUnit unit1) {
		units.put(unit1.getName(), unit1);
	}

	@Override
	public String toString() {
		return "Library<"+name+">";
	}
	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof PyLibraryType|| other instanceof Untyped;
	}

	@Override
	public Type commonSupertype(Type other) {
		if(other==this)
			return this;
		else return  Untyped.INSTANCE;
	}

	@Override
	public Collection<Type> allInstances() {
		return instances;
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;
		PyLibraryType that = (PyLibraryType) o;
		return Objects.equals(name, that.name);
	}

	@Override
	public int hashCode() {
		return Objects.hash(name);
	}

	@Override
	public CompilationUnit getUnit() {
		CompilationUnit res = units.get(this.name);
		if(res==null)
			return PyLibraryType.emptyUnit;//workaround to have a unit when I do not have manual specification of such unit
		else return res;
	}
}
