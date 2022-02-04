package it.unive.pylisa.libraries.pandas;

import java.util.Collection;
import java.util.Set;

import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CFGDescriptor;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.Parameter;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.UnitType;
import it.unive.lisa.type.Untyped;
import it.unive.pylisa.cfg.PythonUnit;

public class PyDataframeType implements UnitType {
	/**
	 * The unique singleton instance of this type.
	 */
	public static final PyDataframeType INSTANCE = new PyDataframeType();

	private PyDataframeType() {
	}

	@Override
	public String toString() {
		return "T";
	}

	@Override
	public boolean equals(Object other) {
		return other instanceof PyDataframeType;
	}

	@Override
	public int hashCode() {
		return PyDataframeType.class.getName().hashCode();
	}

	@Override
	public boolean canBeAssignedTo(Type other) {
		return other instanceof PyDataframeType || other instanceof Untyped;
	}

	@Override
	public Type commonSupertype(Type other) {
		return this;
	}

	@Override
	public Collection<Type> allInstances() {
		return Set.of(INSTANCE);
	}

	@Override
	public CompilationUnit getUnit() {
		PythonUnit unit1 = new PythonUnit(new SourceCodeLocation("pandas.Dataframe", 0, 0), "pandas.Dataframe", true);
		unit1.addInstanceConstruct(new NativeCFG(
				new CFGDescriptor(new SourceCodeLocation("pandas.Dataframe", 0, 0),
						unit1,
						true,
						"head",
						new Parameter(new SourceCodeLocation("pandas.Dataframe", 0, 0), "arg1",
								PyDataframeType.INSTANCE)),
				Head.class));
		unit1.addInstanceConstruct(new NativeCFG(
				new CFGDescriptor(new SourceCodeLocation("pandas.Dataframe", 0, 0),
						unit1,
						true,
						"tail",
						new Parameter(new SourceCodeLocation("pandas.Dataframe", 0, 0), "arg1",
								PyDataframeType.INSTANCE)),
				Tail.class));
		unit1.addInstanceConstruct(new NativeCFG(
				new CFGDescriptor(new SourceCodeLocation("pandas.Dataframe", 0, 0),
						unit1,
						true,
						"info",
						new Parameter(new SourceCodeLocation("pandas.Dataframe", 0, 0), "arg1",
								PyDataframeType.INSTANCE)),
				Info.class));
		return unit1;
	}
}
