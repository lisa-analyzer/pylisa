package it.unive.pylisa.cfg.statement;

import it.unive.lisa.analysis.*;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.edge.Edge;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.util.datastructures.graph.GraphVisitor;
import it.unive.pylisa.analysis.ObjectRegister;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.program.ModuleUnit;
import java.util.List;
import org.apache.commons.lang3.tuple.Pair;

public class FromImportClasses extends Expression {

	private final ModuleUnit currentModule;
	private final List<Pair<String, CompilationUnit>> classImports;
	private final ModuleUnit sourceModule;

	public FromImportClasses(
			CFG cfg,
			CodeLocation location,
			ModuleUnit currentModule,
			List<Pair<String, CompilationUnit>> classImports,
			ModuleUnit sourceModule) {
		super(cfg, location);
		this.currentModule = currentModule;
		this.classImports = classImports;
		this.sourceModule = sourceModule;
	}

	@Override
	public String toString() {
		return "from import classes: " + classImports.stream()
				.map(Pair::getLeft)
				.reduce((
						a,
						b) -> a + ", " + b)
				.orElse("");
	}

	@Override
	public <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> forwardSemantics(
			AnalysisState<A> entryState,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		AnalysisState<A> state = entryState;
		if (sourceModule != null)
			state = ObjectRegister.initialize(state, this, sourceModule, interprocedural);
		for (Pair<String, CompilationUnit> entry : classImports) {
			String shortName = entry.getLeft();
			CompilationUnit classUnit = entry.getRight();
			Expression target;
			if (currentModule != null) {
				target = new PythonUnitAttributeAccessRef(getCFG(), getLocation(),
						currentModule, new Global(getLocation(), currentModule, shortName, false));
			} else {
				target = new VariableRef(getCFG(), getLocation(), shortName);
			}
			Expression value = new ClassLiteral(getCFG(), getLocation(), classUnit);
			PyAssign assign = new PyAssign(getCFG(), getLocation(), target, value);
			state = assign.forwardSemantics(state, interprocedural, expressions);
		}
		return state;
	}

	@Override
	protected int compareSameClass(
			Statement o) {
		return 0;
	}

	@Override
	public <V> boolean accept(
			GraphVisitor<CFG, Statement, Edge, V> visitor,
			V tool) {
		return visitor.visit(tool, getCFG(), this);
	}
}
