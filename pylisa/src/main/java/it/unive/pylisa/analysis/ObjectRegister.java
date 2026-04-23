package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.symbols.NameSymbol;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.lattices.ExpressionSet;
import it.unive.lisa.lattices.FunctionalLattice;
import it.unive.lisa.lattices.string.StringConstant;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.Global;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.VariableRef;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.pylisa.cfg.expression.PyAssign;
import it.unive.pylisa.cfg.statement.ModuleLiteral;
import it.unive.pylisa.cfg.statement.PythonScopedAttributeAccessRef;
import it.unive.pylisa.program.FunctionUnit;
import it.unive.pylisa.program.ModuleUnit;
import java.util.Map;

/**
 * A {@link FunctionalLattice} mapping module names (Strings) to
 * {@link StringConstant} objects, representing state identifier. This lattice
 * can be stored in {@link it.unive.lisa.analysis.FixpointInfo} using key
 * {@value #INFO_KEY}. It models Python's sys.modules dictionary in the abstract
 * interpreter.
 */
public class ObjectRegister extends FunctionalLattice<ObjectRegister, String, StringConstant> {

	/**
	 * Key to retrieve this lattice from FixpointInfo.
	 */
	public static final String INFO_KEY = "object-register";

	/**
	 * Builds an empty module mapping.
	 */
	public ObjectRegister() {
		super(StringConstant.BOTTOM);
	}

	private ObjectRegister(
			StringConstant lattice,
			Map<String, StringConstant> function) {
		super(lattice, function);
	}

	@Override
	public StringConstant stateOfUnknown(
			String key) {
		return StringConstant.BOTTOM;
	}

	/**
	 * Registers a module in the abstract sys.modules. Any previous binding will
	 * be overwritten.
	 *
	 * @param name            the module name
	 * @param stateIdentifier the PythonUnit representing the module
	 *
	 * @return a copy of this lattice with the new binding
	 */
	public ObjectRegister putModule(
			String name,
			StringConstant stateIdentifier) {
		ObjectRegister result = super.putState(name, stateIdentifier);
		int sz = result.getMap() == null ? 0 : result.getMap().size();
		int callN = PUT_CALL_COUNT.incrementAndGet();
		if (callN % 2000 == 0 || (sz > 0 && sz % 1000 == 0 && sz != LAST_SIZE.getAndSet(sz)))
			LOG.info("[OR-PUT] putModule call#{} mapSize={} key={} value={}", callN, sz, name, stateIdentifier.value);
		return result;
	}

	public java.util.Map<String, StringConstant> getMap() {
		return function;
	}

	private static final java.util.concurrent.atomic.AtomicInteger PUT_CALL_COUNT = new java.util.concurrent.atomic.AtomicInteger(
			0);
	private static final java.util.concurrent.atomic.AtomicInteger LAST_SIZE = new java.util.concurrent.atomic.AtomicInteger(
			0);

	@Override
	public ObjectRegister top() {
		return new ObjectRegister(lattice.top(), null);
	}

	@Override
	public ObjectRegister bottom() {
		return new ObjectRegister(lattice.bottom(), null);
	}

	@Override
	public ObjectRegister mk(
			StringConstant lattice,
			Map<String, StringConstant> function) {
		return new ObjectRegister(lattice, function);
	}

	private static final org.apache.logging.log4j.Logger LOG = org.apache.logging.log4j.LogManager
			.getLogger(ObjectRegister.class);
	private static final java.util.concurrent.atomic.AtomicInteger INIT_CALL_COUNT = new java.util.concurrent.atomic.AtomicInteger(
			0);

	public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> initialize(
			AnalysisState<A> state,
			Statement init,
			CompilationUnit compilationUnit,
			InterproceduralAnalysis<A, D> interprocedural,
			StatementStore<A> expressions)
			throws SemanticException {
		// If the incoming state is already bottom (e.g. a prior unsupported
		// statement
		// propagated bottom to this import), do not attempt registration or
		// re-run $init.
		// Without this guard, an import statement receiving bottom would
		// re-trigger $init
		// every fixpoint iteration, creating an infinite loop.
		if (state.isBottom())
			return state;
		int callN = INIT_CALL_COUNT.incrementAndGet();
		if (callN % 200 == 0)
			LOG.info(
					"[OR-TRACK] ObjectRegister.initialize call#{} module={} UAT-global={}",
					callN, compilationUnit.getName(),
					it.unive.pylisa.program.type.UnknownAttributeType.getRegisteredCount());
		if (state.getExecutionInfo(SymbolAliasing.INFO_KEY) == null)
			state = state.storeExecutionInfo(SymbolAliasing.INFO_KEY, new SymbolAliasing());

		state = state.storeExecutionInfo(SymbolAliasing.INFO_KEY,
				state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class).alias(
						new NameSymbol(compilationUnit.getName()),
						new NameSymbol(compilationUnit.getName())));

		if (state.getExecutionInfo(ObjectRegister.INFO_KEY) == null)
			state = state.storeExecutionInfo(ObjectRegister.INFO_KEY, new ObjectRegister());

		if (state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class).getState(compilationUnit.getName())
				.isBottom()) {
			state = state.storeExecutionInfo(ObjectRegister.INFO_KEY,
					state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class)
							.putModule(compilationUnit.getName(), new StringConstant("$" + compilationUnit)));
			if (compilationUnit instanceof FunctionUnit) {
				return state;
			}

			UnresolvedCall moduleInit = new UnresolvedCall(
					init.getCFG(),
					init.getLocation(),
					Call.CallType.STATIC,
					compilationUnit.getName(),
					"$init");
			// Save the state that already contains the registration so we can
			// restore it if $init returns bottom. Without this guard the
			// registration is overwritten by the bottom state, causing the
			// module
			// to appear unregistered on the next fixpoint iteration and
			// triggering
			// an infinite re-import loop.
			AnalysisState<A> stateWithRegistration = state;
			state = moduleInit.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],
					new StatementStore<>(state.bottom()));
			if (state.isBottom()) {
				// $init returned bottom (e.g. an unsupported statement inside
				// the
				// module body propagated bottom). Fall back to the pre-$init
				// state
				// so that the registration is preserved across fixpoint
				// iterations.
				return stateWithRegistration;
			}
			VariableRef v = new VariableRef(init.getCFG(), init.getLocation(), "$" + compilationUnit.getName());
			if (compilationUnit instanceof ModuleUnit) {
				PythonScopedAttributeAccessRef access = new PythonScopedAttributeAccessRef(init.getCFG(),
						init.getLocation(), compilationUnit,
						new Global(init.getLocation(), compilationUnit, "__name__", false));
				PyAssign assign = new PyAssign(init.getCFG(), init.getLocation(), v,
						new ModuleLiteral(init.getCFG(), init.getLocation(), compilationUnit));
				state = assign.forwardSemantics(state, interprocedural, expressions);
			}
			return state;

		}

		return state;
	}

}