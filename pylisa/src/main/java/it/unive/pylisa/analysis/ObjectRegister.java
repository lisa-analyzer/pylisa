package it.unive.pylisa.analysis;

import it.unive.lisa.analysis.*;
import it.unive.lisa.analysis.lattices.ExpressionSet;
import it.unive.lisa.analysis.lattices.FunctionalLattice;
import it.unive.lisa.analysis.symbols.NameSymbol;
import it.unive.lisa.analysis.symbols.QualifiedNameSymbol;
import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.InterproceduralAnalysis;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.statement.Statement;
import it.unive.lisa.program.cfg.statement.call.Call;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.lattices.string.StringConstant;
import it.unive.pylisa.program.FunctionUnit;

import java.util.Map;

/**
 * A {@link FunctionalLattice} mapping module names (Strings) to
 * {@link StringConstant} objects, representing state identifier.
 * This lattice can be stored in {@link it.unive.lisa.analysis.FixpointInfo} using key {@value #INFO_KEY}.
 *
 * It models Python's sys.modules dictionary in the abstract interpreter.
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

    private ObjectRegister(StringConstant lattice, Map<String, StringConstant> function) {
        super(lattice, function);
    }

    @Override
    public StringConstant stateOfUnknown(String key) {
        return StringConstant.BOTTOM;
    }


    /**
     * Registers a module in the abstract sys.modules.
     * Any previous binding will be overwritten.
     *
     * @param name  the module name
     * @param stateIdentifier the PythonUnit representing the module
     * @return a copy of this lattice with the new binding
     */
    public ObjectRegister putModule(String name, StringConstant stateIdentifier) {
        return super.putState(name, stateIdentifier);
    }

    @Override
    public ObjectRegister top() {
        return new ObjectRegister(lattice.top(), null);
    }

    @Override
    public ObjectRegister bottom() {
        return new ObjectRegister(lattice.bottom(), null);
    }

    @Override
    public ObjectRegister mk(StringConstant lattice, Map<String, StringConstant> function) {
        return new ObjectRegister(lattice, function);
    }

    public static <A extends AbstractLattice<A>, D extends AbstractDomain<A>> AnalysisState<A> initialize(
            AnalysisState<A> state,
            Statement init,
            CompilationUnit compilationUnit,
            InterproceduralAnalysis<A, D> interprocedural)
            throws SemanticException {
        if (state.getExecutionInfo(SymbolAliasing.INFO_KEY) == null)
            state = state.storeExecutionInfo(SymbolAliasing.INFO_KEY, new SymbolAliasing());

        state = state.storeExecutionInfo(SymbolAliasing.INFO_KEY,
                state.getExecutionInfo(SymbolAliasing.INFO_KEY, SymbolAliasing.class).alias(
                        new NameSymbol(compilationUnit.getName()),
                        new NameSymbol(compilationUnit.getName())));

        if (state.getExecutionInfo(ObjectRegister.INFO_KEY) == null)
            state = state.storeExecutionInfo(ObjectRegister.INFO_KEY, new ObjectRegister());

        if (state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class).getState(compilationUnit.getName()).isBottom()) {
            state = state.storeExecutionInfo(ObjectRegister.INFO_KEY, state.getExecutionInfo(ObjectRegister.INFO_KEY, ObjectRegister.class)
                    .putModule(compilationUnit.getName(), new StringConstant("$" + compilationUnit)));
            if (compilationUnit instanceof FunctionUnit) {
                return state;
            }

            UnresolvedCall moduleInit = new UnresolvedCall(
                    init.getCFG(),
                    init.getLocation(),
                    Call.CallType.STATIC,
                    compilationUnit.getName(),
                    "$init"
            );
            return moduleInit.forwardSemanticsAux(interprocedural, state, new ExpressionSet[0],new StatementStore<>(state.bottom()));
        }
        // if needed, calling the class initializer (if the class has one)

        //Collection<CodeMember> target = PyModuleType.lookup(className).getUnit().getCodeMembersByName(name);
        // we perform the call if (i) the clinit exits, (ii) we are not already
        // executing
        // it (to avoid recursion) and (iii) it has not been executed yet
        // condition (ii) might happen if the state goes to bottom
        // within the clinit, and we lose the information that we are executing
        // it
        /*if (!target.isEmpty() && target.iterator().next() != init.getCFG() && !info.contains(className)) {
            UnresolvedCall clinit = new UnresolvedCall(
                    init.getCFG(),
                    init.getLocation(),
                    Call.CallType.STATIC,
                    className,
                    name,
                    new Expression[0]);

            result = state.storeExecutionInfo(InitializedClassSet.INFO_KEY, info.add(className));
            result.withExecutionExpression(new Skip(init.getLocation()));
            result = clinit.forwardSemanticsAux(interprocedural, result, new ExpressionSet[0],
                    new StatementStore<>(result.bottom()));
        }
*/
        return state;
    }

}