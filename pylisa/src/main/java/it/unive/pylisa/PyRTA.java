package it.unive.pylisa;

import it.unive.lisa.analysis.symbols.SymbolAliasing;
import it.unive.lisa.interprocedural.callgraph.BaseCallGraph;
import it.unive.lisa.interprocedural.callgraph.CallResolutionException;
import it.unive.lisa.program.CompilationUnit;
import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeMember;
import it.unive.lisa.program.cfg.NativeCFG;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.cfg.statement.call.UnresolvedCall;
import it.unive.lisa.type.Type;
import it.unive.lisa.type.UnitType;
import it.unive.lisa.util.collections.externalSet.ExternalSet;
import java.util.Collection;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

public class PyRTA extends BaseCallGraph {

	@Override
	protected Collection<Type> getPossibleTypesOfReceiver(Expression receiver, ExternalSet<Type> types)
			throws CallResolutionException {
		return types;
	}

	@Override
	protected void resolveInstance(
			UnresolvedCall call,
			ExternalSet<Type>[] types,
			Collection<CFG> targets,
			Collection<NativeCFG> natives,
			SymbolAliasing aliasing)
			throws CallResolutionException {
		// FIXME temporary patch to https://github.com/UniVE-SSV/lisa/issues/212
		if (call.getParameters().length == 0)
			throw new CallResolutionException(
					"An instance call should have at least one parameter to be used as the receiver of the call");
		Expression receiver = call.getParameters()[0];
		for (Type recType : getPossibleTypesOfReceiver(receiver, types[0])) {
			Collection<CompilationUnit> units;
			if (recType.isUnitType())
				units = Collections.singleton(recType.asUnitType().getUnit());
			else if (recType.isPointerType() && recType.asPointerType().getInnerTypes().anyMatch(Type::isUnitType))
				units = recType.asPointerType()
						.getInnerTypes()
						.stream()
						.filter(Type::isUnitType)
						.map(Type::asUnitType)
						.map(UnitType::getUnit)
						.collect(Collectors.toSet());
			else
				continue;

			Set<CompilationUnit> seen = new HashSet<>();
			for (CompilationUnit unit : units)
				for (CompilationUnit cu : call.getTraversalStrategy().traverse(call, unit))
					if (seen.add(cu))
						// we inspect only the ones of the current unit
						for (CodeMember cm : cu.getInstanceCodeMembers(false))
							checkMember(call, types, targets, natives, aliasing, cm, true);
		}
	}

}
