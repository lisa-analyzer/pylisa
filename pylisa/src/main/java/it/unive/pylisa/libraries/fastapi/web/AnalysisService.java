package it.unive.pylisa.libraries.fastapi.web;

import it.unive.lisa.LiSA;
import it.unive.lisa.analysis.SimpleAbstractState;
import it.unive.lisa.analysis.nonrelational.value.TypeEnvironment;
import it.unive.lisa.analysis.nonrelational.value.ValueEnvironment;
import it.unive.lisa.analysis.types.InferredTypes;
import it.unive.lisa.conf.LiSAConfiguration;
import it.unive.lisa.interprocedural.ReturnTopPolicy;
import it.unive.lisa.interprocedural.callgraph.RTACallGraph;
import it.unive.lisa.interprocedural.context.ContextBasedAnalysis;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFieldSensitivePointBasedHeap;
import it.unive.pylisa.PyFrontend;
import it.unive.pylisa.analysis.constants.ConstantPropagation;
import it.unive.pylisa.checks.FastApiSyntacticChecker;
import org.springframework.stereotype.Service;

import java.io.File;
import java.io.IOException;

@Service
public class AnalysisService {

    private final LiSAConfiguration conf;
    private final FastApiSyntacticChecker syntacticChecker = new FastApiSyntacticChecker();

    public AnalysisService() {

        LiSAConfiguration conf = new LiSAConfiguration();
        conf.workdir = "test-outputs/";
        conf.serializeResults = true;
        conf.jsonOutput = true;
        conf.analysisGraphs = LiSAConfiguration.GraphType.HTML_WITH_SUBNODES;
        conf.interproceduralAnalysis = new ContextBasedAnalysis<>();
        conf.callGraph = new RTACallGraph();
        conf.openCallPolicy = ReturnTopPolicy.INSTANCE;
        conf.optimize = false;
        conf.syntacticChecks.add(this.syntacticChecker);

        PyFieldSensitivePointBasedHeap heap = (PyFieldSensitivePointBasedHeap) new PyFieldSensitivePointBasedHeap().bottom();
        TypeEnvironment<InferredTypes> type = new TypeEnvironment<>(new InferredTypes());
        ValueEnvironment<ConstantPropagation> domain = new ValueEnvironment<>(new ConstantPropagation());
        conf.abstractState = new SimpleAbstractState<>(heap, domain, type);

        this.conf = conf;
    }

    public FastApiSyntacticChecker doSyntacticCheck() throws IOException {

        File folder = new File("pylisa/py-testcases/microservices/uploded-from-endpoint/");
        File[] files = folder.listFiles();

        if (files != null) {

            for (File file : files) {

                PyFrontend frontend = new PyFrontend(file.getPath(), false);
                LiSA lisa = new LiSA(this.conf);
                Program program2 = frontend.toLiSAProgram();
                lisa.run(program2);
            }

        } else {
            System.err.println("Folder does not exist or is not a directory");
        }

        return syntacticChecker;
    }
}
