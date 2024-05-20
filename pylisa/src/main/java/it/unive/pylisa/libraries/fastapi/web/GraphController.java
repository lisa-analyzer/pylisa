package it.unive.pylisa.libraries.fastapi.web;

import it.unive.pylisa.checks.FastApiSyntacticChecker;
import it.unive.pylisa.libraries.fastapi.graph.GraphServiceForWeb;
import jakarta.servlet.http.HttpServletResponse;
import jakarta.servlet.http.HttpSession;
import org.springframework.http.HttpStatus;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.stereotype.Controller;
import org.springframework.ui.Model;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PathVariable;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.multipart.MultipartFile;
import org.springframework.web.multipart.MultipartHttpServletRequest;
import org.springframework.web.servlet.mvc.support.RedirectAttributes;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Map;

@Controller
public class GraphController {

    private final AnalysisService analysisService;
    private final GraphServiceForWeb graphServiceForWeb;

    private FastApiSyntacticChecker syntacticChecker;

    private static final String STORAGE_FOLDER = "pylisa/py-testcases/microservices/uploded-from-endpoint/";

    public GraphController(AnalysisService analysisService, GraphServiceForWeb graphServiceForWeb) {
        this.analysisService = analysisService;
        this.graphServiceForWeb = graphServiceForWeb;
    }

    @GetMapping("/")
    public String index() {
        return "index";
    }

    @GetMapping(path = "/loadingscreen")
    public String loadingScreen() {
        return "loadingscreen";
    }

    @PostMapping(path = "/upload", consumes = MediaType.MULTIPART_FORM_DATA_VALUE)
    public ResponseEntity<String> upload(MultipartHttpServletRequest request, RedirectAttributes redirectAttributes) throws IOException {

        Map<String, MultipartFile> files = request.getFileMap();
        for (MultipartFile file : files.values()) {

            if (!file.isEmpty()) {

                try {
                    byte[] bytes = file.getBytes();
                    Path path = Paths.get(STORAGE_FOLDER + file.getOriginalFilename());
                    Files.write(path, bytes);

                } catch (Exception e) {
                    redirectAttributes.addFlashAttribute("message", "Could not upload file: "
                            + file.getOriginalFilename());
                    return ResponseEntity.status(HttpStatus.BAD_REQUEST).body("Upload failed");
                }
            }
        }

        this.syntacticChecker = analysisService.doSyntacticCheck();
        return ResponseEntity.ok("Upload successful");
    }

    @GetMapping(path ="/graph", produces = MediaType.TEXT_HTML_VALUE)
    public String initialGraph(Model model, HttpSession session) throws IOException {

        String clustername = null;
        if (session.getAttribute("cluster") != null) {
             clustername = session.getAttribute("cluster").toString();
        }

        if (clustername == null) {
            List<String> clusterOptions = this.graphServiceForWeb.getClusterOptions(this.syntacticChecker.endpoints);

            if (!clusterOptions.isEmpty()) {
                clustername = clusterOptions.get(0);

            } else {
                throw new RuntimeException("Nothing to provide");
            }
        }

        String dot = graphServiceForWeb.buildDot(clustername, this.syntacticChecker.endpoints);

        model.addAttribute("dotContent", dot);
        model.addAttribute("emptyCallImg", "pylisa/src/main/resources/static/cross-mark.png");
        return "microservice-graph";
    }

    @GetMapping(path = "/cluster/{name}")
    public void cluster(@PathVariable("name") String name, HttpSession session, HttpServletResponse response) throws IOException {

        if (name == null) {
            List<String> clusterOptions = this.graphServiceForWeb.getClusterOptions(this.syntacticChecker.endpoints);

            if (!clusterOptions.isEmpty()) {
                name = clusterOptions.get(0);

            } else {
                throw new RuntimeException("Nothing to provide");
            }
        }

        session.setAttribute("cluster", name);
        response.setStatus(HttpServletResponse.SC_FOUND);
        response.sendRedirect("/graph");
    }
}
