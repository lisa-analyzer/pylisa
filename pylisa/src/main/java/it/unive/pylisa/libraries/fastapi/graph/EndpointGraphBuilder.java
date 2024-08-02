package it.unive.pylisa.libraries.fastapi.graph;

import static guru.nidi.graphviz.attribute.Color.GREY80;
import static guru.nidi.graphviz.attribute.Color.TRANSPARENT;
import static guru.nidi.graphviz.attribute.Color.WHITE;
import static guru.nidi.graphviz.attribute.GraphAttr.splines;
import static guru.nidi.graphviz.attribute.GraphAttr.SplineMode.ORTHO;
import static guru.nidi.graphviz.attribute.Rank.RankDir.LEFT_TO_RIGHT;
import static guru.nidi.graphviz.model.Factory.mutGraph;
import static guru.nidi.graphviz.model.Factory.mutNode;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.springframework.web.util.UriTemplate;
import org.thymeleaf.TemplateEngine;
import org.thymeleaf.context.Context;
import org.thymeleaf.templatemode.TemplateMode;
import org.thymeleaf.templateresolver.StringTemplateResolver;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import guru.nidi.graphviz.attribute.Color;
import guru.nidi.graphviz.attribute.Font;
import guru.nidi.graphviz.attribute.Image;
import guru.nidi.graphviz.attribute.Label;
import guru.nidi.graphviz.attribute.Rank;
import guru.nidi.graphviz.attribute.Size;
import guru.nidi.graphviz.attribute.Style;
import guru.nidi.graphviz.engine.Format;
import guru.nidi.graphviz.engine.Graphviz;
import guru.nidi.graphviz.model.MutableGraph;
import guru.nidi.graphviz.model.MutableNode;
import it.unive.pylisa.libraries.fastapi.analysis.syntax.EndpointService;
import it.unive.pylisa.libraries.fastapi.definitions.Endpoint;
import it.unive.pylisa.libraries.fastapi.definitions.GroupBy;
import it.unive.pylisa.libraries.fastapi.definitions.Role;
import it.unive.pylisa.libraries.fastapi.helpers.TextHelper;

public class EndpointGraphBuilder {

    private final Random rand = new Random();
    private final ObjectMapper mapper = new ObjectMapper();

    private MutableGraph base;

    private Map<String, List<Endpoint>> grouped;
    private List<Endpoint> providers;
    private List<Endpoint> consumers;

    private final String emptyCallImg = EndpointGraphBuilder.class.getResource("/assets/cross-mark.png").getFile();

    private Color randomColor() {
        int r = 128 + rand.nextInt(128);
        int g = 128 + rand.nextInt(128);
        int b = 128 + rand.nextInt(128);
        return Color.rgb(r, g, b);
    }

    public void build(List<Endpoint> endpoints, String basePathNoExt) throws IOException {

        this.buildBase(endpoints);

        for (Map.Entry<String, List<Endpoint>> group : grouped.entrySet()) {

            List<MutableNode> nodes = buildNodes(group);

            MutableGraph cluster = this.buildClusterBase(group.getKey());
            cluster.add(nodes);

            base.add(cluster);
        }

        this.connectNodes();
        this.pointEmptyConsumers();

        this.exportToPNG(basePathNoExt + ".png");
        this.exportToDOT(basePathNoExt + ".dot");
        this.exportToHTML(basePathNoExt + ".html");
    }

    private void buildBase(List<Endpoint> endpoints) {

        this.grouped = EndpointService.groupEndpoints(endpoints, GroupBy.BELONGS);
        this.providers = EndpointService.groupEndpoints(endpoints, GroupBy.ROLE_PROVIDER).values().stream().flatMap(Collection::stream).toList();
        this.consumers = EndpointService.groupEndpoints(endpoints, GroupBy.ROLE_CONSUMER).values().stream().flatMap(Collection::stream).toList();

        MutableGraph base = mutGraph("microservice_graph").setDirected(true);

        base.graphAttrs().add(splines(ORTHO), WHITE.gradient(WHITE).background().angle(90));

        this.base = base;
    }

    private MutableGraph buildClusterBase(String clusterName) {

        MutableGraph cluster = mutGraph().setDirected(true).setCluster(true).setName(clusterName).graphAttrs().add(Rank.dir(LEFT_TO_RIGHT));

        cluster.graphAttrs().add(Style.ROUNDED);
        cluster.graphAttrs().add(Color.BLACK, Label.of(clusterName));
        cluster.graphAttrs().add(Font.config("Helvetica-bold", 24));
        cluster.graphAttrs().add(splines(ORTHO), GREY80.gradient(randomColor()).background().angle(90));

        return cluster;
    }

    private MutableNode buildConsumerNode(Endpoint endpoint) throws JsonProcessingException {

        String label = endpoint.getMethod() + " " + endpoint.getFullPath();
        String prettyLabel = "<b>" + endpoint.getMethod() + "</b><br/>" + endpoint.getFullPath();

        MutableNode consumer = mutNode(label).setName(label).add(Label.html(prettyLabel), Color.RED3, Font.config("Helvetica", 14), WHITE.font(), Style.FILLED);

        String json = mapper.writeValueAsString(endpoint);
        consumer.add("tooltip", json);

        return consumer;
    }

    private MutableNode buildProviderNode(Endpoint endpoint) throws JsonProcessingException {

        String label = endpoint.getMethod() + " " + endpoint.getFullPath();
        String prettyLabel = "<b>" + endpoint.getMethod() + "</b><br/>" + endpoint.getFullPath();

        MutableNode provider = mutNode(label).setName(label).add(Label.html(prettyLabel), Color.LIGHTBLUE, Font.config("Helvetica", 14), Style.FILLED);

        String json = mapper.writeValueAsString(endpoint);
        provider.add("tooltip", json);

        return provider;
    }

    private MutableNode buildPathlessNode(Endpoint endpoint) {

        String label = endpoint.getMethod().toString();
        String prettyLabel = "<b>" + endpoint.getMethod() + "</b><br/>" + "This endpoint was left without path definition";

        MutableNode empty = mutNode(label).setName(label).add(Label.html(prettyLabel), Color.LIGHTGRAY.fill(), Font.config("Helvetica", 14), Style.DOTTED);

        return empty;
    }

    private List<MutableNode> buildNodes(Map.Entry<String, List<Endpoint>> group) throws JsonProcessingException {

        List<MutableNode> nodes = new ArrayList<>();

        for (Endpoint endpoint : group.getValue()) {

            if (endpoint.getRole() == Role.CONSUMER) {
                MutableNode consumer = buildConsumerNode(endpoint);
                nodes.add(consumer);

            } else if (endpoint.getFullPath() == null) {
                MutableNode empty = buildPathlessNode(endpoint);
                nodes.add(empty);

            } else {
                MutableNode provider = buildProviderNode(endpoint);
                nodes.add(provider);
            }
        }

        return nodes;
    }

    private void connectNodes() {

        for (Endpoint provider : providers) {
            for (Endpoint consumer : consumers) {

                String providerPath = provider.getFullPath();
                String consumerPath = consumer.getFullPath();

                String providerLabel = provider.getMethod() + " " + providerPath;
                String consumerLabel = consumer.getMethod() + " " + consumerPath;

                if (providerPath != null) {
                    UriTemplate template = new UriTemplate(providerPath);

                    Map<String, String> variables = template.match(consumerPath);

                    if (!variables.isEmpty()) {

                        MutableNode providerNode = mutNode(providerLabel);
                        MutableNode consumerNode = mutNode(consumerLabel);
                        base.add(consumerNode.addLink(providerNode));
                    }
                }
            }
        }
    }

    private void pointEmptyConsumers() {

        List<String> connectedConsumers = base.edges().stream().map(edge -> edge.from().name().toString()).toList();

        for (Endpoint consumer : consumers) {

            String label = consumer.getMethod() + " " + consumer.getFullPath();

            if (!connectedConsumers.contains(label)) {

                MutableNode from = mutNode(label);

                String id = String.valueOf(rand.nextInt());
                MutableNode to = mutNode(" ").setName(id).add(TRANSPARENT.font(), Color.TRANSPARENT, Size.std().margin(0, 0), Image.of(emptyCallImg));
                base.add(from.addLink(to));
            }
        }
    }

    private void exportToHTML(String path) throws IOException {
        String exportedGraph = Graphviz.fromGraph(base).render(Format.DOT).toString();

        TemplateEngine templateEngine = new TemplateEngine();
        StringTemplateResolver resolver = new StringTemplateResolver();
        resolver.setTemplateMode(TemplateMode.HTML);
        templateEngine.setTemplateResolver(resolver);

        String htmlTemplate = TextHelper.loadResourceTemplate("/templates/microservice-graph.html");

        Context context = new Context();
        context.setVariable("dotContent", exportedGraph);
        context.setVariable("emptyCallImg", emptyCallImg);

        String html = templateEngine.process(htmlTemplate, context);

        Path outputPath = Path.of(path);
        Files.writeString(outputPath, html);
    }

    private void exportToDOT(String path) throws IOException {
        Graphviz.fromGraph(base).render(Format.DOT).toFile(new File(path));
    }

    private void exportToPNG(String path) throws IOException {
        Graphviz.fromGraph(base).height(300).render(Format.PNG).toFile(new File(path));
    }
}
