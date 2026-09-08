package it.unive.pylisa;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;

import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.DefaultParser;
import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;

public class StubbedMainForSVCOMP {

	public static void main(String[] args) throws Exception {

		Options options = new Options();
		options.addOption(new Option("h", "help", false, "Print this help message"));
		options.addOption(Option.builder("s").longOpt("source").hasArgs().build());
		options.addOption(Option.builder("o").longOpt("outdir").hasArg().build());
		options.addOption(Option.builder("l").longOpt("log-level").hasArg().build());
		options.addOption(Option.builder("c").longOpt("checker").hasArg().build());
		options.addOption(Option.builder("n").longOpt("numericalDomain").hasArg().build());
		options.addOption(Option.builder("m").longOpt("mode").hasArg().build());
		options.addOption(new Option("v", "version", false, "Version of the tool"));
		options.addOption(Option.builder().longOpt("no-html").build());
		options.addOption(new Option("e", "dump-exceptions", false, null));
		options.addOption(new Option("d", "debug-information", false, null));

		CommandLine cmd = new DefaultParser().parse(options, args);

		Path outdir = Paths.get(cmd.getOptionValue("o", "."));
		Files.createDirectories(outdir);
		Files.copy(StubbedMainForSVCOMP.class.getResourceAsStream("/stub/report.json"),
				outdir.resolve("report.json"), StandardCopyOption.REPLACE_EXISTING);
	}
}
