<img src="logo.png" alt="logo" width="300"/>

# PyLiSA - Python Frontend for LiSA (Library for Static Analysis)

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
[![Built on LiSA](https://img.shields.io/badge/Built%20on-LiSA-informational)](https://github.com/lisa-analyzer/lisa)

**PyLiSA** is a static analysis tool for Python programs, built on top of the [LiSA (Library for Static Analysis)](https://github.com/lisa-analyzer/lisa) framework. It provides a front-end that translates Python source files into LiSA's control flow graph (CFG) representation, enriches it with the semantics of a subset of the Python standard library, and runs configurable abstract interpretation analyses to detect bugs and verify program properties.

PyLiSA is a joint effort between the **[Software and Systems Verification (SSV) Research Group](https://unive-ssv.github.io/)** at [Ca' Foscari University of Venice](https://www.unive.it) and the **[University of Parma](https://www.unipr.it)**.

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Usage](#usage)
- [Command-Line Options](#command-line-options)
- [Architecture](#architecture)
- [Development](#development)
- [License](#license)

---

## Overview

PyLiSA translates Python source code into [LiSA](https://github.com/lisa-analyzer/lisa)'s intermediate representation and runs abstract interpretation analyses over the resulting program model. The analysis configuration is fully customizable — abstract domains, interprocedural strategy, and semantic checkers can all be selected independently. For SV-COMP, PyLiSA was configured with the following components:

- **Field-sensitive, point-based heap abstraction** — tracks heap objects at allocation sites with per-field sensitivity
- **Reduced product of constant propagation and interval domain** — abstracts numerical, string, and Boolean values
- **Type inference** — tracks runtime types of expressions
- **Reachability analysis** — distinguishes definitely reachable instructions from possibly reachable or unreachable ones, improving precision on conditional branches
- **Call-string-based interprocedural analysis** — context-sensitive up to 150 nested calls, with 0-CFA for call graph construction
- **Semantic checkers** — verifies no runtime exceptions are thrown

---

## Installation

**Prerequisites:** Java 17+, Gradle (wrapper included), GitHub credentials for the LiSA dependency.

### LiSA Dependency

LiSA packages are hosted on [GitHub Packages](https://github.com/lisa-analyzer/lisa/packages). Add your credentials to `~/.gradle/gradle.properties`:

```properties
gpr.user=<your-github-username>
gpr.key=<your-github-personal-access-token>
```

Or export the environment variables `USERNAME` and `TOKEN`.

### Build

```bash
git clone https://github.com/lisa-analyzer/pylisa.git
cd pylisa/pylisa
./gradlew build
```

This runs code style checks and compiles the project. To produce a self-contained executable ZIP:

```bash
./gradlew distZip
```

The archive is written to `build/distributions/pylisa-0.1.zip`.

---

## Usage

After building the distribution, run PyLiSA directly:

```bash
./build/distributions/pylisa-0.1/bin/pylisa -s path/to/File.py -o out/
```

Or via Gradle without packaging:

```bash
./gradlew run --args="-s path/to/File.py -o out/ -n ConstantPropagation"
```

Results are written to the output directory as JSON files (one per CFG) and a `report.json` summary.

---

## Command-Line Options

| Option | Long Option         | Argument  | Description                                            |
| ------ | ------------------- | --------- | ------------------------------------------------------ |
| `-s`   | `--source`          | `file...` | Python source files to analyze (space-separated)       |
| `-o`   | `--outdir`          | `path`    | Output directory for analysis results                  |
| `-n`   | `--numericalDomain` | `domain`  | Numerical domain: `ConstantPropagation`                |
| `-c`   | `--checker`         | `checker` | Semantic checker: `Exceptions`                         |
| `-l`   | `--log-level`       | `level`   | Log verbosity: `INFO`, `DEBUG`, `WARN`, `ERROR`, `OFF` |
| `-m`   | `--mode`            | `mode`    | Execution mode: `Debug` (default), `Statistics`        |
| `-v`   | `--version`         | —         | Print the tool version                                 |
| `-h`   | `--help`            | —         | Print the help message                                 |
| N/A    | `--no-html`         | —         | Disable HTML output (enabled by default)               |

---

## Architecture

Python source code is parsed via an ANTLR grammar taken from [here](https://github.com/antlr/grammars-v4).

### Python Standard Library

Standard library classes are not parsed from source. Instead, hand-written stub files (`src/main/resources/libraries/*.txt`) declare class hierarchies, fields, and method signatures using a custom DSL parsed by an ANTLR grammar. Method semantics are implemented as Java classes under `it.unive.pylisa.program.python.constructs`. Stubs are loaded lazily at analysis time by `LibrarySpecificationProvider`.

### Key Packages

| Package                                     | Purpose                                                               |
| ------------------------------------------- | --------------------------------------------------------------------- |
| `it.unive.pylisa.frontend`                  | Parsing pipeline, `PyFrontend`, `ParserContext`                       |
| `it.unive.pylisa.frontend.util`             | Shared utilities (e.g. `FQNUtils` for building fully qualified names) |
| `it.unive.pylisa.frontend.visitors`         | AST visitor base classes and hierarchy                                |
| `it.unive.pylisa.program.cfg.expression`    | Python-specific expression nodes                                      |
| `it.unive.pylisa.program.cfg.statement`     | Python-specific statement nodes                                       |
| `it.unive.pylisa.program.python.constructs` | Library method semantics                                              |
| `it.unive.pylisa.program.libraries`         | Library stub loader                                                   |
| `it.unive.pylisa.program.type`              | Python type system                                                    |
| `it.unive.pylisa.analysis`                  | Abstract domains (heap, value, type)                                  |
| `it.unive.pylisa.interprocedural`           | Call graph and interprocedural analysis                               |
| `it.unive.pylisa.checkers`                  | Semantic checkers (`AssertChecker`)                                   |
| `it.unive.pylisa.witness`                   | Violation witness generation (GraphML)                                |

---

## Development

### Running Tests

```bash
# Run all tests
./gradlew test

# Run a single test class
./gradlew test --tests "it.unive.pylisa.cron.MathTest"

# Run a single test method
./gradlew test --tests "it.unive.pylisa.cron.MathTest.testMath"
```

Test inputs live in `python-testcases/` and expected JSON outputs are stored alongside them. To regenerate expected outputs after an intentional change, temporarily set `conf.forceUpdate = true` in `TestHelpers`.

### Code Style

PyLiSA uses [Spotless](https://github.com/diffplug/spotless) (Eclipse formatter) and [Checkstyle](https://checkstyle.org/). Formatting uses tabs, not spaces.

```bash
# Check style
./gradlew checkCodeStyle

# Auto-fix formatting
./gradlew spotlessApply
```

---

## License

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).
