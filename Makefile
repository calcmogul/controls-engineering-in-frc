NAME := controls-engineering-in-frc

DEPS_JSON := $(wildcard deps/*.json)
DEPS_STAMP := $(DEPS_JSON:.json=.stamp)
DEPS_STAMP := $(addprefix build/,$(DEPS_STAMP))

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

# Python files that generate SVG files
PY := $(filter-out ./bookutil/% ./build/% ./deps/% ./lint/% ./snippets/%,$(call rwildcard,./,*.py))
STAMP := $(PY:.py=.stamp)
STAMP := $(addprefix build/,$(STAMP))

TEX := $(call rwildcard,./,*.tex)
BIB := $(wildcard *.bib)
FIGS := $(wildcard figs/*)

CSV := $(filter-out ./bookutil/% ./build/% ./deps/% ./lint/% ./snippets/%,$(call rwildcard,./,*.csv))
CSV := $(addprefix build/,$(CSV))

ROOT := $(shell pwd)

.PHONY: all
all: $(NAME).pdf

.PHONY: ebook
ebook: $(NAME)-ebook.pdf

.PHONY: printer
printer: $(NAME)-printer.pdf

.PHONY: prepress
prepress: $(NAME)-prepress.pdf

$(NAME).pdf: $(TEX) $(STAMP) $(BIB) $(FIGS) \
		build/commit-date.tex build/commit-year.tex build/commit-hash.tex
	latexmk -interaction=nonstopmode -xelatex $(NAME)

$(NAME)-ebook.pdf: $(NAME).pdf
	gs -sDEVICE=pdfwrite \
		-dCompatibilityLevel=1.7 \
		-dPDFSETTINGS=/ebook \
		-dEmbedAllFonts=true \
		-dSubsetFonts=true \
		-dFastWebView=true \
		-dPrinted=false \
		-dNOPAUSE \
		-dQUIET \
		-dBATCH \
		-sOutputFile=$(NAME)-ebook.pdf \
		$(NAME).pdf

$(NAME)-printer.pdf: $(NAME).pdf
	gs -sDEVICE=pdfwrite \
		-dCompatibilityLevel=1.7 \
		-dPDFSETTINGS=/printer \
		-dEmbedAllFonts=true \
		-dSubsetFonts=true \
		-dFastWebView=true \
		-dPrinted=false \
		-dNOPAUSE \
		-dQUIET \
		-dBATCH \
		-sOutputFile=$(NAME)-printer.pdf \
		$(NAME).pdf

$(NAME)-prepress.pdf: $(NAME).pdf
	gs -sDEVICE=pdfwrite \
		-dCompatibilityLevel=1.7 \
		-dPDFSETTINGS=/prepress \
		-dEmbedAllFonts=true \
		-dSubsetFonts=true \
		-dFastWebView=true \
		-dPrinted=false \
		-dNOPAUSE \
		-dQUIET \
		-dBATCH \
		-sOutputFile=$(NAME)-prepress.pdf \
		$(NAME).pdf

build/commit-date.tex: .git/refs/heads/$(git rev-parse --abbrev-ref HEAD) .git/HEAD
	@mkdir -p $(@D)
	date -d @`git log -1 --format=%at` "+%B %-d, %Y" > build/commit-date.tex

build/commit-year.tex: .git/refs/heads/$(git rev-parse --abbrev-ref HEAD) .git/HEAD
	@mkdir -p $(@D)
	date -d @`git log -1 --format=%at` +%Y > build/commit-year.tex

build/commit-hash.tex: .git/refs/heads/$(git rev-parse --abbrev-ref HEAD) .git/HEAD
	@mkdir -p $(@D)
	echo "\href{https://github.com/calcmogul/$(NAME)/commit/`git rev-parse --short HEAD`}{commit `git rev-parse --short HEAD`}" > build/commit-hash.tex

$(DEPS_STAMP): build/%.stamp: %.json
	@mkdir -p $(@D)
	$(ROOT)/deps/pkg.py init
	$(ROOT)/build/venv/bin/pip3 install -e $(ROOT)/bookutil
	$(ROOT)/deps/pkg.py install_all
	touch $@

# This rule places CSVs into the build folder so scripts executed from the build
# folder can use them.
$(CSV): build/%.csv: %.csv
	@mkdir -p $(@D)
	cp $< $@

$(STAMP): build/%.stamp: %.py $(CSV) $(DEPS_STAMP)
	@mkdir -p $(@D)
	cd $(@D) && $(ROOT)/build/venv/bin/python3 $(ROOT)/$< --noninteractive
	touch $@

.PHONY: format
format:
	./lint/format_bibliography.py
	./lint/format_json.py
	./lint/format_paragraph_breaks.py
	python3 -m black -q .

.PHONY: lint
lint: format build/commit-date.tex build/commit-year.tex build/commit-hash.tex
	git --no-pager diff --exit-code HEAD  # Ensure formatter made no changes
	./lint/check_filenames.py
	./lint/check_tex_includes.py
	./lint/check_tex_labels.py
	./lint/check_links.py

.PHONY: clean
clean: clean_tex
	rm -rf build

.PHONY: clean_tex
clean_tex:
	latexmk -xelatex -C
	rm -f controls-engineering-in-frc-*.pdf

.PHONY: upload
upload: ebook
	rsync --progress $(NAME)-ebook.pdf file.tavsys.net:/srv/file/control/$(NAME).pdf

.PHONY: setup_arch
setup_arch:
	sudo pacman -Sy --needed --noconfirm \
		base-devel \
		biber \
		ghostscript \
		inkscape \
		texlive-bibtexextra \
		texlive-core \
		texlive-latexextra \
		python \
		python-black \
		python-pip \
		python-requests

.PHONY: setup_ubuntu
setup_ubuntu:
	sudo apt-get update -y
	sudo apt-get install -y \
		biber \
		build-essential \
		cm-super \
		ghostscript \
		inkscape \
		latexmk \
		texlive-bibtex-extra \
		texlive-generic-extra \
		texlive-latex-extra \
		texlive-xetex \
		python3 \
		python3-pip \
		python3-requests \
		python3-setuptools
	# Install "black" Ubuntu package instead on 19.10+
	pip3 install --user black
