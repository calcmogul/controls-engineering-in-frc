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
IMGS := $(wildcard imgs/*)
SNIPPETS := $(wildcard snippets/*)

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

$(NAME).pdf: $(TEX) $(STAMP) $(BIB) $(IMGS) $(SNIPPETS) \
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
	git log -1 --pretty=format:%ad --date="format:%B %-d, %Y" > build/commit-date.tex

build/commit-year.tex: .git/refs/heads/$(git rev-parse --abbrev-ref HEAD) .git/HEAD
	@mkdir -p $(@D)
	git log -1 --pretty=format:%ad --date=format:%Y > build/commit-year.tex

build/commit-hash.tex: .git/refs/heads/$(git rev-parse --abbrev-ref HEAD) .git/HEAD
	@mkdir -p $(@D)
	echo "\href{https://github.com/calcmogul/$(NAME)/commit/`git rev-parse --short HEAD`}{commit `git rev-parse --short HEAD`}" > build/commit-hash.tex

$(DEPS_STAMP): build/%.stamp: %.json
	@mkdir -p $(@D)
	$(ROOT)/deps/pkg.py init
	$(ROOT)/build/venv/bin/pip3 install -e $(ROOT)/bookutil
	$(ROOT)/deps/pkg.py install_all
	@touch $@

# This rule places CSVs into the build folder so scripts executed from the build
# folder can use them.
$(CSV): build/%.csv: %.csv
	@mkdir -p $(@D)
	cp $< $@

$(STAMP): build/%.stamp: %.py $(CSV) $(DEPS_STAMP)
	@mkdir -p $(@D)
	cd $(@D) && $(ROOT)/build/venv/bin/python3 $(ROOT)/$< --noninteractive
	@touch $@

# Run formatters
.PHONY: format
format:
	./lint/format_bibliography.py
	./lint/format_eol.py
	./lint/format_json.py
	./lint/format_paragraph_breaks.py
	cd snippets && clang-format -i *.cpp
	python3 -m black -q .
	git --no-pager diff --exit-code HEAD  # Ensure formatters made no changes

# Run formatters and all linters except link checker. The commit metadata files
# are dependencies because check_tex_includes.py will fail if they're missing.
.PHONY: lint_no_linkcheck
lint_no_linkcheck: format build/commit-date.tex build/commit-year.tex build/commit-hash.tex
	./lint/check_filenames.py
	./lint/check_tex_includes.py
	./lint/check_tex_labels.py

# Run formatters and linters
.PHONY: lint
lint: lint_no_linkcheck
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

.PHONY: setup_archlinux
setup_archlinux:
	sudo pacman -Sy --needed --noconfirm \
		base-devel \
		biber \
		clang \
		ghostscript \
		inkscape \
		python \
		python-black \
		python-pip \
		python-requests \
		python-wheel \
		texlive-bibtexextra \
		texlive-core \
		texlive-latexextra

.PHONY: setup_ubuntu
setup_ubuntu:
	sudo apt-get update -y
	sudo apt-get install -y \
		biber \
		build-essential \
		cm-super \
		clang-format \
		ghostscript \
		inkscape \
		latexmk \
		python3 \
		python3-pip \
		python3-requests \
		python3-setuptools \
		python3-wheel \
		texlive-bibtex-extra \
		texlive-latex-extra \
		texlive-xetex
	# The Ubuntu 20.04 package is too old
	pip3 install --user black

.PHONY: setup_macos
setup_macos:
	brew install \
		basictex \
		clang-format \
		ghostscript \
		inkscape
	sudo /Library/TeX/texbin/tlmgr update --self
	sudo /Library/TeX/texbin/tlmgr install \
		biber \
		biblatex \
		cm-super \
		csquotes \
		datatool \
		enumitem \
		footmisc \
		glossaries \
		glossaries-english \
		imakeidx \
		latexmk \
		mdframed \
		mfirstuc \
		needspace \
		placeins \
		titlesec \
		tracklang \
		type1cm \
		was \
		xfor \
		zref
	pip3 install --user black requests wheel
