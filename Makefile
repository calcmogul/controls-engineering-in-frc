NAME := state-space-guide

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

# Python files that generate SVG files
PY := $(filter-out code/latexutils.py,$(wildcard code/*.py))
STAMP := $(PY:.py=.stamp)
STAMP := $(addprefix build/,$(STAMP))

TEX := $(call rwildcard,./,*.tex)
BIB := $(wildcard *.bib)

FIGS := $(wildcard figs/*)
FIGS := $(addprefix build/,$(FIGS))

CSV := $(wildcard code/*.csv)
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

$(NAME).pdf: $(TEX) $(STAMP) $(BIB) $(FIGS) build/commit-hash.txt
	latexmk -interaction=nonstopmode -xelatex $(NAME)

$(NAME)-ebook.pdf: $(NAME).pdf
	gs -sDEVICE=pdfwrite \
		-dCompatibilityLevel=1.4 \
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
		-dCompatibilityLevel=1.4 \
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
		-dCompatibilityLevel=1.4 \
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

build/commit-hash.txt: .git/refs/heads/master .git/HEAD
	echo "\href{https://github.com/calcmogul/state-space-guide/commit/`git rev-parse --short HEAD`}{commit `git rev-parse --short HEAD`}" > build/commit-hash.txt

# Runs if frccontrol directory doesn't exist yet to perform compilation prep
build/frccontrol:
	git clone git://github.com/calcmogul/frccontrol build/frccontrol
	./generate_frccontrol_plots.py

$(FIGS): build/%.jpg: %.jpg
	@mkdir -p $(@D)
	cp $< $@

$(CSV): build/%.csv: %.csv
	@mkdir -p $(@D)
	cp $< $@

$(STAMP): build/%.stamp: %.py $(CSV) | build/frccontrol
	@mkdir -p $(@D)
	cd $(@D) && $(ROOT)/$< --noninteractive
	touch $@

.PHONY: clean
clean:
	rm -rf build *.aux *.bbl *.bcf *.blg *.fdb_latexmk *.fls *.glg *.glo *.gls *.idx *.ilg *.ind *.ist *.lof *.log *.los *.lot *.out *.toc *.pdf *.ptc *.xdv *.xml

.PHONY: upload
upload: ebook
	rsync --progress $(NAME)-ebook.pdf file.tavsys.net:/srv/file/control/$(NAME).pdf
