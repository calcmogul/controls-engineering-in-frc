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

$(NAME).pdf: $(TEX) $(STAMP) $(BIB) $(FIGS) build/commit-hash.txt
	latexmk -interaction=nonstopmode -xelatex $(NAME)

build/commit-hash.txt: .git/refs/heads/master .git/HEAD
	git rev-parse --short HEAD > build/commit-hash.txt

# Runs if frccontrol directory doesn't exist yet to perform compilation prep
build/frccontrol:
	git clone git://github.com/calcmogul/frccontrol build/frccontrol
	./generate_frccontrol_plots.py

$(FIGS): build/%.jpg: %.jpg
	@mkdir -p $(@D)
	convert -density 300 $< -resample 150 -units PixelsPerInch $@

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
upload: $(NAME).pdf
	rsync --progress $(NAME).pdf file.tavsys.net:/srv/file/control/$(NAME).pdf
