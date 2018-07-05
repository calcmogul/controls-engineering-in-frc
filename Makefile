NAME := state-space-guide

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

# Python files that generate SVG files
PY := $(wildcard code/*.py)
SVGGEN := $(filter-out code/format_all.py,$(wildcard code/*.py))
SVG := $(SVGGEN:.py=.svg)
SVG := $(addprefix build/,$(SVG))
PDF := $(SVG:.svg=.pdf)

EXAMPLES_PY := $(call rwildcard,code/frccontrol/examples/,*.py)
EXAMPLES_STAMP := $(EXAMPLES_PY:.py=.stamp)
EXAMPLES_STAMP := $(addprefix build/,$(EXAMPLES_STAMP))

TEX := $(call rwildcard,./,*.tex)
BIB := $(wildcard *.bib)
FIGS := $(wildcard figs/*)

.PHONY: all
all: $(NAME)-ebook.pdf

.PHONY: ebook
ebook: $(NAME)-ebook.pdf

.PHONY: printer
printer: $(NAME)-printer.pdf

.PHONY: prepress
prepress: $(NAME)-prepress.pdf

$(NAME)-ebook.pdf: book-stamp
	gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/ebook -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$(NAME)-ebook.pdf $(NAME).pdf

$(NAME)-printer.pdf: book-stamp
	gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/printer -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$(NAME)-printer.pdf $(NAME).pdf

$(NAME)-prepress.pdf: book-stamp
	gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/prepress -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$(NAME)-prepress.pdf $(NAME).pdf

book-stamp: $(TEX) $(PDF) $(EXAMPLES_STAMP) $(BIB) $(FIGS)
	xelatex $(NAME)
	makeglossaries $(NAME)
	latexmk -xelatex $(NAME)
	# Marks successful PDF generation because xelatex updates PDF contents even on
	# failure
	touch book-stamp

$(PDF): build/%.pdf: build/%.svg
	@mkdir -p $(@D)
	inkscape -D -z --file=$< --export-pdf=$@

$(SVG): build/%.svg: %.py
	@mkdir -p $(@D)
	PYTHONPATH=code ./$<
	mv $(notdir $@) $(@D)

$(EXAMPLES_STAMP): build/%.stamp: %.py
	@mkdir -p $(@D)
	PYTHONPATH=code ./$<
	inkscape -D -z --file=$(<F:.py=_pzmaps.svg) \
		--export-pdf=$(<F:.py=_pzmaps.pdf)
	inkscape -D -z --file=$(<F:.py=_response.svg) \
		--export-pdf=$(<F:.py=_response.pdf)
	mv *.cpp *.h *.pdf *.svg $(@D)
	touch $@

.PHONY: clean
clean:
	rm -rf build *.aux *.bbl *.bcf *.blg *.cpp *.fdb_latexmk *.fls *.glg *.glo *.gls *.glsdefs *.h *.idx *.ilg *.ind *.ist *.lof *.log *.los *.lot *.out *.toc *.pdf *.ptc *.svg *.xdv *.xml book-stamp

.PHONY: upload
upload: $(NAME)-ebook.pdf
	rsync $(NAME)-ebook.pdf file.tavsys.net:/srv/file/control/$(NAME).pdf
