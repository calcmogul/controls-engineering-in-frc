TARGET := state-space-guide.pdf

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

SVG := root_locus.svg
PDF_TEX := $(SVG:.svg=.pdf_tex)
TEX := $(call rwildcard,./,*.tex)
BIB := $(wildcard *.bib)
CODE := $(wildcard code/*)

.PHONY: all
all: $(TARGET)

$(TARGET): $(TEX) $(PDF_TEX) $(BIB) format-stamp
	pdflatex $(basename $@)
	makeglossaries $(basename $@)
	latexmk -pdf $(basename $@)

$(PDF_TEX): %.pdf_tex: %.svg
	inkscape -D -z --file=$< --export-pdf=$(basename $<).pdf --export-latex

format-stamp: $(CODE)
	python code/format.py
	touch format-stamp

root_locus.svg: code/root_locus.py
	python code/root_locus.py

.PHONY: clean
clean:
	rm -f *.aux *.bbl *.blg *.fdb_latexmk *.fls *.glg *.glo *.gls *.glsdefs *.ist *.lof *.log *.los *.lot *.out *.toc *.pdf *.pdf_tex *.svg

.PHONY: upload
upload: all
	scp $(TARGET) file.tavsys.net:/srv/file/control
