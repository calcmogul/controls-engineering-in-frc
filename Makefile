TARGET := state-space-guide.pdf

SVG := $(wildcard *.svg)
PDF_TEX := $(SVG:.svg=.pdf_tex)
BIB := $(wildcard *.bib)
CODE := $(wildcard code/*)

.PHONY: all
all: $(TARGET)

$(TARGET): $(basename $(TARGET)).tex $(PDF_TEX) $(BIB) $(CODE)
	pdflatex $(basename $@)
	makeglossaries $(basename $@)
	latexmk -pdf $(basename $@)

%.pdf_tex: %.svg
	inkscape -D -z --file=$< --export-pdf=$(basename $<).pdf --export-latex

.PHONY: clean
clean:
	rm -f *.aux *.bbl *.blg *.fdb_latexmk *.fls *.glg *.glo *.gls *.glsdefs *.ist *.lof *.log *.los *.lot *.out *.pdf_tex *.toc *.pdf
