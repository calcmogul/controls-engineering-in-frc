TARGET := state-space-guide.pdf

BIB := $(wildcard *.bib)
SVG := $(wildcard *.svg)
PDF_TEX := $(SVG:.svg=.pdf_tex)

.PHONY: all
all: $(TARGET)

$(TARGET): $(basename $(TARGET)).tex $(PDF_TEX) $(BIB)
	pdflatex $(basename $@)
	makeglossaries $(basename $@)
	latexmk -pdf $(basename $@)

%.pdf_tex: %.svg
	SELF_CALL=true inkscape -D --export-type=pdf --export-latex --export-filename=$(basename $<).pdf $<

.PHONY: clean
clean:
	rm -f *.aux *.bbl *.blg *.dvi *.fdb_latexmk *.fls *.glg *.glo *.gls *.glsdefs *.ist *.lof *.log *.lot *.out *.pdf_tex *.toc *.pdf
