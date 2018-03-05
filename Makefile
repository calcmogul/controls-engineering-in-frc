TARGET := state-space-guide.pdf

SVG := $(wildcard figs/*.svg)
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
	rm -f figs/*.pdf figs/*.pdf_tex *.aux *.bbl *.blg *.fdb_latexmk *.fls *.glg *.glo *.gls *.glsdefs *.ist *.lof *.log *.los *.lot *.out *.toc *.pdf

.PHONY: upload
upload: all
	scp $(TARGET) file.tavsys.net:/srv/file/control
