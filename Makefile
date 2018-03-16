TARGET := state-space-guide.pdf

SVG := figs/root_locus.svg
PDF_TEX := $(SVG:.svg=.pdf_tex)
TEX := $(wildcard *.tex)
BIB := $(wildcard *.bib)
CODE := $(wildcard code/*)

.PHONY: all
all: $(TARGET)

$(TARGET): $(TEX) $(PDF_TEX) $(BIB) $(CODE)
	pdflatex $(basename $@)
	makeglossaries $(basename $@)
	latexmk -pdf $(basename $@)

$(PDF_TEX): figs/%.pdf_tex: figs/%.svg
	inkscape -D -z --file=$< --export-pdf=$(basename $<).pdf --export-latex

figs/root_locus.svg: code/root_locus.py
	python code/root_locus.py

.PHONY: clean
clean:
	rm -f figs/*.pdf figs/*.pdf_tex figs/root_locus.svg *.aux *.bbl *.blg *.fdb_latexmk *.fls *.glg *.glo *.gls *.glsdefs *.ist *.lof *.log *.los *.lot *.out *.toc *.pdf

.PHONY: upload
upload: all
	scp $(TARGET) file.tavsys.net:/srv/file/control
