NAME := state-space-guide

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

SVG := root_locus.svg
PDF_TEX := $(SVG:.svg=.pdf_tex)
TEX := $(call rwildcard,./,*.tex)
BIB := $(wildcard *.bib)
CODE := $(wildcard code/*)

.PHONY: all
all: $(NAME).pdf

$(NAME).pdf: $(TEX) $(PDF_TEX) $(BIB) format-stamp
	xelatex $(NAME)
	makeglossaries $(NAME)
	latexmk -xelatex $(NAME)

$(PDF_TEX): %.pdf_tex: %.svg
	inkscape -D -z --file=$< --export-pdf=$(basename $<).pdf --export-latex

format-stamp: $(CODE)
	python code/format.py
	touch format-stamp

root_locus.svg: code/root_locus.py
	python code/root_locus.py

.PHONY: clean
clean:
	rm -f *.aux *.bbl *.bcf *.blg *.fdb_latexmk *.fls *.glg *.glo *.gls *.glsdefs *.idx *.ilg *.ind *.ist *.lof *.log *.los *.lot *.out *.toc *.pdf *.pdf_tex *.ptc *.svg *.xdv *.xml format-stamp

.PHONY: upload
upload: all
	scp $(NAME).pdf file.tavsys.net:/srv/file/control
