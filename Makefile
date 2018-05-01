NAME := state-space-guide

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

SVG := root_locus.svg
PDF_TEX := $(SVG:.svg=.pdf_tex)
TEX := $(call rwildcard,./,*.tex)
BIB := $(wildcard *.bib)
CODE := $(call rwildcard,./,*.py)
FIGS := $(wildcard figs/*)

.PHONY: all
all: $(NAME)-ebook.pdf

.PHONY: ebook
ebook: $(NAME)-ebook.pdf

.PHONY: printer
printer: $(NAME)-printer.pdf

.PHONY: prepress
prepress: $(NAME)-prepress.pdf

$(NAME)-ebook.pdf: $(NAME).pdf
	gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/ebook -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$(NAME)-ebook.pdf $(NAME).pdf

$(NAME)-printer.pdf: $(NAME).pdf
	gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/printer -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$(NAME)-printer.pdf $(NAME).pdf

$(NAME)-prepress.pdf: $(NAME).pdf
	gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/prepress -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$(NAME)-prepress.pdf $(NAME).pdf

$(NAME).pdf: $(TEX) $(PDF_TEX) $(BIB) $(FIGS) format-stamp
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
upload: $(NAME)-ebook.pdf
	scp $(NAME)-ebook.pdf file.tavsys.net:/srv/file/control/$(NAME).pdf
