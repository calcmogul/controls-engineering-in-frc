NAME := state-space-guide

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

SVG := root_locus.svg zoh.svg discretization_methods.svg sampling_simulation.svg
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

$(NAME)-ebook.pdf: book-stamp
	gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/ebook -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$(NAME)-ebook.pdf $(NAME).pdf

$(NAME)-printer.pdf: book-stamp
	gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/printer -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$(NAME)-printer.pdf $(NAME).pdf

$(NAME)-prepress.pdf: book-stamp
	gs -sDEVICE=pdfwrite -dCompatibilityLevel=1.4 -dPDFSETTINGS=/prepress -dNOPAUSE -dQUIET -dBATCH -sOutputFile=$(NAME)-prepress.pdf $(NAME).pdf

book-stamp: $(TEX) $(PDF_TEX) $(BIB) $(FIGS) format-stamp
	xelatex $(NAME)
	makeglossaries $(NAME)
	latexmk -xelatex $(NAME)
	# Marks successful PDF generation because xelatex updates PDF contents even on
	# failure
	touch book-stamp

$(PDF_TEX): %.pdf_tex: %.svg
	inkscape -D -z --file=$< --export-pdf=$(basename $<).pdf --export-latex

format-stamp: $(CODE)
	python code/format.py
	touch format-stamp

root_locus.svg: format-stamp
	python code/root_locus.py

zoh.svg: format-stamp
	python code/zoh.py

discretization_methods.svg: format-stamp
	python code/discretization_methods.py

sampling_simulation.svg: format-stamp
	python code/sampling_simulation.py

.PHONY: clean
clean:
	rm -f *.aux *.bbl *.bcf *.blg *.fdb_latexmk *.fls *.glg *.glo *.gls *.glsdefs *.idx *.ilg *.ind *.ist *.lof *.log *.los *.lot *.out *.toc *.pdf *.pdf_tex *.ptc *.svg *.xdv *.xml book-stamp format-stamp

.PHONY: upload
upload: $(NAME)-ebook.pdf
	scp $(NAME)-ebook.pdf file.tavsys.net:/srv/file/control/$(NAME).pdf
