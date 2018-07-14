NAME := state-space-guide

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

# Python files that generate SVG files
PY := $(wildcard code/*.py) \
	$(call rwildcard,code/frccontrol/examples/,*.py)
STAMP := $(PY:.py=.stamp)
STAMP := $(addprefix build/,$(STAMP))

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

book-stamp: $(NAME).gls $(TEX) $(STAMP) $(BIB) $(FIGS)
	latexmk -xelatex $(NAME)
	# Marks successful PDF generation because xelatex updates PDF contents even on
	# failure
	touch book-stamp

$(NAME).gls: $(NAME).aux
	makeglossaries $(NAME)
	latexmk -xelatex $(NAME)
	# makeglossaries ran twice because latexmk writes a new .aux file on the first
	# run
	makeglossaries $(NAME)

$(NAME).aux: init-stamp

init-stamp: $(STAMP)
	xelatex $(NAME)
	touch init-stamp

$(STAMP): build/%.stamp: %.py
	@mkdir -p $(@D)
	PYTHONPATH=code ./$< --noninteractive
	./svg2pdf.py
	mv *.pdf *.svg $(@D)
	touch $@

.PHONY: clean
clean:
	rm -rf build *.aux *.bbl *.bcf *.blg *.cpp *.fdb_latexmk *.fls *.glg *.glo *.gls *.glsdefs *.h *.idx *.ilg *.ind *.ist *.lof *.log *.los *.lot *.out *.toc *.pdf *.ptc *.svg *.xdv *.xml book-stamp init-stamp

.PHONY: upload
upload: $(NAME)-ebook.pdf
	rsync $(NAME)-ebook.pdf file.tavsys.net:/srv/file/control/$(NAME).pdf
