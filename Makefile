NAME := state-space-guide

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

# Python files that generate SVG files
PY := $(wildcard code/*.py)
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
	rm -rf build/frccontrol && git clone git://github.com/calcmogul/frccontrol build/frccontrol --depth=1
	cd build && ./frccontrol/examples/drivetrain.py --noninteractive
	cd build && ./frccontrol/examples/elevator.py --noninteractive
	cd build && ./frccontrol/examples/flywheel.py --noninteractive
	cd build && ./frccontrol/examples/single_jointed_arm.py --noninteractive
	cd build && ../svg2pdf.py
	xelatex $(NAME)
	touch init-stamp

$(STAMP): build/%.stamp: %.py
	@mkdir -p $(@D)
	./$<
	./svg2pdf.py
	mv *.pdf *.svg $(@D)
	touch $@

.PHONY: clean
clean:
	rm -rf build *.aux *.bbl *.bcf *.blg *.cpp *.fdb_latexmk *.fls *.glg *.glo *.gls *.glsdefs *.h *.idx *.ilg *.ind *.ist *.lof *.log *.los *.lot *.out *.toc *.pdf *.ptc *.svg *.xdv *.xml book-stamp init-stamp

.PHONY: upload
upload: $(NAME)-ebook.pdf
	rsync $(NAME)-ebook.pdf file.tavsys.net:/srv/file/control/$(NAME).pdf
