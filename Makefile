NAME := state-space-guide

# Make does not offer a recursive wildcard function, so here's one:
rwildcard=$(wildcard $1$2) $(foreach dir,$(wildcard $1*),$(call rwildcard,$(dir)/,$2))

# Python files that generate SVG files
PY := $(filter-out code/latexutils.py,$(wildcard code/*.py))
STAMP := $(PY:.py=.stamp)
STAMP := $(addprefix build/,$(STAMP))

TEX := $(call rwildcard,./,*.tex)
BIB := $(wildcard *.bib)
FIGS := $(wildcard figs/*)

ROOT := $(shell pwd)

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

# book-stamp marks successful PDF generation because xelatex updates PDF
# contents even on failure
book-stamp: $(TEX) $(STAMP) $(BIB) $(FIGS)
	latexmk -xelatex $(NAME)
	touch book-stamp

# Runs if frccontrol directory doesn't exist yet to perform compilation prep
build/frccontrol:
	mkdir -p build/code && cp code/kalman_robot.csv build/code
	rm -rf build/frccontrol && git clone git://github.com/calcmogul/frccontrol build/frccontrol --depth=1
	cd build && ./frccontrol/examples/drivetrain.py --save-plots --noninteractive
	cd build && inkscape -D -z --file=drivetrain_pzmaps.svg --export-pdf=drivetrain_pzmaps.pdf
	cd build && inkscape -D -z --file=drivetrain_response.svg --export-pdf=drivetrain_response.pdf
	cd build && ./frccontrol/examples/elevator.py --save-plots --noninteractive
	cd build && inkscape -D -z --file=elevator_pzmaps.svg --export-pdf=elevator_pzmaps.pdf
	cd build && inkscape -D -z --file=elevator_response.svg --export-pdf=elevator_response.pdf
	cd build && ./frccontrol/examples/flywheel.py --save-plots --noninteractive
	cd build && inkscape -D -z --file=flywheel_pzmaps.svg --export-pdf=flywheel_pzmaps.pdf
	cd build && inkscape -D -z --file=flywheel_response.svg --export-pdf=flywheel_response.pdf
	cd build && ./frccontrol/examples/single_jointed_arm.py --save-plots --noninteractive
	cd build && inkscape -D -z --file=single_jointed_arm_pzmaps.svg --export-pdf=single_jointed_arm_pzmaps.pdf
	cd build && inkscape -D -z --file=single_jointed_arm_response.svg --export-pdf=single_jointed_arm_response.pdf

$(STAMP): build/%.stamp: %.py | build/frccontrol
	@mkdir -p $(@D)
	cd $(@D) && $(ROOT)/$< --noninteractive
	touch $@

.PHONY: clean
clean:
	rm -rf build *.aux *.bbl *.bcf *.blg *.cpp *.fdb_latexmk *.fls *.glg *.glo *.gls *.h *.idx *.ilg *.ind *.ist *.lof *.log *.los *.lot *.out *.toc *.pdf *.ptc *.svg *.xdv *.xml book-stamp

.PHONY: upload
upload: $(NAME)-ebook.pdf
	rsync $(NAME)-ebook.pdf file.tavsys.net:/srv/file/control/$(NAME).pdf
