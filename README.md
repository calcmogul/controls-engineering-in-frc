# Controls Engineering in the FIRST Robotics Competition
## Graduate-level control theory for high schoolers

I originally wrote this as a final project for an undergraduate technical
writing class I took at University of California, Santa Cruz in Spring 2017
([CMPE 185](https://cmpe185-spring17-01.courses.soe.ucsc.edu/)). It is intended
as a digest of graduate-level control theory aimed at veteran FIRST Robotics
Competition (FRC) students who know algebra and a bit of physics. As I learned
the subject of control theory, I found that it wasn't particularly difficult,
but very few resources exist outside of academia for learning it. This book is
intended to rectify that situation and lower the barrier to entry to the field.

This book reads a lot like a reference manual on control theory and related
tools. It teaches the reader how to start designing and implementing control
systems for practical systems with an emphasis on pragmatism rather than theory.
While the theory is mathematically elegant at times and helps inform what is
going on, one shouldn't lose sight of how it behaves when applied to real
systems.

## Download

A PDF version is available at https://controls-in-frc.link (full link:
https://file.tavsys.net/control/controls-engineering-in-frc.pdf).

## Running the examples

Example Python scripts can be obtained from frccontrol's Git repository at
https://github.com/calcmogul/frccontrol/tree/main/examples. Furthermore, all
scripts in the [code](code) directory are runnable by the user. They require
Python 3.5+ and frccontrol. frccontrol can be installed with the following
command.

```
pip3 install --user frccontrol
```

Some also use bookutil, a Python package containing common utilities for this
book. That can be installed by running `pip3 install --user -e bookutil` from
the root of this Git repository.

The scripts can be run as follows:

```
./elevator.py
```

Some Linux platforms use tk as a backend for matplotlib, so that may need to be
installed to see the plots.

## Compiling the book

After installing the dependencies, just run `make`. It will produce a PDF named
controls-engineering-in-frc.pdf.

### Dependencies

To compile the book, the following packages are required.

#### Arch Linux

These can be installed via `make setup_archlinux`.

* base-devel (for `make` to run the makefile)
* biber (for generating bibliography)
* ghostscript (to reduce size of final PDF for publishing)
* inkscape (to convert SVGs to PDFs)
* texlive-bibtexextra (for additional BibTeX styles and bibliography databases)
* texlive-core (for latexmk and xelatex)
* texlive-latexextra (for bibtex and makeglossaries)
* python >= 3.6 (for generating plots)
* python-pip (for installing required Python packages)
* python-wheel (for "setup.py bdist_wheel")

#### Ubuntu

These can be installed via `make setup_ubuntu`.

* biber (for generating bibliography)
* build-essential (for `make` to run the makefile)
* cm-super (for type1ec.sty)
* ghostscript (to reduce size of final PDF for publishing)
* inkscape (to convert SVGs to PDFs)
* latexmk
* texlive-bibtex-extra (for additional BibTeX styles and bibliography databases)
* texlive-generic-extra (for miscellaneous LaTeX .sty files)
* texlive-latex-extra (for bibtex and makeglossaries)
* texlive-xetex (for xelatex)
* python3 >= 3.6 (for generating plots)
* python3-pip (for installing required Python packages)
* python3-wheel (for "setup.py bdist_wheel")

#### Python packages

These packages are installed via pip3 (e.g., `pip3 install --user frccontrol`).

* frccontrol (to provide FRC wrappers for Python Control and generate plots and
  state-space results)

The book's build process automatically sets these up in a venv so they don't
have to be installed manually. Modifications to the Python package folders in
`build` will be reflected in any scripts which use the venv.

The following packages are optional because the book can compile without them.

* black (to format Python source code)
* requests (for .tex HTTP link checker)

### Style guide

All LaTeX labels, including bibliography entries, should use underscores to
represent spaces between words instead of hyphens. Hyphens should only be used
where the character being written is already a hyphen. `.tex` file names should
use hyphens and `.py` file names should use underscores.

Glossary entries in [glossary-entries.tex](glossary-entries.tex) should be
lexographically sorted by entry key. The entry key should be the same as the
name. The words in the name should be lowercase, and the description should
start with a capital letter and end with a period. The first sentence should be
a fragment, but sentences after that, if applicable, should be complete.

Bibliography entries in
[controls-engineering-in-frc.bib](controls-engineering-in-frc.bib) should be
sorted lexographically by label. Links to online videos should use the `@misc`
tag. Links to online static resources like PDFs should use the `@online` tag.

Book content should answer the question _why_ something works the way it does
and how and when to use it to solve problems.

## Future improvements

See [TODO.md](TODO.md).

## Licensing

This project, except for the software, is released under the Creative Commons
Attribution-ShareAlike 4.0 International license. The software is released under
the 3-clause BSD license.
