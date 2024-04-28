#!/bin/bash

# apt packages
#   * biber (for generating bibliography)
#   * build-essential (for `make` to run the makefile)
#   * cm-super (for type1ec.sty)
#   * clang-format (to format C++ code snippets)
#   * cmake (for building C++ plot generators)
#   * dvipng (to convert DVIs to PNGs)
#   * imagemagick (to compress JPEGs)
#   * inkscape (to convert SVGs to PDFs)
#   * latexmk
#   * python3 >= 3.6 (for generating plots)
#   * python3-pip (for installing required Python packages)
#   * python3-requests (for .tex HTTP link checker)
#   * python3-setuptools (for dependencies of setup.py)
#   * python3-wheel (for "setup.py bdist_wheel")
#   * texlive-base (for latexmk)
#   * texlive-bibtex-extra (for additional BibTeX styles and bibliography
#     databases)
#   * texlive-generic-extra (for miscellaneous LaTeX .sty files)
#   * texlive-latex-extra (for bibtex and makeglossaries)
#   * texlive-xetex (for xelatex)
sudo apt-get update -y
sudo apt-get install -y \
  biber \
  build-essential \
  cm-super \
  clang-format \
  cmake \
  dvipng \
  imagemagick \
  inkscape \
  latexmk \
  python3 \
  python3-pip \
  python3-requests \
  python3-setuptools \
  python3-wheel \
  texlive-base \
  texlive-bibtex-extra \
  texlive-latex-extra \
  texlive-xetex

# The Ubuntu 22.04 packages are too old
# Python packages
#   * black (to format Python source code)
#   * pylint (for Python linting)
pip3 install --user autoflake black==24.3.0 pylint qrcode
