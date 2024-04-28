#!/bin/bash

# brew packages
#  * basictex (for xelatex)
#  * clang-format (to format C++ code snippets)
#  * cmake (for building C++ plot generators)
#  * imagemagick (to compress JPEGs)
#  * inkscape (to convert SVGs to PDFs)
brew install \
  basictex \
  clang-format \
  cmake \
  imagemagick \
  inkscape

# tlmgr packages
#  * biber (for generating bibliography)
#  * biblatex (for generating bibliography)
#  * cm-super (for type1ec.sty)
#  * csquotes (used by textcomp package)
#  * datatool (used by xfor package)
#  * enumitem (customize lists)
#  * footmisc (used by gensymb)
#  * gensymb
#  * glossaries (for makeglossaries command)
#  * glossaries-english (english language module for glossaries package)
#  * imakeidx (used by listings package)
#  * latexmk
#  * mdframed (for creating the theorem, definition, exercise, and corollary
#    boxes)
#  * mfirstuc (used by glossaries package)
#  * needspace (used by zref package)
#  * placeins (used by subcaption package)
#  * titlesec (for titletoc package)
#  * tracklang (used by glossaries package)
#  * type1cm (for type1cm.sty)
#  * was (for gensymb package)
#  * xfor (used by textcase package)
#  * zref (used by mdframed package)
sudo /Library/TeX/texbin/tlmgr update --self
sudo /Library/TeX/texbin/tlmgr install \
  biber \
  biblatex \
  cm-super \
  csquotes \
  datatool \
  enumitem \
  footmisc \
  gensymb \
  glossaries \
  glossaries-english \
  imakeidx \
  latexmk \
  mdframed \
  mfirstuc \
  needspace \
  placeins \
  titlesec \
  tracklang \
  type1cm \
  was \
  xfor \
  zref

# Python packages
#  * black (to format Python source code)
#  * pylint (for Python linting)
pip3 install --user autoflake black==24.3.0 pylint qrcode requests wheel
