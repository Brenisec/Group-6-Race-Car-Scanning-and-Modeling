export PATH := /usr/local/apps/tex_live/current/bin/x86_64-linux:$(PATH)

default: pdf concurrency zip pdf clean

all: pdf pdf concurrency zip clean

pdf:
	pdflatex -synctex=1 -interaction=nonstopmode main.tex

concurrency:
	gcc -lpthread -o concurrency1-13 concurrency1-13.c

# Trim out the excess fat.
clean:
	rm -f *.ps *.log *.aux *.out *.dvi *.bbl *.blg *.gz *.acn *.ist *.acr *.alg *.glg *.pyg
