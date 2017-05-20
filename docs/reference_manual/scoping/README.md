Annotated oscillograms
======================

This is how these images were made:

1. Scope the necessary signals using Saleae Logic or any other oscilloscope capable of exporting data in CSV.
2. Using Wolfram Mathematica and the included notebook, plot the data from the CSV file and adjust it as necessary.
3. Export the plot from Mathematica into an EPS (Encapsulated PostScript) image file.
**Do not use PDF**, Mathematica rarely can export into it correctly, especially if complex graphics is involved.
4. Using Inkscape, open the exported EPS file and add annotations manually.
Don't forget to save the annotated file into the Inkscape SVG format for future adjustments,
shall that be needed.
Use extension `*.inkscape.svg` for clarity.
5. Export the annotated image into a PDF image file.
**Now you shouldn't use EPS anymore**, because `pdflatex` can't handle it properly
(loss of quality, buggy interpolation, and other bugs afoot).
6. Include the final image into the document.
