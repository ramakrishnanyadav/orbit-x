# How to Compile the Technical Report

## Method 1: Online LaTeX Editor (Easiest - No Installation)

### Overleaf (Recommended)
1. Go to https://www.overleaf.com
2. Create free account
3. Click "New Project" → "Upload Project"
4. Upload `technical_report.tex`
5. Click "Recompile" (or enable auto-compile)
6. Download PDF from "Download PDF" button

**Estimated time**: 2 minutes

---

## Method 2: Local LaTeX Installation

### Windows
```bash
# Install MiKTeX (LaTeX distribution)
# Download from: https://miktex.org/download

# Compile
pdflatex technical_report.tex
pdflatex technical_report.tex  # Run twice for references
```

### macOS
```bash
# Install MacTeX
brew install --cask mactex

# Compile
pdflatex technical_report.tex
pdflatex technical_report.tex
```

### Linux (Ubuntu/Debian)
```bash
# Install TeX Live
sudo apt-get install texlive-full

# Compile
pdflatex technical_report.tex
pdflatex technical_report.tex
```

---

## Method 3: Online PDF Conversion (If LaTeX Fails)

1. Convert LaTeX to Markdown:
   - Use `technical_report.md` (already provided)

2. Convert Markdown to PDF:
   - **Pandoc**: `pandoc technical_report.md -o technical_report.pdf --pdf-engine=xelatex`
   - **Typora**: Open in Typora, File → Export → PDF
   - **VS Code**: Use "Markdown PDF" extension
   - **Online**: https://www.markdowntopdf.com

---

## Troubleshooting

### Missing Packages
If you get errors like `! LaTeX Error: File 'algorithm.sty' not found`:

**MiKTeX (Windows)**:
- MiKTeX will auto-download packages on first use
- Or use: Package Manager → Install packages

**TeX Live (macOS/Linux)**:
```bash
sudo tlmgr install algorithms
sudo tlmgr install algorithmicx
```

### Common Errors

**Error**: `Undefined control sequence \toprule`
**Fix**: Install `booktabs` package

**Error**: `Unknown graphics extension`
**Fix**: Ensure you're using `pdflatex` (not `latex`)

---

## Expected Output

- **File**: `technical_report.pdf`
- **Pages**: 7-8 pages
- **Size**: ~200-300 KB
- **Content**:
  - Title page with abstract
  - Introduction (0.5 pages)
  - Aircraft mission (2 pages)
  - Spacecraft mission (2 pages)
  - Unified architecture (0.5 pages)
  - Validation (1 page)
  - Discussion & conclusion (0.5 pages)
  - References

---

## Quick Quality Check

After generating PDF, verify:
- [ ] Title and author names correct
- [ ] All equations render properly
- [ ] All tables have captions
- [ ] All sections present (7 main sections)
- [ ] References formatted correctly
- [ ] Page count: 7-8 pages
- [ ] No LaTeX compilation errors in output

---

## Alternative: Use Pre-Generated Template

If LaTeX is problematic, we can:
1. Convert to Word document (.docx)
2. Use Google Docs with equation editor
3. Generate from Jupyter Notebook with LaTeX cells

**Recommendation**: Use Overleaf (Method 1) - it's the easiest and most reliable!
