#!/usr/bin/env python3
"""
C++ Source Code to PDF Converter with Syntax Highlighting
Converts simple_drone_plugin.cpp to a beautiful, readable PDF with color-coded syntax.
"""

import sys
import os
from pathlib import Path

def check_dependencies():
    """Check if required packages are installed."""
    missing = []

    try:
        import pygments
    except ImportError:
        missing.append("pygments")

    try:
        from weasyprint import HTML
    except ImportError:
        missing.append("weasyprint")

    if missing:
        print("❌ Missing dependencies! Install with:")
        print(f"   pip install {' '.join(missing)}")
        return False
    return True

def cpp_to_pdf(cpp_file, output_pdf=None, style="default"):
    """
    Convert C++ file to PDF with syntax highlighting.

    Args:
        cpp_file: Path to .cpp file
        output_pdf: Output PDF path (default: same name with .pdf)
        style: Pygments style (default, monokai, github, vs, arduino, etc.)
    """
    from pygments import highlight
    from pygments.lexers import CppLexer
    from pygments.formatters import HtmlFormatter
    from weasyprint import HTML as WeasyprintHTML

    # Read C++ source
    cpp_path = Path(cpp_file)
    if not cpp_path.exists():
        print(f"❌ File not found: {cpp_file}")
        return False

    with open(cpp_path, 'r', encoding='utf-8') as f:
        code = f.read()

    # Default output path
    if output_pdf is None:
        output_pdf = cpp_path.with_suffix('.pdf')

    # Generate syntax-highlighted HTML
    lexer = CppLexer()
    formatter = HtmlFormatter(
        style=style,
        full=True,
        linenos='table',  # Line numbers in table format
        cssclass="source",
        linespans='line',
        anchorlinenos=True,
        lineanchors='L',
        hl_lines=None,
        noclasses=False
    )

    highlighted_html = highlight(code, lexer, formatter)

    # Custom CSS for better readability on tablet
    custom_css = """
    <style>
        @page {
            size: A4;
            margin: 1.5cm;
        }
        body {
            font-family: 'Consolas', 'Monaco', 'Courier New', monospace;
            font-size: 9pt;
            line-height: 1.3;
            background-color: white;
            color: #333;
        }
        .source {
            background-color: white !important;
            border: 1px solid #ddd;
            border-radius: 4px;
            padding: 10px;
        }
        .linenos {
            background-color: #f5f5f5 !important;
            color: #999 !important;
            padding-right: 10px !important;
            border-right: 1px solid #ddd !important;
            user-select: none;
            font-size: 8pt;
        }
        pre {
            margin: 0;
            white-space: pre-wrap;
            word-wrap: break-word;
        }
        .highlight {
            background-color: white !important;
        }
        /* Title */
        h1 {
            font-family: Arial, sans-serif;
            color: #2c3e50;
            border-bottom: 3px solid #3498db;
            padding-bottom: 10px;
            margin-bottom: 20px;
        }
        .metadata {
            font-family: Arial, sans-serif;
            font-size: 10pt;
            color: #7f8c8d;
            margin-bottom: 20px;
            border-left: 4px solid #3498db;
            padding-left: 15px;
        }
    </style>
    """

    # Add title and metadata
    file_stats = cpp_path.stat()
    lines = code.count('\n') + 1

    title_html = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <meta charset="UTF-8">
        <title>{cpp_path.name}</title>
        {custom_css}
    </head>
    <body>
        <h1>📄 {cpp_path.name}</h1>
        <div class="metadata">
            <strong>File:</strong> {cpp_path.absolute()}<br>
            <strong>Lines:</strong> {lines}<br>
            <strong>Size:</strong> {file_stats.st_size / 1024:.1f} KB<br>
            <strong>Language:</strong> C++ (Gazebo Plugin)<br>
            <strong>Generated:</strong> {Path(__file__).name}
        </div>
    """

    # Combine HTML
    full_html = highlighted_html.replace(
        '<!DOCTYPE html PUBLIC',
        title_html + '<!DOCTYPE html PUBLIC'
    ).replace('</body>', '</div></body>')

    # Convert to PDF
    print(f"📝 Converting {cpp_path.name} to PDF...")
    print(f"   Style: {style}")
    print(f"   Lines: {lines}")

    WeasyprintHTML(string=full_html).write_pdf(output_pdf)

    output_size = Path(output_pdf).stat().st_size / 1024
    print(f"✅ PDF created: {output_pdf}")
    print(f"   Size: {output_size:.1f} KB")
    return True

def main():
    """Main entry point."""
    if not check_dependencies():
        sys.exit(1)

    # Default: Convert simple_drone_plugin.cpp
    script_dir = Path(__file__).parent.parent
    cpp_file = script_dir / "agent_control_pkg" / "plugins" / "simple_drone_plugin.cpp"
    output_pdf = script_dir / "simple_drone_plugin_highlighted.pdf"

    # Available styles: default, monokai, github, vs, arduino, paraiso-light, etc.
    # For white background, use: default, github, vs, arduino, paraiso-light
    style = "vs"  # Visual Studio style (white background, good colors)

    if len(sys.argv) > 1:
        cpp_file = sys.argv[1]
    if len(sys.argv) > 2:
        output_pdf = sys.argv[2]
    if len(sys.argv) > 3:
        style = sys.argv[3]

    success = cpp_to_pdf(cpp_file, output_pdf, style)

    if success:
        print("\n🎉 Done! You can now transfer the PDF to your tablet.")
        print(f"   Open: {output_pdf}")
    else:
        sys.exit(1)

if __name__ == "__main__":
    main()
