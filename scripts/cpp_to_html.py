#!/usr/bin/env python3
"""
C++ to HTML converter with syntax highlighting.
Output can be printed to PDF from browser (Ctrl+P -> Save as PDF)
"""

from pygments import highlight
from pygments.lexers import CppLexer
from pygments.formatters import HtmlFormatter
from pathlib import Path
import webbrowser
import sys

def cpp_to_html(cpp_file, output_html=None):
    """Convert C++ file to syntax-highlighted HTML."""
    cpp_path = Path(cpp_file)

    with open(cpp_path, 'r', encoding='utf-8') as f:
        code = f.read()

    if output_html is None:
        output_html = cpp_path.with_suffix('.html')

    lines = code.count('\n') + 1

    # Syntax highlighting with line numbers
    lexer = CppLexer()
    formatter = HtmlFormatter(
        style='vs',  # Visual Studio style - white bg, good colors
        linenos='table',
        linenostart=1,
        cssclass='highlight',
        noclasses=False
    )

    highlighted = highlight(code, lexer, formatter)
    css = formatter.get_style_defs('.highlight')

    # Full HTML with print-optimized CSS
    html = f'''<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>{cpp_path.name}</title>
    <style>
        @page {{
            size: A4;
            margin: 1cm;
        }}
        @media print {{
            body {{ font-size: 8pt !important; }}
            .no-print {{ display: none; }}
        }}
        body {{
            font-family: 'Consolas', 'Monaco', 'Courier New', monospace;
            font-size: 10pt;
            line-height: 1.4;
            background: white;
            color: #333;
            margin: 0;
            padding: 20px;
        }}
        h1 {{
            font-family: Arial, sans-serif;
            color: #2c3e50;
            border-bottom: 3px solid #3498db;
            padding-bottom: 10px;
            font-size: 18pt;
        }}
        .info {{
            font-family: Arial, sans-serif;
            font-size: 9pt;
            color: #666;
            margin-bottom: 15px;
            padding: 10px;
            background: #f8f9fa;
            border-left: 4px solid #3498db;
        }}
        .print-btn {{
            background: #3498db;
            color: white;
            border: none;
            padding: 10px 20px;
            font-size: 14px;
            cursor: pointer;
            border-radius: 5px;
            margin-bottom: 20px;
        }}
        .print-btn:hover {{
            background: #2980b9;
        }}
        {css}
        .highlight {{
            background: white !important;
            border: 1px solid #ddd;
            border-radius: 4px;
            overflow-x: auto;
        }}
        .highlight pre {{
            margin: 0;
            padding: 10px;
        }}
        .linenodiv {{
            background: #f5f5f5 !important;
            border-right: 1px solid #ddd !important;
            padding-right: 8px !important;
        }}
        .linenodiv pre {{
            color: #999 !important;
            margin: 0;
            padding: 10px 5px;
        }}
        td.code {{
            padding-left: 10px !important;
        }}
        /* Syntax colors */
        .highlight .c {{ color: #008000; }} /* Comment */
        .highlight .k {{ color: #0000ff; font-weight: bold; }} /* Keyword */
        .highlight .kt {{ color: #2b91af; }} /* Keyword Type */
        .highlight .n {{ color: #333; }} /* Name */
        .highlight .nf {{ color: #795e26; }} /* Function */
        .highlight .s {{ color: #a31515; }} /* String */
        .highlight .m {{ color: #098658; }} /* Number */
        .highlight .o {{ color: #333; }} /* Operator */
        .highlight .p {{ color: #333; }} /* Punctuation */
        .highlight .cp {{ color: #0000ff; }} /* Preprocessor */
    </style>
</head>
<body>
    <h1>{cpp_path.name}</h1>
    <div class="info">
        <strong>File:</strong> {cpp_path.name} |
        <strong>Lines:</strong> {lines} |
        <strong>Language:</strong> C++ (Gazebo Plugin)
    </div>
    <button class="print-btn no-print" onclick="window.print()">
        Print / Save as PDF (Ctrl+P)
    </button>
    {highlighted}
</body>
</html>'''

    with open(output_html, 'w', encoding='utf-8') as f:
        f.write(html)

    print(f"Created: {output_html}")
    print(f"Lines: {lines}")
    return output_html

if __name__ == "__main__":
    script_dir = Path(__file__).parent.parent
    cpp_file = script_dir / "agent_control_pkg" / "plugins" / "simple_drone_plugin.cpp"
    output = script_dir / "simple_drone_plugin.html"

    html_file = cpp_to_html(cpp_file, output)

    # Open in browser
    print("\nOpening in browser...")
    print("Press Ctrl+P to save as PDF")
    webbrowser.open(f'file:///{html_file}')