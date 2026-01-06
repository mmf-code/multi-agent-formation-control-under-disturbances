#!/usr/bin/env python3
"""
Generate syntax-highlighted HTML files for all study materials.
Based on COMPLETE_SYSTEM_MANUAL.md Ek A2: Kod Okuma Sirasi

Usage:
    python scripts/generate_study_pdfs.py          # Generate all
    python scripts/generate_study_pdfs.py --open   # Generate and open in browser
    python scripts/generate_study_pdfs.py 1        # Generate only file #1
    python scripts/generate_study_pdfs.py 1 2 3    # Generate files #1, #2, #3
"""

from pygments import highlight
from pygments.lexers import CppLexer, PythonLexer, YamlLexer, get_lexer_by_name
from pygments.formatters import HtmlFormatter
from pathlib import Path
import webbrowser
import sys
import os

# Study files in recommended reading order (from COMPLETE_SYSTEM_MANUAL.md)
STUDY_FILES = [
    {
        "num": 1,
        "name": "MetricsData.msg",
        "path": "other_packages/my_custom_interfaces_pkg/msg/MetricsData.msg",
        "lang": "cpp",  # .msg is similar to struct definition
        "topic": "Veri Yapilari - ROS2 mesaj tipleri",
        "lines": "1-50",
        "day": 1
    },
    {
        "num": 2,
        "name": "simple_drone_plugin.cpp",
        "path": "agent_control_pkg/plugins/simple_drone_plugin.cpp",
        "lang": "cpp",
        "topic": "Gazebo Fizik - F=ma, odom, wind injection",
        "lines": "ALL",
        "day": 1
    },
    {
        "num": 3,
        "name": "pid_controller.cpp",
        "path": "agent_control_pkg/src/pid_controller.cpp",
        "lang": "cpp",
        "topic": "PID Algoritma - P, I, D terimleri, anti-windup",
        "lines": "77-150",
        "day": 2
    },
    {
        "num": 4,
        "name": "it2_fuzzy_logic_system.cpp",
        "path": "agent_control_pkg/src/it2_fuzzy_logic_system.cpp",
        "lang": "cpp",
        "topic": "IT2 Fuzzy - MF, rule inference, Karnik-Mendel",
        "lines": "150-500",
        "day": 2
    },
    {
        "num": 5,
        "name": "gt2_fuzzy_logic_system.cpp",
        "path": "agent_control_pkg/src/gt2_fuzzy_logic_system.cpp",
        "lang": "cpp",
        "topic": "GT2 Fuzzy - Alpha-cuts, secondary MF",
        "lines": "ALL",
        "day": 2
    },
    {
        "num": 6,
        "name": "agent_controller_node.cpp",
        "path": "agent_control_pkg/src/ros/agent_controller_node.cpp",
        "lang": "cpp",
        "topic": "ROS2 Node - Hibrit kontrol, subscriber/publisher",
        "lines": "150-350",
        "day": 3
    },
    {
        "num": 7,
        "name": "formation_coordinator_node.cpp",
        "path": "other_packages/formation_coordinator_pkg/src/formation_coordinator_node.cpp",
        "lang": "cpp",
        "topic": "Formasyon - Waypoint uretimi, target_pose",
        "lines": "55-400",
        "day": 4
    },
    {
        "num": 8,
        "name": "wind_publisher.py",
        "path": "agent_control_pkg/scripts/wind_publisher.py",
        "lang": "python",
        "topic": "Ruzgar Modelleri - von Karman, Dryden, stochastic",
        "lines": "240-450",
        "day": 4
    },
    {
        "num": 9,
        "name": "phase_metrics_logger.py",
        "path": "agent_control_pkg/scripts/phase_metrics_logger.py",
        "lang": "python",
        "topic": "Metrik Toplama - CSV export, RMSE, ITAE",
        "lines": "277-625",
        "day": 4
    },
    {
        "num": 10,
        "name": "phased_comparison.launch.py",
        "path": "agent_control_pkg/launch/phased_comparison.launch.py",
        "lang": "python",
        "topic": "Launch - 6-Phase framework, node setup",
        "lines": "ALL",
        "day": 5
    },
]

def get_lexer(lang):
    """Get appropriate lexer for language."""
    if lang == "cpp":
        return CppLexer()
    elif lang == "python":
        return PythonLexer()
    elif lang == "yaml":
        return YamlLexer()
    else:
        return get_lexer_by_name(lang)

def generate_html(file_info, base_dir, output_dir):
    """Generate syntax-highlighted HTML for a single file."""
    num = file_info["num"]
    name = file_info["name"]
    rel_path = file_info["path"]
    lang = file_info["lang"]
    topic = file_info["topic"]
    lines_hint = file_info["lines"]
    day = file_info["day"]

    full_path = base_dir / rel_path

    if not full_path.exists():
        print(f"  [{num}] SKIP - File not found: {rel_path}")
        return None

    with open(full_path, 'r', encoding='utf-8') as f:
        code = f.read()

    total_lines = code.count('\n') + 1

    # Syntax highlighting
    lexer = get_lexer(lang)
    formatter = HtmlFormatter(
        style='vs',
        linenos='table',
        linenostart=1,
        cssclass='highlight',
        noclasses=False
    )

    highlighted = highlight(code, lexer, formatter)
    css = formatter.get_style_defs('.highlight')

    # HTML template
    html = f'''<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>{num:02d} - {name}</title>
    <style>
        @page {{ size: A4; margin: 1cm; }}
        @media print {{
            body {{ font-size: 8pt !important; }}
            .no-print {{ display: none; }}
            .header {{ page-break-after: avoid; }}
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
        .header {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            border-radius: 8px;
            margin-bottom: 20px;
        }}
        .header h1 {{
            margin: 0 0 10px 0;
            font-family: Arial, sans-serif;
            font-size: 20pt;
        }}
        .header .meta {{
            font-family: Arial, sans-serif;
            font-size: 10pt;
            opacity: 0.9;
        }}
        .badge {{
            display: inline-block;
            background: rgba(255,255,255,0.2);
            padding: 3px 10px;
            border-radius: 12px;
            margin-right: 10px;
            font-size: 9pt;
        }}
        .study-info {{
            background: #f8f9fa;
            border-left: 4px solid #667eea;
            padding: 15px;
            margin-bottom: 20px;
            font-family: Arial, sans-serif;
        }}
        .study-info strong {{ color: #333; }}
        .print-btn {{
            background: #667eea;
            color: white;
            border: none;
            padding: 10px 20px;
            font-size: 14px;
            cursor: pointer;
            border-radius: 5px;
            margin-bottom: 20px;
        }}
        .print-btn:hover {{ background: #5a6fd6; }}
        {css}
        .highlight {{
            background: white !important;
            border: 1px solid #ddd;
            border-radius: 4px;
            overflow-x: auto;
        }}
        .highlight pre {{ margin: 0; padding: 10px; }}
        .linenodiv {{
            background: #f5f5f5 !important;
            border-right: 1px solid #ddd !important;
            padding-right: 8px !important;
        }}
        .linenodiv pre {{ color: #999 !important; margin: 0; padding: 10px 5px; }}
        td.code {{ padding-left: 10px !important; }}
        /* Syntax colors - VS style */
        .highlight .c, .highlight .c1, .highlight .cm {{ color: #008000; }} /* Comment */
        .highlight .k, .highlight .kn, .highlight .kd {{ color: #0000ff; font-weight: bold; }} /* Keyword */
        .highlight .kt {{ color: #2b91af; }} /* Type */
        .highlight .nf, .highlight .nc {{ color: #795e26; }} /* Function/Class */
        .highlight .s, .highlight .s1, .highlight .s2 {{ color: #a31515; }} /* String */
        .highlight .mi, .highlight .mf {{ color: #098658; }} /* Number */
        .highlight .cp {{ color: #0000ff; }} /* Preprocessor */
        .highlight .nb {{ color: #2b91af; }} /* Builtin */
    </style>
</head>
<body>
    <div class="header">
        <h1>{num:02d}. {name}</h1>
        <div class="meta">
            <span class="badge">Day {day}</span>
            <span class="badge">{lang.upper()}</span>
            <span class="badge">{total_lines} lines</span>
            <span class="badge">Focus: {lines_hint}</span>
        </div>
    </div>

    <div class="study-info">
        <strong>Topic:</strong> {topic}<br>
        <strong>Path:</strong> {rel_path}<br>
        <strong>Recommended lines:</strong> {lines_hint}
    </div>

    <button class="print-btn no-print" onclick="window.print()">
        Print / Save as PDF (Ctrl+P)
    </button>

    {highlighted}
</body>
</html>'''

    # Save HTML
    output_file = output_dir / f"{num:02d}_{name.replace('.', '_')}.html"
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(html)

    print(f"  [{num:02d}] {name} ({total_lines} lines) -> {output_file.name}")
    return output_file

def generate_index(output_dir, generated_files):
    """Generate index.html with links to all files."""
    links = []
    for info, html_path in generated_files:
        if html_path:
            links.append(f'''
            <tr>
                <td>{info["num"]:02d}</td>
                <td><a href="{html_path.name}">{info["name"]}</a></td>
                <td>{info["topic"]}</td>
                <td>Day {info["day"]}</td>
                <td>{info["lines"]}</td>
            </tr>''')

    html = f'''<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>Study Materials - Multi-Agent Formation Control</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            max-width: 1000px;
            margin: 0 auto;
            padding: 20px;
            background: #f5f5f5;
        }}
        h1 {{
            color: #2c3e50;
            border-bottom: 3px solid #3498db;
            padding-bottom: 10px;
        }}
        .intro {{
            background: white;
            padding: 20px;
            border-radius: 8px;
            margin-bottom: 20px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            background: white;
            border-radius: 8px;
            overflow: hidden;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }}
        th {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 15px;
            text-align: left;
        }}
        td {{
            padding: 12px 15px;
            border-bottom: 1px solid #eee;
        }}
        tr:hover {{ background: #f8f9fa; }}
        a {{
            color: #3498db;
            text-decoration: none;
            font-weight: bold;
        }}
        a:hover {{ text-decoration: underline; }}
        .day-badge {{
            display: inline-block;
            background: #e8f4f8;
            color: #2980b9;
            padding: 3px 10px;
            border-radius: 12px;
            font-size: 12px;
        }}
    </style>
</head>
<body>
    <h1>Multi-Agent Formation Control - Study Materials</h1>

    <div class="intro">
        <p><strong>Based on:</strong> COMPLETE_SYSTEM_MANUAL.md - Ek A2: Kod Okuma Sirasi</p>
        <p><strong>Usage:</strong> Click on a file to view syntax-highlighted code. Use Ctrl+P to save as PDF.</p>
        <p><strong>Recommended study order:</strong> Day 1 -> Day 2 -> Day 3 -> Day 4 -> Day 5</p>
    </div>

    <table>
        <tr>
            <th>#</th>
            <th>File</th>
            <th>Topic</th>
            <th>Day</th>
            <th>Focus Lines</th>
        </tr>
        {''.join(links)}
    </table>
</body>
</html>'''

    index_path = output_dir / "index.html"
    with open(index_path, 'w', encoding='utf-8') as f:
        f.write(html)

    print(f"\n  INDEX -> {index_path.name}")
    return index_path

def main():
    base_dir = Path(__file__).parent.parent
    output_dir = base_dir / "study_materials"
    output_dir.mkdir(exist_ok=True)

    # Parse arguments
    open_browser = "--open" in sys.argv
    selected_nums = [int(arg) for arg in sys.argv[1:] if arg.isdigit()]

    if selected_nums:
        files_to_generate = [f for f in STUDY_FILES if f["num"] in selected_nums]
    else:
        files_to_generate = STUDY_FILES

    print(f"\n{'='*60}")
    print("  GENERATING STUDY MATERIALS")
    print(f"  Output: {output_dir}")
    print(f"{'='*60}\n")

    generated = []
    for file_info in files_to_generate:
        html_path = generate_html(file_info, base_dir, output_dir)
        generated.append((file_info, html_path))

    # Generate index
    index_path = generate_index(output_dir, generated)

    print(f"\n{'='*60}")
    print(f"  DONE! Generated {len([g for g in generated if g[1]])} files")
    print(f"  Open: {index_path}")
    print(f"{'='*60}\n")

    if open_browser or not selected_nums:
        print("Opening index in browser...")
        webbrowser.open(f'file:///{index_path}')

if __name__ == "__main__":
    main()