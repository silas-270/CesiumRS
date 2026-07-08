import xml.etree.ElementTree as ET
import re
import sys

def main():
    try:
        tree = ET.parse("flamegraph.svg")
        root = tree.getroot()
        
        functions = {}
        for element in root.iter():
            # SVG tags often have namespaces, e.g., {http://www.w3.org/2000/svg}title
            if element.tag.endswith('title'):
                text = element.text
                if text:
                    # typical format: "function_name (1,234 samples, 5.6%)"
                    # sometimes: "function_name (1,234 samples, 5.6%)" or similar
                    match = re.search(r'^(.*?) \(([\d,]+) samples, ([\d.]+)%\)', text)
                    if match:
                        func_name = match.group(1).strip()
                        samples = int(match.group(2).replace(',', ''))
                        pct = float(match.group(3))
                        
                        # filter out generic thread start / winit event loop overheads if they dominate
                        # but keep everything for the raw list
                        if func_name not in functions or samples > functions[func_name]['samples']:
                            functions[func_name] = {'samples': samples, 'pct': pct}
        
        sorted_funcs = sorted(functions.items(), key=lambda x: x[1]['samples'], reverse=True)
        
        print("Top CPU Time Consumers:")
        print("-" * 80)
        for f, data in sorted_funcs[:50]:
            print(f"{data['pct']:>5.2f}% ({data['samples']:>6})  {f}")
            
    except Exception as e:
        print(f"Error parsing SVG: {e}")

if __name__ == '__main__':
    main()
