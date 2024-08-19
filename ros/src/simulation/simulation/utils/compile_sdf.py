"""
Provides methods to replace variable syntax {% VARIABLE %} in an SDF file with
provided variable values
"""

import re
from typing import Dict


def compile_sdf(content: str, subs: Dict[str, str]) -> str:
    """Replace variables in `content` with values in `subs`"""
    pattern = re.compile(r"{%\s(\w+)\s%}")

    def replace_match(match):
        key = match.group(1)
        if key not in subs:
            raise ValueError(f"{key} was not supplied a value for compilation")
        return str(subs[key])

    result = pattern.sub(replace_match, content)

    unused_subs = set(subs.keys()) - set(pattern.findall(content))
    if unused_subs:
        raise ValueError(f"Unused substitutions: {', '.join(unused_subs)}")

    return result


def compile_sdf_file(filename: str, subs: dict, remove_xml_tag: bool = False) -> str:
    """
    Calls `compile_sdf` on a given file

    :param bool remove_xml_tag If true, removes the `<?xml ... >` tag from the file
    """
    try:
        with open(filename, "r", encoding="utf-8") as f:
            file = f.read()
            if remove_xml_tag:
                file = file.replace('<?xml version="1.0"?>\n', "")
            return compile_sdf(file, subs)
    except IOError as e:
        raise IOError(f"Error reading file {filename}: {e}") from e
