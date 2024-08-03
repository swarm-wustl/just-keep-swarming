import re
from typing import Dict


def compile_sdf(content: str, subs: Dict[str, str]) -> str:
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


def compile_sdf_file(filename: str, subs: dict) -> str:
    try:
        with open(filename, "r") as f:
            return compile_sdf(f.read(), subs)
    except IOError as e:
        raise IOError(f"Error reading file {filename}: {e}")
