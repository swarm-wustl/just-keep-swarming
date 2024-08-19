import pytest

from simulation.utils.compile_sdf import compile_sdf, compile_sdf_file


def test_compile_sdf_basic_replacement():
    content = "Hello {% NAME %}!"
    subs = {"NAME": "World"}
    result = compile_sdf(content, subs)
    assert result == "Hello World!"


def test_compile_sdf_missing_substitution():
    content = "Hello {% NAME %}!"
    subs = {}  # No substitutions provided
    with pytest.raises(
        ValueError, match="NAME was not supplied a value for compilation"
    ):
        compile_sdf(content, subs)


def test_compile_sdf_unused_substitution():
    content = "Hello {% NAME %}!"
    subs = {"NAME": "World", "AGE": "30"}  # AGE is not used in the content
    with pytest.raises(ValueError, match="Unused substitutions: AGE"):
        compile_sdf(content, subs)


def test_compile_sdf_file(tmpdir):
    # Create a temporary file with sample content
    d = tmpdir.mkdir("sub")
    filename = d.join("test_file.sdf")
    with open(filename, "w", encoding="utf-8") as f:
        f.write("Hello {% NAME %}!")

    subs = {"NAME": "World"}
    result = compile_sdf_file(str(filename), subs)
    assert result == "Hello World!"


def test_remove_xml_tag(tmpdir):
    # Create a temporary file with sample content
    d = tmpdir.mkdir("sub")
    filename = d.join("test_file.sdf")
    with open(filename, "w", encoding="utf-8") as f:
        f.write('<?xml version="1.0"?>\nHello {% NAME %}!')
    subs = {"NAME": "World"}
    result = compile_sdf_file(str(filename), subs, remove_xml_tag=True)
    assert result == "Hello World!"


def test_compile_sdf_file_read_error(tmpdir):
    filename = str(tmpdir.join("non_existent_file.sdf"))
    subs = {"NAME": "World"}
    with pytest.raises(IOError, match=f"Error reading file {filename}"):
        compile_sdf_file(filename, subs)
